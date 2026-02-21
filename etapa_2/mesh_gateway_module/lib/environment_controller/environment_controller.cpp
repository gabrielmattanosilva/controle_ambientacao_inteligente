/**
 * @file environment_controller.cpp
 * @brief Implementação do controle automático simples (setpoints hardcoded).
 */

#include "environment_controller.h"

#include <Arduino.h>
#include <time.h>
#include <string.h>

#include "mesh_proto.h"
#include "logger.h"

static const char *TAG = "ENV_CTRL";

#define ENV_NODE_SRC            "msh-gw"
#define ENV_NODE_DST_ACT        "act-00"
#define ENV_NODE_INT            "int-sen-00"
#define ENV_NODE_EXT            "ext-sen-00"

// ---- Setpoints principais
static const float ENV_TEMP_SP_C         = 27.0f;
static const float ENV_TEMP_HYST_C       = 2.0f;

static const int   ENV_LUX_SP            = 1500;   // lux desejado no sensor interno
static const int   ENV_LUX_HYST          = 200;

// ---- Ventilação/exaustão (valores de PWM em %)
static const int   ENV_FAN_INTAKE_COOL   = 100;
static const int   ENV_FAN_EXHAUST_COOL  = 100;
static const int   ENV_FAN_INTAKE_IDLE   = 0;
static const int   ENV_FAN_EXHAUST_IDLE  = 0;

// ---- Iluminação (PWM em %)
static const int   ENV_LED_ON_PWM        = 100;
static const int   ENV_LED_OFF_PWM       = 0;

// ---- Cor do LED hardcoded
static const char *ENV_LED_RGB           = "ffffff"; // RRGGBB

// ---- Umidificação / Irrigação (limiares)
static const int   ENV_SOIL_MIN          = 40;     // abaixo disso pode irrigar
static const float ENV_RH_MIN            = 55.0f;  // abaixo disso pode umidificar

// ---- Orçamento diário (tempo TOTAL de ON por dia)
static const uint32_t ENV_IRRIGATION_BUDGET_MS = 1UL * 60UL * 1000UL;  // 1 min/dia (tempo ON somado)
static const uint32_t ENV_HUMIDIFIER_BUDGET_MS = 5UL * 60UL * 1000UL;  // 5 min/dia (tempo ON somado)

// ---- Janelas diárias (minutos desde 00:00, horário local)
static const int ENV_IRRIGATION_START_MIN   = 8 * 60;
static const int ENV_IRRIGATION_WINDOW_MIN  = 60;

static const int ENV_HUMIDIFIER_START_MIN   = 12 * 60;
static const int ENV_HUMIDIFIER_WINDOW_MIN  = 120;

// ---- Segurança: duty-cycle obrigatório
static const uint32_t ENV_PUMP_ON_MAX_MS        = 30UL * 1000UL;
static const uint32_t ENV_PUMP_OFF_MIN_MS       = 30UL * 1000UL;

static const uint32_t ENV_HUM_ON_MAX_MS         = 30UL * 1000UL;
static const uint32_t ENV_HUM_OFF_MIN_MS        = 60UL * 1000UL;

// ---- Freshness (evita atuar sem telemetria atual)
static const uint32_t ENV_TELE_FRESH_MS         = 5UL * 60UL * 1000UL; // 5 min

// Força envio após telemetria (mesmo sem mudança)
static bool g_force_tx = false;

/* ============================================================================
 * ESTRUTURAS
 * ============================================================================
 */

typedef struct
{
    bool has_t_in;
    bool has_rh_in;
    bool has_lux_in;
    bool has_soil;

    float t_in;
    float rh_in;
    int   lux_in;
    int   soil;

    bool has_t_out;
    float t_out;

    uint32_t last_update_ms;
} env_sensors_t;

typedef struct
{
    bool output_on;
    bool desired_on;
    uint32_t state_start_ms;
    uint32_t on_max_ms;
    uint32_t off_min_ms;
} env_cycle_limiter_t;

typedef struct
{
    int mode; // 0 auto, 1 manual

    int manual_irrigation_req;
    int manual_humidifier_req;

    uint32_t irrigation_on_acc_ms;
    uint32_t humidifier_on_acc_ms;

    uint32_t last_poll_ms;
    int      day_key;

    env_cycle_limiter_t irrigation_limiter;
    env_cycle_limiter_t humidifier_limiter;

    bool has_last_sent_intake;
    bool has_last_sent_exhaust;
    bool has_last_sent_led;
    bool has_last_sent_led_rgb;
    bool has_last_sent_irrig;
    bool has_last_sent_hum;

    int last_sent_intake_pwm;
    int last_sent_exhaust_pwm;
    int last_sent_led_pwm;
    char last_sent_led_rgb[16];
    int last_sent_irrig;
    int last_sent_hum;
} env_ctrl_t;

/* ============================================================================
 * ESTADO GLOBAL
 * ============================================================================
 */

static env_ctrl_send_cb_t g_send_cb = nullptr;
static uint16_t g_id_counter = 0;

static env_sensors_t g_sensors = {0};
static env_ctrl_t    g_ctrl    = {0};

/* ============================================================================
 * HELPERS
 * ============================================================================
 */

static void env_gen_id(char *buf, size_t len)
{
    g_id_counter++;
    if (!g_id_counter) g_id_counter = 1;
    snprintf(buf, len, "%04X", (unsigned)g_id_counter);
}

static int clampi(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool epoch_is_reasonable(time_t t)
{
    return (t >= (time_t)1609459200);
}

static int minute_of_day_from_time(time_t now_epoch, uint32_t now_ms)
{
    if (epoch_is_reasonable(now_epoch))
    {
        struct tm tm_local;
        localtime_r(&now_epoch, &tm_local);
        return (tm_local.tm_hour * 60) + tm_local.tm_min;
    }
    return (int)((now_ms / 60000UL) % 1440UL);
}

static int day_key_from_time(time_t now_epoch, uint32_t now_ms)
{
    if (epoch_is_reasonable(now_epoch))
    {
        struct tm tm_local;
        localtime_r(&now_epoch, &tm_local);
        return (tm_local.tm_year + 1900) * 1000 + tm_local.tm_yday;
    }
    return (int)(now_ms / (24UL * 60UL * 60UL * 1000UL));
}

static bool within_window(int now_min, int start_min, int window_min)
{
    if (window_min <= 0) return false;

    int end = start_min + window_min;
    if (end <= 1440) return (now_min >= start_min) && (now_min < end);

    end -= 1440;
    return (now_min >= start_min) || (now_min < end);
}

static bool sensors_fresh(uint32_t now_ms)
{
    if (g_sensors.last_update_ms == 0) return false;
    return (now_ms - g_sensors.last_update_ms) <= ENV_TELE_FRESH_MS;
}

static void cycle_limiter_init(env_cycle_limiter_t *c, uint32_t on_max_ms, uint32_t off_min_ms)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->output_on = false;
    c->desired_on = false;
    c->on_max_ms = on_max_ms;
    c->off_min_ms = off_min_ms;

    uint32_t now_ms = millis();
    c->state_start_ms = now_ms - c->off_min_ms;
}

static void cycle_limiter_update(env_cycle_limiter_t *c, uint32_t now_ms)
{
    if (!c) return;

    uint32_t elapsed = now_ms - c->state_start_ms;

    if (c->output_on)
    {
        if (elapsed >= c->on_max_ms || !c->desired_on)
        {
            c->output_on = false;
            c->state_start_ms = now_ms;
        }
    }
    else
    {
        if (c->desired_on && elapsed >= c->off_min_ms)
        {
            c->output_on = true;
            c->state_start_ms = now_ms;
        }
    }
}

static void send_cfg_int_if_needed(bool *has_last, int *last_val,
                                  const char *field, int value)
{
    if (!g_send_cb || !field) return;

    if (!g_force_tx && *has_last && (*last_val == value))
        return;

    char id[8];
    char json[256];

    env_gen_id(id, sizeof(id));

    if (!mesh_proto_build_cfg_int(id,
                                  (uint32_t)millis(),
                                  0,
                                  ENV_NODE_SRC,
                                  ENV_NODE_DST_ACT,
                                  field,
                                  value,
                                  json,
                                  sizeof(json)))
    {
        LOG(TAG, "Falha ao montar CFG int field=%s", field);
        return;
    }

    *has_last = true;
    *last_val = value;

    g_send_cb(json);
    LOG(TAG, "TX %s=%d -> %s%s", field, value, ENV_NODE_DST_ACT, g_force_tx ? " (force)" : "");
}

static void send_cfg_str_if_needed(bool *has_last,
                                  char *last_val,
                                  size_t last_len,
                                  const char *field,
                                  const char *value)
{
    if (!g_send_cb || !field || !value || !has_last || !last_val || last_len == 0)
        return;

    if (!g_force_tx && *has_last && (strncmp(last_val, value, last_len) == 0))
        return;

    char id[8];
    char json[256];

    env_gen_id(id, sizeof(id));

    if (!mesh_proto_build_cfg_str(id,
                                  (uint32_t)millis(),
                                  0,
                                  ENV_NODE_SRC,
                                  ENV_NODE_DST_ACT,
                                  field,
                                  value,
                                  json,
                                  sizeof(json)))
    {
        LOG(TAG, "Falha ao montar CFG str field=%s", field);
        return;
    }

    strncpy(last_val, value, last_len - 1);
    last_val[last_len - 1] = '\0';
    *has_last = true;

    g_send_cb(json);
    LOG(TAG, "TX %s=%s -> %s%s", field, value, ENV_NODE_DST_ACT, g_force_tx ? " (force)" : "");
}

/* ============================================================================
 * API
 * ============================================================================
 */

void environment_controller_init(env_ctrl_send_cb_t send_cb)
{
    g_send_cb = send_cb;
    g_id_counter = 0;

    memset(&g_sensors, 0, sizeof(g_sensors));
    memset(&g_ctrl, 0, sizeof(g_ctrl));

    g_ctrl.mode = 0; // AUTO default
    g_ctrl.manual_humidifier_req = 0;
    g_ctrl.manual_irrigation_req = 0;

    g_ctrl.last_poll_ms = millis();
    g_ctrl.day_key = day_key_from_time(time(nullptr), g_ctrl.last_poll_ms);

    cycle_limiter_init(&g_ctrl.irrigation_limiter, ENV_PUMP_ON_MAX_MS, ENV_PUMP_OFF_MIN_MS);
    cycle_limiter_init(&g_ctrl.humidifier_limiter, ENV_HUM_ON_MAX_MS, ENV_HUM_OFF_MIN_MS);

    // força um envio inicial quando entrar AUTO + tiver telemetria
    g_force_tx = true;

    LOG(TAG, "init ok (AUTO default). SP: T=%.1fC lux=%d soil_min=%d rh_min=%.1f",
        ENV_TEMP_SP_C, ENV_LUX_SP, ENV_SOIL_MIN, ENV_RH_MIN);
}

void environment_controller_on_mesh_msg(const mesh_msg_t *msg)
{
    if (!msg) return;

    uint32_t now_ms = millis();

    if (msg->type == MESH_MSG_TELE)
    {
        if (strcmp(msg->src, ENV_NODE_INT) == 0)
        {
            if (msg->tele.has_t_in)   { g_sensors.has_t_in = true;   g_sensors.t_in = msg->tele.t_in; }
            if (msg->tele.has_rh_in)  { g_sensors.has_rh_in = true;  g_sensors.rh_in = msg->tele.rh_in; }
            if (msg->tele.has_lux_in) { g_sensors.has_lux_in = true; g_sensors.lux_in = msg->tele.lux_in; }
            if (msg->tele.has_soil_moist) { g_sensors.has_soil = true; g_sensors.soil = msg->tele.soil_moist; }

            g_sensors.last_update_ms = now_ms;

            g_force_tx = true;
        }
        else if (strcmp(msg->src, ENV_NODE_EXT) == 0)
        {
            if (msg->tele.has_t_out) { g_sensors.has_t_out = true; g_sensors.t_out = msg->tele.t_out; }
            g_sensors.last_update_ms = now_ms;

            g_force_tx = true;
        }
    }
}

bool environment_controller_on_uart_msg(const mesh_msg_t *msg)
{
    if (!msg) return true;
    if (msg->type != MESH_MSG_CFG) return true;

    if (msg->cfg.has_mode)
    {
        g_ctrl.mode = msg->cfg.mode ? 1 : 0;

        // ao trocar modo, força reenvio (garante led_rgb hardcoded etc.)
        g_force_tx = true;
        g_ctrl.has_last_sent_led_rgb = false;

        LOG(TAG, "mode atualizado via UART: %d", g_ctrl.mode);
        return true;
    }

    // ignora led_rgb vindo do Blynk e força "ffffff"
    if (msg->cfg.has_led_rgb)
    {
        send_cfg_str_if_needed(&g_ctrl.has_last_sent_led_rgb,
                               g_ctrl.last_sent_led_rgb,
                               sizeof(g_ctrl.last_sent_led_rgb),
                               "led_rgb",
                               ENV_LED_RGB);
        return false;
    }

    // MANUAL: duty-cycle seguro (engole irrig/humid)
    if (g_ctrl.mode == 1)
    {
        bool swallowed = false;

        if (msg->cfg.has_irrigation)
        {
            g_ctrl.manual_irrigation_req = msg->cfg.irrigation ? 1 : 0;
            swallowed = true;
        }

        if (msg->cfg.has_humidifier)
        {
            g_ctrl.manual_humidifier_req = msg->cfg.humidifier ? 1 : 0;
            swallowed = true;
        }

        if (swallowed) return false;
    }

    return true;
}

void environment_controller_poll(void)
{
    if (!g_send_cb) return;

    uint32_t now_ms = millis();
    time_t now_epoch = time(nullptr);

    uint32_t delta_ms = now_ms - g_ctrl.last_poll_ms;
    g_ctrl.last_poll_ms = now_ms;

    int dk = day_key_from_time(now_epoch, now_ms);
    if (dk != g_ctrl.day_key)
    {
        g_ctrl.day_key = dk;
        g_ctrl.irrigation_on_acc_ms = 0;
        g_ctrl.humidifier_on_acc_ms = 0;
        g_force_tx = true;
    }

    if (g_ctrl.irrigation_limiter.output_on) g_ctrl.irrigation_on_acc_ms += delta_ms;
    if (g_ctrl.humidifier_limiter.output_on) g_ctrl.humidifier_on_acc_ms += delta_ms;

    bool irr_desired = false;
    bool hum_desired = false;

    if (g_ctrl.mode == 0)
    {
        if (sensors_fresh(now_ms))
        {
            int now_min = minute_of_day_from_time(now_epoch, now_ms);

            bool irr_window = within_window(now_min, ENV_IRRIGATION_START_MIN, ENV_IRRIGATION_WINDOW_MIN);
            bool irr_budget_ok = g_ctrl.irrigation_on_acc_ms < ENV_IRRIGATION_BUDGET_MS;
            bool irr_need = g_sensors.has_soil && (g_sensors.soil < ENV_SOIL_MIN);
            irr_desired = irr_window && irr_budget_ok && irr_need;

            bool hum_window = within_window(now_min, ENV_HUMIDIFIER_START_MIN, ENV_HUMIDIFIER_WINDOW_MIN);
            bool hum_budget_ok = g_ctrl.humidifier_on_acc_ms < ENV_HUMIDIFIER_BUDGET_MS;
            bool hum_need = g_sensors.has_rh_in && (g_sensors.rh_in < ENV_RH_MIN);
            hum_desired = hum_window && hum_budget_ok && hum_need;
        }
        else
        {
            irr_desired = false;
            hum_desired = false;
        }
    }
    else
    {
        irr_desired = (g_ctrl.manual_irrigation_req != 0);
        hum_desired = (g_ctrl.manual_humidifier_req != 0);
    }

    g_ctrl.irrigation_limiter.desired_on = irr_desired;
    g_ctrl.humidifier_limiter.desired_on = hum_desired;

    cycle_limiter_update(&g_ctrl.irrigation_limiter, now_ms);
    cycle_limiter_update(&g_ctrl.humidifier_limiter, now_ms);

    // AUTO: fans e leds
    if (g_ctrl.mode == 0)
    {
        int intake = ENV_FAN_INTAKE_IDLE;
        int exhaust = ENV_FAN_EXHAUST_IDLE;
        int led_pwm = ENV_LED_OFF_PWM;

        if (sensors_fresh(now_ms) && g_sensors.has_t_in)
        {
            float t_in = g_sensors.t_in;

            if (t_in > (ENV_TEMP_SP_C + ENV_TEMP_HYST_C))
            {
                if (g_sensors.has_t_out && (g_sensors.t_out <= t_in))
                    intake = ENV_FAN_INTAKE_COOL;
                else
                    intake = (ENV_FAN_INTAKE_COOL * 60) / 100;

                exhaust = ENV_FAN_EXHAUST_COOL;
            }
            else if (t_in < (ENV_TEMP_SP_C - ENV_TEMP_HYST_C))
            {
                intake = ENV_FAN_INTAKE_IDLE;
                exhaust = ENV_FAN_EXHAUST_IDLE;
            }
        }

        if (sensors_fresh(now_ms) && g_sensors.has_lux_in)
        {
            int lux = g_sensors.lux_in;

            if (lux < (ENV_LUX_SP - ENV_LUX_HYST))
                led_pwm = ENV_LED_ON_PWM;
            else if (lux > (ENV_LUX_SP + ENV_LUX_HYST))
                led_pwm = ENV_LED_OFF_PWM;
        }

        intake = clampi(intake, 0, 100);
        exhaust = clampi(exhaust, 0, 100);
        led_pwm = clampi(led_pwm, 0, 100);

        send_cfg_int_if_needed(&g_ctrl.has_last_sent_intake, &g_ctrl.last_sent_intake_pwm, "intake_pwm", intake);
        send_cfg_int_if_needed(&g_ctrl.has_last_sent_exhaust, &g_ctrl.last_sent_exhaust_pwm, "exhaust_pwm", exhaust);
        send_cfg_int_if_needed(&g_ctrl.has_last_sent_led, &g_ctrl.last_sent_led_pwm, "led_pwm", led_pwm);

        send_cfg_str_if_needed(&g_ctrl.has_last_sent_led_rgb,
                               g_ctrl.last_sent_led_rgb,
                               sizeof(g_ctrl.last_sent_led_rgb),
                               "led_rgb",
                               ENV_LED_RGB);
    }

    int irr_out = g_ctrl.irrigation_limiter.output_on ? 1 : 0;
    int hum_out = g_ctrl.humidifier_limiter.output_on ? 1 : 0;

    send_cfg_int_if_needed(&g_ctrl.has_last_sent_irrig, &g_ctrl.last_sent_irrig, "irrigation", irr_out);
    send_cfg_int_if_needed(&g_ctrl.has_last_sent_hum, &g_ctrl.last_sent_hum, "humidifier", hum_out);

    g_force_tx = false;
}
