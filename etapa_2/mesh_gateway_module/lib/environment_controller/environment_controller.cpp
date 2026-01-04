/**
 * @file environment_controller.cpp
 * @brief Controle ambiental para caixa/estufa (BH1750 + RTC).
 */

#include "environment_controller.h"

#include <Arduino.h>
#include <string.h>
#include <time.h>

#include "environment_config.h"
#include "mesh_proto.h"
#include "logger.h"

static const char *TAG = "ENV_CTRL";

/* =========================
 * Estado interno
 * ========================= */

static env_send_cb_t g_send_cb = nullptr;
static env_mode_t g_mode = ENV_MODE_AUTO;

static uint16_t g_id_counter = 0;

typedef struct {
    bool has_t;
    bool has_rh;
    bool has_lux;
    bool has_soil;

    float t;
    float rh;
    int   lux;
    int   soil;

    uint32_t last_ms;
} int_tele_t;

typedef struct {
    bool has_t;
    bool has_rh;
    bool has_lux;

    float t;
    float rh;
    int   lux;

    uint32_t last_ms;
} ext_tele_t;

static int_tele_t g_int = {0};
static ext_tele_t g_ext = {0};

typedef struct {
    int intake_pwm;
    int exhaust_pwm;
    int humidifier;   // 0/1
    int irrigation;   // 0/1
    int led_pwm;      // 0..255
    char led_rgb[16]; // "#RRGGBB"
} outputs_t;

static outputs_t g_out = {0};
static outputs_t g_last_sent = {0};

static uint32_t g_last_tick_ms = 0;
static uint32_t g_last_force_send_ms = 0;

/* Irrig FSM */
typedef enum { IRR_IDLE=0, IRR_ON=1 } irr_state_t;
static irr_state_t g_irr_state = IRR_IDLE;
static uint32_t g_irr_on_since_ms = 0;
static uint32_t g_irr_last_done_ms = 0;

/* Humidifier anti-chatter */
static uint32_t g_hum_last_toggle_ms = 0;
static int g_hum_state = 0;
static const uint32_t HUM_MIN_ONOFF_MS = 20000UL;

/* LED controller */
static int g_led_pwm_cmd = 0;

/* =========================
 * Helpers
 * ========================= */

static inline float ema_f(float prev, float x, float alpha) {
    return (prev == 0.0f) ? x : (alpha * x + (1.0f - alpha) * prev);
}

static inline int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float clamp_f(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void gen_id(char *buf, size_t len) {
    g_id_counter++;
    if (!g_id_counter) g_id_counter = 1;
    snprintf(buf, len, "%04X", (unsigned)g_id_counter);
}

static bool is_stale_int(uint32_t now_ms) {
    return (g_int.last_ms == 0) || ((now_ms - g_int.last_ms) > ENV_STALE_INT_MS);
}

static bool is_stale_ext(uint32_t now_ms) {
    return (g_ext.last_ms == 0) || ((now_ms - g_ext.last_ms) > ENV_STALE_EXT_MS);
}

/* fotoperíodo usando RTC interno (time()) */
static bool light_should_be_on(void) {
    time_t now = time(nullptr);
    struct tm tm_local;
    localtime_r(&now, &tm_local);
    int h = tm_local.tm_hour;

    if (ENV_LIGHT_ON_HOUR < ENV_LIGHT_OFF_HOUR) {
        return (h >= ENV_LIGHT_ON_HOUR) && (h < ENV_LIGHT_OFF_HOUR);
    }
    /* caso invertido (ex.: 20->8) */
    return (h >= ENV_LIGHT_ON_HOUR) || (h < ENV_LIGHT_OFF_HOUR);
}

/* retorna fator 0..1 para rampa sunrise/sunset */
static float light_ramp_factor(bool on) {
    /* Se não tiver hora válida, localtime pode cair em epoch0;
       mesmo assim o sistema funciona (só segue o horário fixo).
       Para robustez, a rampa é sempre baseada em “tempo desde mudança” via millis().
    */
    static bool s_prev_on = false;
    static uint32_t s_changed_ms = 0;

    uint32_t now_ms = millis();
    if (on != s_prev_on) {
        s_prev_on = on;
        s_changed_ms = now_ms;
    }

    uint32_t dt = now_ms - s_changed_ms;
    float a = (ENV_LIGHT_RAMP_SEC > 0) ? (float)dt / (float)(ENV_LIGHT_RAMP_SEC * 1000UL) : 1.0f;
    a = clamp_f(a, 0.0f, 1.0f);

    return on ? a : (1.0f - a);
}

static int map_clamp_i(float x, float x0, float x1, int y0, int y1) {
    if (x <= x0) return y0;
    if (x >= x1) return y1;
    float t = (x - x0) / (x1 - x0);
    float y = (float)y0 + t * (float)(y1 - y0);
    return (int)(y + 0.5f);
}

/* envia um cfg (1 campo) para act-00 */
static void send_cfg_int(const char *field, int value) {
    if (!g_send_cb) return;
    char id[8];
    char json[256];
    gen_id(id, sizeof(id));

    if (mesh_proto_build_cfg_int(id,
                                 (uint32_t)millis(),
                                 0, /* qos0 */
                                 ENV_NODE_MSH_GW,
                                 ENV_NODE_ACT,
                                 field,
                                 value,
                                 json,
                                 sizeof(json))) {
        g_send_cb(json);
    }
}

static void send_cfg_str(const char *field, const char *value) {
    if (!g_send_cb) return;
    char id[8];
    char json[256];
    gen_id(id, sizeof(id));

    if (mesh_proto_build_cfg_str(id,
                                 (uint32_t)millis(),
                                 0, /* qos0 */
                                 ENV_NODE_MSH_GW,
                                 ENV_NODE_ACT,
                                 field,
                                 value,
                                 json,
                                 sizeof(json))) {
        g_send_cb(json);
    }
}

/* decide se vale enviar (deadband) */
static bool diff_pwm(int a, int b) { return abs(a - b) >= 5; }

static void maybe_send_outputs(uint32_t now_ms, bool force) {
    bool changed =
        diff_pwm(g_out.intake_pwm,  g_last_sent.intake_pwm)  ||
        diff_pwm(g_out.exhaust_pwm, g_last_sent.exhaust_pwm) ||
        (g_out.humidifier != g_last_sent.humidifier)         ||
        (g_out.irrigation != g_last_sent.irrigation)         ||
        diff_pwm(g_out.led_pwm,     g_last_sent.led_pwm)     ||
        (strncmp(g_out.led_rgb, g_last_sent.led_rgb, sizeof(g_out.led_rgb)) != 0);

    if (!force && !changed) return;

    /* Envia campo a campo (compatível com o projeto atual) */
    send_cfg_int("intake_pwm",  g_out.intake_pwm);
    send_cfg_int("exhaust_pwm", g_out.exhaust_pwm);
    send_cfg_int("humidifier",  g_out.humidifier);
    send_cfg_int("irrigation",  g_out.irrigation);
    send_cfg_int("led_pwm",     g_out.led_pwm);
    send_cfg_str("led_rgb",     g_out.led_rgb);

    g_last_sent = g_out;
    g_last_force_send_ms = now_ms;
}

/* =========================
 * Lógicas de controle
 * ========================= */

static void set_failsafe(void) {
    /* Failsafe conservador: desliga água/umidificador e luz, e ventila leve. */
    g_out.irrigation = 0;
    g_out.humidifier = 0;
    g_out.led_pwm = 0;
    strncpy(g_out.led_rgb, "#000000", sizeof(g_out.led_rgb));

    g_out.intake_pwm  = 30;
    g_out.exhaust_pwm = 30;
}

static int led_control_step(bool light_on, float t_in, int lux_in) {
    int target = light_on ? ENV_LUX_TARGET_DAY : ENV_LUX_TARGET_NIGHT;

    /* Proporcional simples em torno do alvo (BH1750) */
    float err = (float)target - (float)lux_in;
    int delta = (int)(ENV_LED_KP * err);

    int cmd = clamp_i(g_led_pwm_cmd + delta, 0, ENV_LED_PWM_MAX);

    /* Slew rate */
    int max_step = ENV_LED_SLEW_PER_SEC; // por tick de 1s
    int step = cmd - g_led_pwm_cmd;
    step = clamp_i(step, -max_step, max_step);
    g_led_pwm_cmd = clamp_i(g_led_pwm_cmd + step, 0, ENV_LED_PWM_MAX);

    /* Rampa sunrise/sunset */
    float ramp = light_ramp_factor(light_on);
    int ramped = (int)((float)g_led_pwm_cmd * ramp + 0.5f);

    /* Derating por temperatura */
    if (t_in >= ENV_T_LED_CUTOFF) {
        ramped = 0;
    } else if (t_in > ENV_T_LED_DERATE) {
        float f = 1.0f - (t_in - ENV_T_LED_DERATE) / (ENV_T_LED_CUTOFF - ENV_T_LED_DERATE);
        f = clamp_f(f, 0.0f, 1.0f);
        ramped = (int)((float)ramped * f + 0.5f);
    }

    return clamp_i(ramped, 0, ENV_LED_PWM_MAX);
}

static int vent_pwm_from_temp(float t_in, bool can_cool) {
    if (!can_cool) return 0;
    if (t_in <= (ENV_T_SP + ENV_T_HYST)) return 0;

    float over = t_in - (ENV_T_SP + ENV_T_HYST);
    /* 0..5°C -> 0..200 PWM */
    int pwm = map_clamp_i(over, 0.0f, 5.0f, 0, 200);
    return clamp_i(pwm, 0, ENV_PWM_MAX);
}

static int vent_pwm_from_rh(float rh_in, bool can_dehum) {
    if (!can_dehum) return 0;

    if (rh_in <= (ENV_RH_SP_VENT + ENV_RH_HYST)) return 0;

    float over = rh_in - (ENV_RH_SP_VENT + ENV_RH_HYST);
    /* 0..20% -> 0..200 PWM */
    int pwm = map_clamp_i(over, 0.0f, 20.0f, 0, 200);
    return clamp_i(pwm, 0, ENV_PWM_MAX);
}

static int humidifier_logic(float rh_in, int fan_pwm, uint32_t now_ms) {
    int want = g_hum_state;

    /* teto duro */
    if (rh_in >= ENV_RH_MAX) {
        want = 0;
    } else {
        if (g_hum_state == 0) {
            if (rh_in < ENV_RH_LOW) {
                /* lockout: se ventilando forte, não liga */
                if (fan_pwm < 80) want = 1;
            }
        } else {
            if (rh_in > ENV_RH_HIGH) {
                want = 0;
            }
            if (fan_pwm >= 100) {
                want = 0;
            }
        }
    }

    /* anti-chatter */
    if (want != g_hum_state) {
        if ((now_ms - g_hum_last_toggle_ms) >= HUM_MIN_ONOFF_MS) {
            g_hum_state = want;
            g_hum_last_toggle_ms = now_ms;
        }
    }

    return g_hum_state;
}

static int irrigation_fsm(int soil, bool soil_valid, uint32_t now_ms) {
    if (!soil_valid) {
        g_irr_state = IRR_IDLE;
        return 0;
    }

    switch (g_irr_state) {
        case IRR_IDLE:
            if (soil < ENV_SOIL_DRY_TH) {
                if (g_irr_last_done_ms == 0 ||
                    (now_ms - g_irr_last_done_ms) >= ENV_IRRIG_MIN_INTERVAL_MS) {
                    g_irr_state = IRR_ON;
                    g_irr_on_since_ms = now_ms;
                    return 1;
                }
            }
            return 0;

        case IRR_ON: {
            uint32_t on_dt = now_ms - g_irr_on_since_ms;
            if (soil > ENV_SOIL_WET_TH || on_dt >= ENV_IRRIG_MAX_ON_MS) {
                g_irr_state = IRR_IDLE;
                g_irr_last_done_ms = now_ms;
                return 0;
            }
            return 1;
        }
        default:
            g_irr_state = IRR_IDLE;
            return 0;
    }
}

/* =========================
 * API pública
 * ========================= */

void env_ctrl_init(env_send_cb_t send_cb) {
    g_send_cb = send_cb;
    g_mode = ENV_MODE_AUTO;

    memset(&g_int, 0, sizeof(g_int));
    memset(&g_ext, 0, sizeof(g_ext));
    memset(&g_out, 0, sizeof(g_out));
    memset(&g_last_sent, 0, sizeof(g_last_sent));

    strncpy(g_out.led_rgb, "#FFFFFF", sizeof(g_out.led_rgb));
    strncpy(g_last_sent.led_rgb, "#FFFFFF", sizeof(g_last_sent.led_rgb));

    g_id_counter = 0;
    g_last_tick_ms = 0;
    g_last_force_send_ms = 0;

    g_irr_state = IRR_IDLE;
    g_irr_on_since_ms = 0;
    g_irr_last_done_ms = 0;

    g_hum_last_toggle_ms = 0;
    g_hum_state = 0;

    g_led_pwm_cmd = 0;

    LOG(TAG, "init OK (mode=AUTO)");
}

void env_ctrl_set_mode(env_mode_t mode) {
    if (mode != ENV_MODE_AUTO && mode != ENV_MODE_MANUAL) return;

    if (g_mode != mode) {
        g_mode = mode;
        LOG(TAG, "mode -> %s", (g_mode == ENV_MODE_AUTO) ? "AUTO" : "MANUAL");

        /* Ao entrar em MANUAL, não mexe nos atuadores (não envia nada).
           Ao voltar para AUTO, força refresh imediato. */
        if (g_mode == ENV_MODE_AUTO) {
            g_last_force_send_ms = 0;
        }
    }
}

env_mode_t env_ctrl_get_mode(void) {
    return g_mode;
}

void env_ctrl_on_mesh_msg(const mesh_msg_t *msg) {
    if (!msg) return;
    if (msg->type != MESH_MSG_TELE) return;

    uint32_t now_ms = millis();

    /* TELE interna */
    if (strcmp(msg->src, ENV_NODE_INT) == 0) {
        if (msg->tele.has_t_in) {
            g_int.t = ema_f(g_int.t, msg->tele.t_in, ENV_EMA_ALPHA_T);
            g_int.has_t = true;
        }
        if (msg->tele.has_rh_in) {
            g_int.rh = ema_f(g_int.rh, msg->tele.rh_in, ENV_EMA_ALPHA_RH);
            g_int.has_rh = true;
        }
        if (msg->tele.has_lux_in) {
            float prev = (float)g_int.lux;
            float cur  = (float)msg->tele.lux_in;
            float filt = (prev == 0.0f) ? cur : (ENV_EMA_ALPHA_LUX * cur + (1.0f - ENV_EMA_ALPHA_LUX) * prev);
            g_int.lux = (int)(filt + 0.5f);
            g_int.has_lux = true;
        }
        if (msg->tele.has_soil_moist) {
            float prev = (float)g_int.soil;
            float cur  = (float)msg->tele.soil_moist;
            float filt = (prev == 0.0f) ? cur : (ENV_EMA_ALPHA_SOIL * cur + (1.0f - ENV_EMA_ALPHA_SOIL) * prev);
            g_int.soil = (int)(filt + 0.5f);
            g_int.has_soil = true;
        }
        g_int.last_ms = now_ms;
    }

    /* TELE externa (opcional) */
    if (strcmp(msg->src, ENV_NODE_EXT) == 0) {
        if (msg->tele.has_t_out) {
            g_ext.t = ema_f(g_ext.t, msg->tele.t_out, ENV_EMA_ALPHA_T);
            g_ext.has_t = true;
        }
        if (msg->tele.has_rh_out) {
            g_ext.rh = ema_f(g_ext.rh, msg->tele.rh_out, ENV_EMA_ALPHA_RH);
            g_ext.has_rh = true;
        }
        if (msg->tele.has_lux_out) {
            g_ext.lux = msg->tele.lux_out;
            g_ext.has_lux = true;
        }
        g_ext.last_ms = now_ms;
    }
}

void env_ctrl_tick(uint32_t now_ms) {
    if ((now_ms - g_last_tick_ms) < ENV_CONTROL_PERIOD_MS) return;
    g_last_tick_ms = now_ms;

    /* MANUAL = algoritmo desligado */
    if (g_mode == ENV_MODE_MANUAL) return;

    /* precisa de tele interna mínima */
    if (is_stale_int(now_ms) || !g_int.has_t || !g_int.has_rh || !g_int.has_lux) {
        set_failsafe();
        bool force = (g_last_force_send_ms == 0) || ((now_ms - g_last_force_send_ms) >= ENV_REFRESH_FORCE_MS);
        maybe_send_outputs(now_ms, force);
        return;
    }

    bool ext_ok = !is_stale_ext(now_ms) && g_ext.has_t && g_ext.has_rh;

    float t_in  = g_int.t;
    float rh_in = g_int.rh;
    int lux_in  = g_int.lux;

    /* ===== LED ===== */
    bool light_on = light_should_be_on();
    g_out.led_pwm = led_control_step(light_on, t_in, lux_in);
    strncpy(g_out.led_rgb, light_on ? "#FFFFFF" : "#000000", sizeof(g_out.led_rgb));

    /* ===== Ventilação =====
       Em caixa fechada, externo é “ajuda”. Se ext inválido, usa só regras internas.
    */
    bool can_cool  = ext_ok ? (g_ext.t  < (t_in - ENV_DT_MIN_COOL)) : (t_in > (ENV_T_SP + 2.0f));
    bool can_dehum = ext_ok ? (g_ext.rh < (rh_in - ENV_DRH_MIN_DEHUM)) : (rh_in > ENV_RH_MAX);

    int pwm_temp = vent_pwm_from_temp(t_in, can_cool);
    int pwm_rh   = vent_pwm_from_rh(rh_in, can_dehum);

    int pwm_fan = max(pwm_temp, pwm_rh);

    /* “LED ON” tende a aquecer → garante ventilação mínima */
    if (light_on && pwm_fan < ENV_FAN_MIN_WHEN_LED_ON) {
        pwm_fan = ENV_FAN_MIN_WHEN_LED_ON;
    }

    /* Umidade muito alta → força mais */
    if (rh_in >= ENV_RH_MAX) {
        pwm_fan = max(pwm_fan, 180);
    }

    g_out.intake_pwm  = clamp_i(pwm_fan, 0, ENV_PWM_MAX);
    g_out.exhaust_pwm = clamp_i(pwm_fan + ENV_EXHAUST_BIAS, 0, ENV_PWM_MAX);

    /* ===== Umidificador ===== */
    g_out.humidifier = humidifier_logic(rh_in, pwm_fan, now_ms);

    /* ===== Irrigação ===== */
    bool soil_valid = g_int.has_soil;
    g_out.irrigation = irrigation_fsm(g_int.soil, soil_valid, now_ms);

    /* ===== Envio ===== */
    bool force = (g_last_force_send_ms == 0) || ((now_ms - g_last_force_send_ms) >= ENV_REFRESH_FORCE_MS);
    maybe_send_outputs(now_ms, force);
}
