#include "environment_controller.h"

#include <Arduino.h>
#include <string.h>
#include <time.h>

#include "logger.h"
#include "mesh_proto.h"

static const char *TAG = "ENV_CTRL";

// nós lógicos
#define NODE_ACT   "act-00"
#define NODE_MSH_GW "msh-gw"

// parâmetros de rede/tempo
static const uint32_t ACT_HB_TIMEOUT_MS   = 15000;  // act-00 considerado offline se >15s sem HB
static const uint32_t CTRL_PERIOD_MS      = 1000;   // calcula a cada 1s (mas só envia se mudou)
static const uint32_t MIN_RESEND_GUARD_MS = 300;    // evita rajada caso múltiplos campos mudem em sequência

// ---------- estado interno ----------
static env_send_json_cb_t g_send_cb = nullptr;

static env_mode_t g_mode = ENV_MODE_AUTO;

static bool     g_act_online = false;
static uint32_t g_last_act_hb_ms = 0;

// telemetria cache
typedef struct {
    bool has_t_in, has_rh_in, has_lux_in, has_soil;
    bool has_t_out, has_rh_out, has_lux_out;

    float t_in, rh_in;
    int   lux_in;
    int   soil;

    float t_out, rh_out;
    int   lux_out;

    uint32_t last_update_ms;
} env_inputs_t;

static env_inputs_t g_in = {0};

// saídas desejadas (calculadas)
typedef struct {
    int  intake_pwm;
    int  exhaust_pwm;
    int  humidifier;
    int  irrigation;
    int  led_pwm;
    char led_rgb[16];
} env_outputs_t;

static env_outputs_t g_out_desired = {0,0,0,0,0,"#000000"};

// último enviado (para diff)
static bool          g_has_last_sent = false;
static env_outputs_t g_out_last_sent = {0,0,0,0,0,"#000000"};

static uint32_t g_last_tick_ms = 0;
static uint32_t g_last_send_ms = 0;

static uint16_t g_msg_counter = 0;

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", (unsigned)g_msg_counter);
}

static bool act_is_online(uint32_t now_ms)
{
    if (!g_act_online) return false;
    return (now_ms - g_last_act_hb_ms) <= ACT_HB_TIMEOUT_MS;
}

static void send_cfg_int_if_changed(uint32_t now_ms, const char *field, int desired, int *last_field)
{
    if (!g_send_cb) return;

    if (!g_has_last_sent || desired != *last_field)
    {
        // evita “rajada” muito apertada (só um pequeno guard)
        if ((now_ms - g_last_send_ms) < MIN_RESEND_GUARD_MS) {
            // deixa passar, mas não é obrigatório; aqui a gente apenas não manda em rajada
            // (na próxima iteração, como ainda estará diferente, vai enviar)
            return;
        }

        char id[8];
        char json[256];
        gen_msg_id(id, sizeof(id));

        if (mesh_proto_build_cfg_int(id,
                                     (uint32_t)millis(),
                                     0,                // QoS0 (não precisa ACK, e reduz overhead)
                                     NODE_MSH_GW,
                                     NODE_ACT,
                                     field,
                                     desired,
                                     json,
                                     sizeof(json)))
        {
            g_send_cb(json);
            LOG("ENV", "AUTO->MESH: %s", json);
            *last_field = desired;
            g_last_send_ms = now_ms;
        }
    }
}

static void send_cfg_str_if_changed(uint32_t now_ms, const char *field, const char *desired, char *last_field, size_t last_len)
{
    if (!g_send_cb) return;

    if (!g_has_last_sent || strncmp(desired, last_field, last_len) != 0)
    {
        if ((now_ms - g_last_send_ms) < MIN_RESEND_GUARD_MS) {
            return;
        }

        char id[8];
        char json[256];
        gen_msg_id(id, sizeof(id));

        if (mesh_proto_build_cfg_str(id,
                                     (uint32_t)millis(),
                                     0,
                                     NODE_MSH_GW,
                                     NODE_ACT,
                                     field,
                                     desired,
                                     json,
                                     sizeof(json)))
        {
            g_send_cb(json);
            LOG("ENV", "AUTO->MESH: %s", json);
            strncpy(last_field, desired, last_len - 1);
            last_field[last_len - 1] = '\0';
            g_last_send_ms = now_ms;
        }
    }
}

static void mark_last_sent_initialized_from_desired()
{
    g_out_last_sent = g_out_desired;
    g_has_last_sent = true;
}

static void compute_outputs(uint32_t now_ms)
{
    // defaults seguros
    int intake  = 30;
    int exhaust = 30;

    // --- controle temperatura (bem simples) ---
    if (g_in.has_t_in)
    {
        if (g_in.t_in >= 30.0f) { intake = 80; exhaust = 80; }
        else if (g_in.t_in <= 22.0f) { intake = 15; exhaust = 15; }
        else { intake = 30; exhaust = 30; }
    }

    // --- controle umidade (histerese simples) ---
    static int humid_state = 0;
    if (g_in.has_rh_in)
    {
        if (g_in.rh_in <= 60.0f) humid_state = 1;
        if (g_in.rh_in >= 70.0f) humid_state = 0;
    }

    // --- irrigação por pulso + cooldown (usa soil se existir) ---
    static int      irr_state = 0;
    static uint32_t irr_on_since = 0;
    static uint32_t irr_last_done = 0;

    const uint32_t IRR_PULSE_MS   = 5000;
    const uint32_t IRR_COOLDOWN_MS= 60UL * 60UL * 1000UL; // 1h

    if (irr_state == 1)
    {
        if ((now_ms - irr_on_since) >= IRR_PULSE_MS)
        {
            irr_state = 0;
            irr_last_done = now_ms;
        }
    }
    else
    {
        if (g_in.has_soil && g_in.soil <= 30)
        {
            if ((now_ms - irr_last_done) >= IRR_COOLDOWN_MS)
            {
                irr_state = 1;
                irr_on_since = now_ms;
            }
        }
    }

    // --- iluminação (RTC): 12h ON (07:00–19:00) ---
    int led_pwm = 0;
    const char *led_rgb = "#000000";

    time_t tnow = time(nullptr);
    struct tm lt;
    localtime_r(&tnow, &lt);
    int hour = lt.tm_hour;

    if (hour >= 7 && hour < 19)
    {
        led_pwm = 180;          // ajuste conforme sua fita/driver (0..255)
        led_rgb = "#FFFFFF";
    }

    // escreve saídas desejadas
    g_out_desired.intake_pwm  = intake;
    g_out_desired.exhaust_pwm = exhaust;
    g_out_desired.humidifier  = humid_state;
    g_out_desired.irrigation  = irr_state;
    g_out_desired.led_pwm     = led_pwm;
    strncpy(g_out_desired.led_rgb, led_rgb, sizeof(g_out_desired.led_rgb) - 1);
    g_out_desired.led_rgb[sizeof(g_out_desired.led_rgb) - 1] = '\0';
}

static void send_only_diffs(uint32_t now_ms)
{
    if (!act_is_online(now_ms)) return;

    if (!g_has_last_sent)
    {
        // primeira sincronização (sem flood): manda tudo uma vez
        g_out_last_sent = g_out_desired;
        g_has_last_sent = true;

        // manda todos como “changed”
        g_out_last_sent.intake_pwm  = g_out_desired.intake_pwm  - 1;
        g_out_last_sent.exhaust_pwm = g_out_desired.exhaust_pwm - 1;
        g_out_last_sent.humidifier  = g_out_desired.humidifier  - 1;
        g_out_last_sent.irrigation  = g_out_desired.irrigation  - 1;
        g_out_last_sent.led_pwm     = g_out_desired.led_pwm     - 1;
        strncpy(g_out_last_sent.led_rgb, "__init__", sizeof(g_out_last_sent.led_rgb) - 1);
        g_out_last_sent.led_rgb[sizeof(g_out_last_sent.led_rgb) - 1] = '\0';
    }

    // manda apenas campos diferentes
    send_cfg_int_if_changed(now_ms, "intake_pwm",  g_out_desired.intake_pwm,  &g_out_last_sent.intake_pwm);
    send_cfg_int_if_changed(now_ms, "exhaust_pwm", g_out_desired.exhaust_pwm, &g_out_last_sent.exhaust_pwm);
    send_cfg_int_if_changed(now_ms, "humidifier",  g_out_desired.humidifier,  &g_out_last_sent.humidifier);
    send_cfg_int_if_changed(now_ms, "irrigation",  g_out_desired.irrigation,  &g_out_last_sent.irrigation);
    send_cfg_int_if_changed(now_ms, "led_pwm",     g_out_desired.led_pwm,     &g_out_last_sent.led_pwm);
    send_cfg_str_if_changed(now_ms, "led_rgb",     g_out_desired.led_rgb,     g_out_last_sent.led_rgb, sizeof(g_out_last_sent.led_rgb));
}

// ---------- API pública ----------
void env_ctrl_init(env_send_json_cb_t send_cb)
{
    g_send_cb = send_cb;
    g_mode = ENV_MODE_AUTO;

    g_act_online = false;
    g_last_act_hb_ms = 0;

    memset(&g_in, 0, sizeof(g_in));

    memset(&g_out_desired, 0, sizeof(g_out_desired));
    strncpy(g_out_desired.led_rgb, "#000000", sizeof(g_out_desired.led_rgb) - 1);
    g_out_desired.led_rgb[sizeof(g_out_desired.led_rgb) - 1] = '\0';

    memset(&g_out_last_sent, 0, sizeof(g_out_last_sent));
    strncpy(g_out_last_sent.led_rgb, "#000000", sizeof(g_out_last_sent.led_rgb) - 1);
    g_out_last_sent.led_rgb[sizeof(g_out_last_sent.led_rgb) - 1] = '\0';

    g_has_last_sent = false;

    g_last_tick_ms = 0;
    g_last_send_ms = 0;
    g_msg_counter = 0;

    LOG(TAG, "init OK (mode=AUTO)");
}

void env_ctrl_on_mesh_msg(const mesh_msg_t *msg, uint32_t now_ms)
{
    if (!msg) return;

    // detecta act-00 online via HB
    if (msg->type == MESH_MSG_HB && strcmp(msg->src, NODE_ACT) == 0)
    {
        g_last_act_hb_ms = now_ms;

        if (!g_act_online)
        {
            g_act_online = true;
            // força uma sync completa quando ele “aparece”
            g_has_last_sent = false;
            LOG(TAG, "act-00 ONLINE (HB recebido) -> sync on-change habilitado");
        }
        return;
    }

    // telemetria
    if (msg->type == MESH_MSG_TELE)
    {
        // atualiza internos/externos independente do src, usando flags
        if (msg->tele.has_t_in)  { g_in.t_in = msg->tele.t_in;  g_in.has_t_in = true; }
        if (msg->tele.has_rh_in) { g_in.rh_in= msg->tele.rh_in; g_in.has_rh_in= true; }
        if (msg->tele.has_lux_in){ g_in.lux_in=msg->tele.lux_in;g_in.has_lux_in=true; }
        if (msg->tele.has_soil_moist){ g_in.soil=msg->tele.soil_moist; g_in.has_soil=true; }

        if (msg->tele.has_t_out)  { g_in.t_out = msg->tele.t_out;  g_in.has_t_out = true; }
        if (msg->tele.has_rh_out) { g_in.rh_out= msg->tele.rh_out; g_in.has_rh_out= true; }
        if (msg->tele.has_lux_out){ g_in.lux_out=msg->tele.lux_out;g_in.has_lux_out=true; }

        g_in.last_update_ms = now_ms;
    }
}

void env_ctrl_on_uart_msg(const mesh_msg_t *msg, uint32_t now_ms)
{
    if (!msg) return;

    // Aqui é a correção do “manual”: quando o Blynk manda cfg.mode para act-00,
    // o gateway também precisa refletir isso internamente, senão o AUTO continua rodando.
    if (msg->type == MESH_MSG_CFG && msg->cfg.has_mode)
    {
        env_mode_t new_mode = (msg->cfg.mode == 1) ? ENV_MODE_MANUAL : ENV_MODE_AUTO;

        if (new_mode != g_mode)
        {
            g_mode = new_mode;

            if (g_mode == ENV_MODE_MANUAL)
            {
                LOG(TAG, "modo -> MANUAL (AUTO desligado)");
            }
            else
            {
                LOG(TAG, "modo -> AUTO (AUTO ligado) -> sync on-change");
                // ao voltar para AUTO, sincroniza outputs (sem flood: só diffs)
                g_has_last_sent = false;
            }
        }
    }

    (void)now_ms;
}

void env_ctrl_tick(uint32_t now_ms)
{
    // se manual, não atua (não manda nada)
    if (g_mode == ENV_MODE_MANUAL)
        return;

    // só calcula em período fixo
    if ((now_ms - g_last_tick_ms) < CTRL_PERIOD_MS)
        return;

    g_last_tick_ms = now_ms;

    // se act offline, nem tenta enviar (mas pode calcular)
    compute_outputs(now_ms);

    // envia apenas diffs (e apenas se act online)
    send_only_diffs(now_ms);
}
