/**
 * Mantido:
 *  - HELLO QoS1 uma vez ao conectar na mesh
 *  - STATE inicial
 *  - QoS poll no loop
 *  - Comandos CFG aplicam apenas em modo MANUAL (mode=1)
 *
 * ATUALIZADO:
 *  - Em AUTO (mode=0) aplica comandos CFG vindos do msh-gw (environment_controller);
 *  - Em MANUAL (mode=1) aplica fans/LED apenas do blynk-gw;
 *  - humidifier/irrigation sempre vêm do msh-gw (duty-cycle / segurança no gateway).
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <FastLED.h>
#include "pins.h"
#include "credentials.h"
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_ACT        "act-00"
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

// Neste nó, tudo vai para o blynk-gw
#define NODE_SINK       NODE_BLYNK_GW

// -----------------------------------------------------------------------------
// Períodos
// -----------------------------------------------------------------------------
#define HB_PERIOD_MS    (60000UL)   // 1 minuto

// -----------------------------------------------------------------------------
// PWM / LEDs
// -----------------------------------------------------------------------------
#define PWM_FREQ_HZ       25000
#define PWM_RES_BITS      8
#define FAN_IN_CHANNEL    0
#define FAN_EX_CHANNEL    1
#define NUM_LEDS          300        // ajuste conforme sua fita

// Observação: no seu controle “estufa_esp32_platformio” havia BRG.
// Se sua cor ficar trocada, troque BRG por GRB.
#ifndef LED_COLOR_ORDER
#define LED_COLOR_ORDER   BRG
#endif

static CRGB g_leds[NUM_LEDS];

// -----------------------------------------------------------------------------
// Objetos da mesh
// -----------------------------------------------------------------------------
Scheduler userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Estado dos atuadores (act-00)
// -----------------------------------------------------------------------------
static int  g_mode        = 0;        // 0=AUTO, 1=MANUAL
static int  g_intake_pwm  = 0;        // 0..100 (percentual)
static int  g_exhaust_pwm = 0;        // 0..100 (percentual)
static int  g_humidifier  = 0;        // 0/1
static int  g_irrigation  = 0;        // 0/1
static int  g_led_pwm     = 0;        // brilho em % (0..100)
static char g_led_rgb[16] = "000000"; // "RRGGBB" normalizado

static bool g_act_dirty   = true;     // aplica no boot

// -----------------------------------------------------------------------------
// Controle de envio / contadores
// -----------------------------------------------------------------------------
static uint16_t      g_msg_counter      = 0;
static unsigned long g_last_hb          = 0;
static bool          g_hello_sent       = false;
static bool          g_hb_sent_on_mesh  = false; // <- NOVO: garante HB apenas na 1ª conexão

// -----------------------------------------------------------------------------
// Helpers (C-like)
// -----------------------------------------------------------------------------
static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

// Aceita "RRGGBB", "#RRGGBB", "0xRRGGBB" e normaliza para "RRGGBB"
static bool normalize_hex6(const char *in, char *out_hex6, size_t out_len)
{
    if (!in || out_len < 7) return false;

    while (*in == ' ' || *in == '\t' || *in == '\r' || *in == '\n') in++;
    if (in[0] == '#') in++;
    if (in[0] == '0' && (in[1] == 'x' || in[1] == 'X')) in += 2;

    for (int i = 0; i < 6; i++)
    {
        if (hex_nibble(in[i]) < 0) return false;
    }

    for (int i = 0; i < 6; i++)
    {
        char c = in[i];
        if (c >= 'a' && c <= 'f') c = (char)('A' + (c - 'a'));
        out_hex6[i] = c;
    }
    out_hex6[6] = '\0';
    return true;
}

static bool parse_rgb_hex6(const char *hex6, CRGB *out)
{
    if (!hex6 || !out) return false;
    int r1 = hex_nibble(hex6[0]), r2 = hex_nibble(hex6[1]);
    int g1 = hex_nibble(hex6[2]), g2 = hex_nibble(hex6[3]);
    int b1 = hex_nibble(hex6[4]), b2 = hex_nibble(hex6[5]);
    if (r1 < 0 || r2 < 0 || g1 < 0 || g2 < 0 || b1 < 0 || b2 < 0) return false;

    uint8_t r = (uint8_t)((r1 << 4) | r2);
    uint8_t g = (uint8_t)((g1 << 4) | g2);
    uint8_t b = (uint8_t)((b1 << 4) | b2);

    *out = CRGB(r, g, b);
    return true;
}

static uint8_t percent_to_255(int pct)
{
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (uint8_t)((pct * 255 + 50) / 100);
}

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// -----------------------------------------------------------------------------
// Atuadores reais (inicialização + apply)
// -----------------------------------------------------------------------------
static void actuators_init(void)
{
    // Fans (LEDC PWM)
    ledcSetup(FAN_IN_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(FAN_INTERNO_PIN, FAN_IN_CHANNEL);
    ledcWrite(FAN_IN_CHANNEL, 0);

    ledcSetup(FAN_EX_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(FAN_EXTERNO_PIN, FAN_EX_CHANNEL);
    ledcWrite(FAN_EX_CHANNEL, 0);

    // Switches
    pinMode(NEBULIZADOR_PIN, OUTPUT);
    digitalWrite(NEBULIZADOR_PIN, LOW);

    pinMode(BOMBA_PIN, OUTPUT);
    digitalWrite(BOMBA_PIN, LOW);

    // LEDs
    FastLED.addLeds<WS2811, LED_DATA_PIN, LED_COLOR_ORDER>(g_leds, NUM_LEDS);
    FastLED.setBrightness(0);
    for (int i = 0; i < NUM_LEDS; i++) g_leds[i] = CRGB::Black;
    FastLED.show();
}

// Aplica o estado atual nos atuadores reais
// (em AUTO recebe do msh-gw; em MANUAL recebe do blynk-gw; relés sempre do msh-gw)
static void actuators_apply_if_dirty(void)
{
    if (!g_act_dirty) return;

    // Fans: protocolo usa % (0..100) -> duty 0..255
    int inPct = clamp_int(g_intake_pwm, 0, 100);
    int exPct = clamp_int(g_exhaust_pwm, 0, 100);
    int inDuty = (int)percent_to_255(inPct);
    int exDuty = (int)percent_to_255(exPct);

    ledcWrite(FAN_IN_CHANNEL, (uint32_t)inDuty);
    ledcWrite(FAN_EX_CHANNEL, (uint32_t)exDuty);

    // Switches (activeHigh = true)
    digitalWrite(NEBULIZADOR_PIN, (g_humidifier != 0) ? HIGH : LOW);
    digitalWrite(BOMBA_PIN,       (g_irrigation != 0) ? HIGH : LOW);

    // LEDs
    uint8_t br = percent_to_255(g_led_pwm);

    CRGB c = CRGB::Black;
    (void)parse_rgb_hex6(g_led_rgb, &c);

    FastLED.setBrightness(br);
    for (int i = 0; i < NUM_LEDS; i++) g_leds[i] = c;
    FastLED.show();

    g_act_dirty = false;

    Serial.printf("[ACT APPLY] mode=%d inPct=%d(inDuty=%d) exPct=%d(exDuty=%d) hum=%d irr=%d led_pct=%d rgb=%s\n",
                  g_mode, inPct, inDuty, exPct, exDuty, g_humidifier, g_irrigation, g_led_pwm, g_led_rgb);
}

// -----------------------------------------------------------------------------
// Callback usado pela lib mesh_proto para envio via mesh
// -----------------------------------------------------------------------------
static void mesh_send_json_cb(const char *json)
{
    mesh.sendBroadcast(json);
    Serial.print("[MESH TX] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// Builders: STATE / HB / HELLO
// -----------------------------------------------------------------------------
static void send_state_now(void)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_state_act(id,
                                   (uint32_t)millis(),
                                   0,              // QoS0
                                   NODE_ACT,
                                   NODE_SINK,
                                   g_intake_pwm,   // % (0..100)
                                   g_exhaust_pwm,  // % (0..100)
                                   g_humidifier,
                                   g_led_pwm,       // led_brig (0..100)
                                   g_led_rgb,       // "RRGGBB"
                                   g_irrigation,
                                   json,
                                   sizeof(json)))
    {
        return;
    }

    mesh_send_json_cb(json);
}

static void send_hb(void)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    // Mantive como estava: rssi “fake” para dar variação (se quiser, troco pra WiFi.RSSI())
    int rssi_fake = -50 - (int)(millis() % 20);

    if (!mesh_proto_build_hb(id,
                            (uint32_t)millis(),
                            NODE_ACT,
                            NODE_SINK,
                            (int)(millis() / 1000),
                            rssi_fake,
                            json,
                            sizeof(json)))
    {
        return;
    }

    mesh_send_json_cb(json);
}

static void send_hello(void)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hello(id,
                               (uint32_t)millis(),
                               1,              // QoS1
                               NODE_ACT,       // src lógico
                               NODE_MSH_GW,    // dst = msh-gw
                               NODE_ACT,
                               "1.0.0-act-real",
                               "actuators",
                               json,
                               sizeof(json)))
    {
        return;
    }

    // Registra no QoS e envia (ou fallback)
    if (!mesh_proto_qos_register_and_send(id, json))
    {
        mesh_send_json_cb(json);
    }
}

// -----------------------------------------------------------------------------
// Aplicar CFG recebido (AUTO/MANUAL)
// -----------------------------------------------------------------------------
static void apply_cfg(const mesh_msg_t *msg)
{
    if (!msg) return;

    bool changed = false;

    const bool from_mesh_gw = (strcmp(msg->src, NODE_MSH_GW) == 0);
    const bool from_blynk   = (strcmp(msg->src, NODE_BLYNK_GW) == 0);

    // Atualiza modo se veio (sempre aceita)
    if (msg->cfg.has_mode)
    {
        int new_mode = msg->cfg.mode ? 1 : 0;
        if (new_mode != g_mode)
        {
            g_mode = new_mode;
            changed = true;
        }
    }

    // AUTO: aceita apenas comandos vindos do msh-gw (environment_controller)
    const bool allow_auto = (g_mode == 0) && from_mesh_gw;

    // MANUAL: aceita fans/leds apenas do blynk-gw
    const bool allow_manual_ui = (g_mode == 1) && from_blynk;

    // Bombas/relés: SEMPRE aceitar apenas do msh-gw (duty-cycle / segurança no gateway)
    const bool allow_relays = from_mesh_gw;

    // Fans (0..100 em %)
    if (msg->cfg.has_intake_pwm && (allow_auto || allow_manual_ui))
    {
        int v = clamp_int(msg->cfg.intake_pwm, 0, 100);
        if (v != g_intake_pwm)
        {
            g_intake_pwm = v;
            changed = true;
        }
    }

    if (msg->cfg.has_exhaust_pwm && (allow_auto || allow_manual_ui))
    {
        int v = clamp_int(msg->cfg.exhaust_pwm, 0, 100);
        if (v != g_exhaust_pwm)
        {
            g_exhaust_pwm = v;
            changed = true;
        }
    }

    // Relés (0/1) — só do msh-gw
    if (msg->cfg.has_humidifier && allow_relays)
    {
        int v = msg->cfg.humidifier ? 1 : 0;
        if (v != g_humidifier)
        {
            g_humidifier = v;
            changed = true;
        }
    }

    if (msg->cfg.has_irrigation && allow_relays)
    {
        int v = msg->cfg.irrigation ? 1 : 0;
        if (v != g_irrigation)
        {
            g_irrigation = v;
            changed = true;
        }
    }

    // LEDs (0..100 + "RRGGBB")
    if (msg->cfg.has_led_pwm && (allow_auto || allow_manual_ui))
    {
        int v = clamp_int(msg->cfg.led_pwm, 0, 100);
        if (v != g_led_pwm)
        {
            g_led_pwm = v;
            changed = true;
        }
    }

    if (msg->cfg.has_led_rgb && (allow_auto || allow_manual_ui))
    {
        char norm[7];
        if (normalize_hex6(msg->cfg.led_rgb, norm, sizeof(norm)))
        {
            if (strncmp(g_led_rgb, norm, 6) != 0)
            {
                strncpy(g_led_rgb, norm, sizeof(g_led_rgb) - 1);
                g_led_rgb[sizeof(g_led_rgb) - 1] = '\0';
                changed = true;
            }
        }
        else
        {
            Serial.printf("[CFG] led_rgb invalido: '%s'\n", msg->cfg.led_rgb);
        }
    }

    Serial.printf("[CFG] src=%s mode=%d allow_auto=%d allow_ui=%d allow_relays=%d -> in=%d ex=%d hum=%d irr=%d led_pct=%d led_rgb=%s changed=%d\n",
                  msg->src, g_mode,
                  allow_auto ? 1 : 0, allow_manual_ui ? 1 : 0, allow_relays ? 1 : 0,
                  g_intake_pwm, g_exhaust_pwm, g_humidifier, g_irrigation, g_led_pwm, g_led_rgb,
                  changed ? 1 : 0);

    // Aplica imediatamente se houve mudança e publica STATE (para refletir no Blynk)
    if (changed)
    {
        g_act_dirty = true;
        actuators_apply_if_dirty();
        send_state_now();
    }

    // Se CFG era QoS1, ACK ok
    mesh_proto_qos_send_ack_ok(msg);
}

// -----------------------------------------------------------------------------
// Handlers de mensagens da mesh
// -----------------------------------------------------------------------------
static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg))
    {
        Serial.print("[RX] parse fail: ");
        Serial.println(json ? json : "(null)");
        return;
    }

    // ACK: sempre notifica QoS
    if (msg.type == MESH_MSG_ACK)
    {
        mesh_proto_qos_on_ack(&msg);
        Serial.printf("[RX ACK] ref=%s from=%s dst=%s\n",
                      msg.ack.ref, msg.src, msg.dst);
        return;
    }

    // Filtro de destino lógico
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, NODE_ACT)    != 0 &&
        strcmp(msg.dst, "*")         != 0)
    {
        return;
    }

    switch (msg.type)
    {
        case MESH_MSG_CFG:
            apply_cfg(&msg);
            break;

        case MESH_MSG_TIME:
            Serial.println("[TIME] recebido (não aplicado)");
            break;

        default:
            break;
    }
}

// -----------------------------------------------------------------------------
// Callbacks painlessMesh
// -----------------------------------------------------------------------------
void mesh_receive_cb(uint32_t from, String &msg)
{
    Serial.printf("[MESH RX] from %u: %s\n", from, msg.c_str());
    handle_mesh_json(msg.c_str());
}

void mesh_new_connection_cb(uint32_t nodeId)
{
    Serial.printf("[MESH] New connection, nodeId=%u\n", nodeId);

    // HB imediato APENAS na 1ª conexão (pedido)
    if (!g_hb_sent_on_mesh)
    {
        g_hb_sent_on_mesh = true;
        Serial.println("[MESH] Primeira conexao: enviando HB imediato.");
        send_hb();

        // atualiza o timer pra não disparar de novo no loop logo em seguida
        g_last_hb = millis();
    }

    // HELLO uma vez
    if (!g_hello_sent)
    {
        g_hello_sent = true;
        Serial.println("[MESH] Primeira conexao: enviando HELLO QoS1 do act-00...");
        send_hello();

        // STATE inicial
        send_state_now();
    }
}

void mesh_changed_connections_cb()
{
    Serial.println("[MESH] Changed connections");
}

void mesh_time_adjusted_cb(int32_t offset)
{
    Serial.printf("[MESH] Time adjusted offset=%ld\n", (long)offset);
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("[act-00] boot (REAL actuators)");

    randomSeed(esp_random());

    // HW init (estado seguro)
    actuators_init();

    // inicia em AUTO
    g_mode = 0;

    // QoS init: usa callback de envio por mesh
    mesh_proto_qos_init(mesh_send_json_cb);

    // Mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    g_last_hb = millis();
}

void loop()
{
    mesh.update();

    // QoS poll
    mesh_proto_qos_poll();

    // HB periódico
    if ((millis() - g_last_hb) >= HB_PERIOD_MS)
    {
        g_last_hb = millis();
        send_hb();
    }

    // aplica pendências (caso tenha vindo CFG e marcado dirty)
    actuators_apply_if_dirty();
}
