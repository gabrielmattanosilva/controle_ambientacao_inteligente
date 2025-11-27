/**
 * @file blynk_client.cpp
 * @brief Camada de integração com Blynk (WiFi, callbacks, Vpins, etc.).
 */

#include "blynk_client.h"
#include "credentials.h"      // BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <time.h>
#include <sys/time.h>
#include "mesh_proto.h"

// IDs dos nós (apenas strings; devem bater com o resto da rede)
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"
#define NODE_EXT_SEN    "ext-sen-00"
#define NODE_INT_SEN    "int-sen-00"
#define NODE_ACT        "act-00"

static BlynkTimer g_timer;
static uint16_t   g_msg_counter   = 0;
static int        g_mode          = 0;
static bool       g_time_sync_ok  = false;  // só para log/debug
static bool       g_mesh_hello_ok = false;  // TRUE depois do HELLO do msh-gw

/* ============================================================
 * Helpers internos
 * ============================================================ */

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

/**
 * Aplica o TIME recebido do mesh_gateway_module ao RTC interno deste ESP32
 * e, como efeito colateral, força o painel para modo AUTOMÁTICO (V13 = 1).
 */
static void apply_time_from_mesh(uint32_t epoch, int tz_offset_min)
{
    struct timeval tv;
    tv.tv_sec  = (time_t)epoch;
    tv.tv_usec = 0;

    if (settimeofday(&tv, nullptr) != 0) {
        Serial.println("[TIME] settimeofday() falhou");
        return;
    }

    time_t t = (time_t)epoch;
    struct tm tm_local;
    localtime_r(&t, &tm_local);

    Serial.printf("[TIME] RTC sincronizado: %04d-%02d-%02d %02d:%02d:%02d (tz_offset_min=%d)\n",
                  tm_local.tm_year + 1900,
                  tm_local.tm_mon + 1,
                  tm_local.tm_mday,
                  tm_local.tm_hour,
                  tm_local.tm_min,
                  tm_local.tm_sec,
                  tz_offset_min);

    g_time_sync_ok = true;

    // Garante que o painel Blynk comece em modo AUTOMATICO (V13 = 1)
    g_mode = 1;
    Blynk.virtualWrite(V13, 1);
}

/* ---------- envio CFG usando QoS da lib mesh_proto ---------- */

static void send_cfg_int_field(const char *field, int value)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_cfg_int(id,
                                  (uint32_t)millis(),
                                  1, // QoS1
                                  NODE_BLYNK_GW,
                                  NODE_MSH_GW,
                                  field,
                                  value,
                                  json,
                                  sizeof(json))) {
        return;
    }

    mesh_proto_qos_register_and_send(id, json);

    Serial.print("[TX CFG INT] ");
    Serial.println(json);
}

static void send_cfg_str_field(const char *field, const char *value)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_cfg_str(id,
                                  (uint32_t)millis(),
                                  1, // QoS1
                                  NODE_BLYNK_GW,
                                  NODE_MSH_GW,
                                  field,
                                  value,
                                  json,
                                  sizeof(json))) {
        return;
    }

    mesh_proto_qos_register_and_send(id, json);

    Serial.print("[TX CFG STR] ");
    Serial.println(json);
}

/* ============================================================
 * Blynk callbacks
 * ============================================================ */

BLYNK_CONNECTED()
{
    Serial.println("[BLYNK] Connected, syncing V13..V19");
    Blynk.syncVirtual(V13, V14, V15, V16, V17, V18, V19);
}

BLYNK_WRITE(V13)
{
    g_mode = param.asInt() ? 1 : 0;
    Serial.printf("[BLYNK] V13 (modo) = %d\n", g_mode);
    send_cfg_int_field("mode", g_mode);
}

BLYNK_WRITE(V14)
{
    if (!g_mode) {
        Serial.println("[BLYNK] V14 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("intake_pwm", param.asInt());
}

BLYNK_WRITE(V15)
{
    if (!g_mode) {
        Serial.println("[BLYNK] V15 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("exhaust_pwm", param.asInt());
}

BLYNK_WRITE(V16)
{
    if (!g_mode) {
        Serial.println("[BLYNK] V16 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("humidifier", param.asInt());
}

BLYNK_WRITE(V17)
{
    if (!g_mode) {
        Serial.println("[BLYNK] V17 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("led_pwm", param.asInt());
}

BLYNK_WRITE(V18)
{
    if (!g_mode) {
        Serial.println("[BLYNK] V18 ignorado (modo AUTO)");
        return;
    }
    send_cfg_str_field("led_rgb", param.asStr());
}

BLYNK_WRITE(V19)
{
    if (!g_mode) {
        Serial.println("[BLYNK] V19 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("irrigation", param.asInt());
}

/* ============================================================
 * Handlers de mensagens da malha -> Blynk
 * ============================================================ */

static void handle_tele(const mesh_msg_t &msg)
{
    if (msg.tele.has_t_out)   Blynk.virtualWrite(0, msg.tele.t_out);
    if (msg.tele.has_rh_out)  Blynk.virtualWrite(1, msg.tele.rh_out);
    if (msg.tele.has_lux_out) Blynk.virtualWrite(2, msg.tele.lux_out);

    if (msg.tele.has_t_in)       Blynk.virtualWrite(3, msg.tele.t_in);
    if (msg.tele.has_rh_in)      Blynk.virtualWrite(4, msg.tele.rh_in);
    if (msg.tele.has_soil_moist) Blynk.virtualWrite(5, msg.tele.soil_moist);
    if (msg.tele.has_lux_in)     Blynk.virtualWrite(6, msg.tele.lux_in);
}

static void handle_state(const mesh_msg_t &msg)
{
    if (msg.state.has_intake_pwm)
        Blynk.virtualWrite(7, msg.state.intake_pwm);
    if (msg.state.has_exhaust_pwm)
        Blynk.virtualWrite(8, msg.state.exhaust_pwm);
    if (msg.state.has_humidifier)
        Blynk.virtualWrite(9, msg.state.humidifier);
    if (msg.state.has_led_brig)
        Blynk.virtualWrite(10, msg.state.led_brig);
    if (msg.state.has_led_rgb)
        Blynk.virtualWrite(11, msg.state.led_rgb);
    if (msg.state.has_irrigation)
        Blynk.virtualWrite(12, msg.state.irrigation);
}

static void handle_hb(const mesh_msg_t &msg)
{
    if (!msg.hb.has_uptime_s) return;
    int up = msg.hb.uptime_s;
    int vpin = -1;

    if (!strcmp(msg.src, NODE_MSH_GW))       vpin = 20;
    else if (!strcmp(msg.src, NODE_EXT_SEN)) vpin = 21;
    else if (!strcmp(msg.src, NODE_INT_SEN)) vpin = 22;
    else if (!strcmp(msg.src, NODE_ACT))     vpin = 23;

    if (vpin >= 0) {
        Blynk.virtualWrite(vpin, up);
    }
}

static void handle_evt(const mesh_msg_t &msg)
{
    Serial.print("[EVT] from=");
    Serial.print(msg.src);
    Serial.print(" event=");
    if (msg.evt.has_event) Serial.print(msg.evt.event);
    Serial.print(" code=");
    if (msg.evt.has_code) Serial.print(msg.evt.code);
    Serial.print(" level=");
    if (msg.evt.has_level) Serial.print(msg.evt.level);
    Serial.println();
}

static void handle_hello(const mesh_msg_t &msg)
{
    Serial.print("[HELLO] from=");
    if (msg.hello.has_node_id) Serial.print(msg.hello.node_id);
    Serial.print(" fw=");
    if (msg.hello.has_fw_ver) Serial.print(msg.hello.fw_ver);
    Serial.println();

    // Só libera depois do HELLO vindo do mesh_gateway (msh-gw)
    if (msg.hello.has_node_id && strcmp(msg.hello.node_id, NODE_MSH_GW) == 0) {
        g_mesh_hello_ok = true;
        Serial.println("[HELLO] Mesh gateway registrado, liberando TELE/STATE/HB/EVT");
    }
}

static void handle_time(const mesh_msg_t &msg)
{
    if (!msg.time_sync.has_epoch) {
        Serial.println("[TIME] recebido sem epoch valido");
        return;
    }

    int tz = msg.time_sync.has_tz_offset_min ? msg.time_sync.tz_offset_min : 0;
    apply_time_from_mesh(msg.time_sync.epoch, tz);
}

/* ============================================================
 * API exposta no header
 * ============================================================ */

void blynk_client_init()
{
    g_mode          = 0;
    g_msg_counter   = 0;
    g_time_sync_ok  = false;
    g_mesh_hello_ok = false;

    // WiFi + Blynk
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Conectando");
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("[WiFi] Conectado, IP=");
    Serial.println(WiFi.localIP());

    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();

    // Heartbeat simples do próprio blynk-gw em V20
    g_timer.setInterval(10000L, []() {
        long up = millis() / 1000;
        Blynk.virtualWrite(20, up);
    });
}

void blynk_client_loop()
{
    Blynk.run();
    g_timer.run();
}

void blynk_client_handle_mesh_msg(const mesh_msg_t &msg)
{
    // HELLO e TIME sempre passam (mesmo antes do HELLO do mesh_gateway)
    if (msg.type == MESH_MSG_HELLO) {
        handle_hello(msg);
        return;
    }

    if (msg.type == MESH_MSG_TIME) {
        handle_time(msg);
        return;
    }

    // TELE/STATE/HB/EVT só depois de receber HELLO do mesh_gateway
    if (!g_mesh_hello_ok &&
        (msg.type == MESH_MSG_TELE ||
         msg.type == MESH_MSG_STATE ||
         msg.type == MESH_MSG_HB   ||
         msg.type == MESH_MSG_EVT)) {
        Serial.println("[MESH] Ignorando TELE/STATE/HB/EVT antes de HELLO do mesh_gateway");
        return;
    }

    switch (msg.type) {
    case MESH_MSG_TELE:   handle_tele(msg);   break;
    case MESH_MSG_STATE:  handle_state(msg);  break;
    case MESH_MSG_HB:     handle_hb(msg);     break;
    case MESH_MSG_EVT:    handle_evt(msg);    break;
    default:
        Serial.print("[MESH] tipo não tratado em blynk_client: ");
        Serial.println(msg.type);
        break;
    }
}
