/**
 * @file main.cpp
 * @brief Blynk Gateway Module
 */

#include "credentials.h"
#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

#include "pins.h"
#include "ipc_uart.h"
#include "mesh_proto.h"

#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"
#define NODE_EXT_SEN    "ext-sen-00"
#define NODE_INT_SEN    "int-sen-00"
#define NODE_ACT        "act-00"

static BlynkTimer g_timer;
static uint16_t   g_msg_counter = 0;
static int        g_mode        = 0;

/* ---------- helpers ---------- */

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

/* ---------- envio CFG usando QoS da lib ---------- */

static void send_cfg_int_field(const char *field, int value)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_cfg_int(id,
                                  (uint32_t)millis(),
                                  1,               // QoS=1
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
                                  1,               // QoS=1
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

/* ---------- Blynk callbacks ---------- */

BLYNK_CONNECTED()
{
    Serial.println("[BLYNK] Connected, syncing V13..V19");
    Blynk.syncVirtual(V13, V14, V15, V16, V17, V18, V19);

    // opcional: HELLO para mesh
    char id[8], json[256];
    gen_msg_id(id, sizeof(id));
    if (mesh_proto_build_hello(id,
                               (uint32_t)millis(),
                               0,
                               NODE_BLYNK_GW,
                               NODE_MSH_GW,
                               NODE_BLYNK_GW,
                               "1.0.0",
                               "blynk-gw boot",
                               json,
                               sizeof(json))) {
        ipc_uart_send_json(json);
        Serial.print("[TX HELLO] ");
        Serial.println(json);
    }
}

BLYNK_WRITE(V13)
{
    g_mode = param.asInt() ? 1 : 0;
    Serial.printf("[BLYNK] mode=%d\n", g_mode);
    send_cfg_int_field("mode", g_mode);
}

/* Igual ao mock Python:
 * V14 = intake_pwm
 * V15 = exhaust_pwm
 * V16 = humidifier
 * V17 = led_pwm (led_brig)
 * V18 = led_rgb
 * V19 = irrigation
 */

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

/* ---------- handlers mesh_proto ---------- */

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
    Serial.print("[HELLO] node_id=");
    if (msg.hello.has_node_id) Serial.print(msg.hello.node_id);
    Serial.print(" fw=");
    if (msg.hello.has_fw_ver) Serial.print(msg.hello.fw_ver);
    Serial.print(" extra=");
    if (msg.hello.has_extra) Serial.print(msg.hello.extra);
    Serial.println();
}

static void handle_time(const mesh_msg_t &msg)
{
    Serial.print("[TIME] epoch=");
    if (msg.time_sync.has_epoch) Serial.print(msg.time_sync.epoch);
    Serial.print(" tz_offset_min=");
    if (msg.time_sync.has_tz_offset_min) Serial.print(msg.time_sync.tz_offset_min);
    Serial.println();
}

static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[MESH] JSON parse fail: ");
        Serial.println(json);
        return;
    }

    if (strcmp(msg.dst, NODE_BLYNK_GW) != 0 &&
        strcmp(msg.dst, "*") != 0) {
        return;
    }

    switch (msg.type) {
    case MESH_MSG_TELE:   handle_tele(msg);          break;
    case MESH_MSG_STATE:  handle_state(msg);         break;
    case MESH_MSG_HB:     handle_hb(msg);            break;
    case MESH_MSG_EVT:    handle_evt(msg);           break;
    case MESH_MSG_HELLO:  handle_hello(msg);         break;
    case MESH_MSG_ACK:    mesh_proto_qos_on_ack(&msg); break;
    case MESH_MSG_TIME:   handle_time(msg);          break;
    default:
        Serial.print("[MESH] tipo não tratado: ");
        Serial.println(msg.type);
        break;
    }
}

/* ---------- setup / loop ---------- */

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("[blynk_gateway_module] boot");

    // IPC UART
    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);

    // QoS da lib usa o mesmo transporte
    mesh_proto_qos_init(ipc_uart_send_json);

    // WiFi + Blynk
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Conectando");
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print('.');
    }
    Serial.println("\n[WiFi] Conectado");

    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();

    g_timer.setInterval(10000L, []() {
        long up = millis() / 1000;
        Blynk.virtualWrite(20, up);
    });
}

void loop()
{
    Blynk.run();
    g_timer.run();

    char json[256];
    if (ipc_uart_read_json(json, sizeof(json))) {
        Serial.print("[RX JSON] ");
        Serial.println(json);
        handle_mesh_json(json);
    }

    // gerenciador de QoS da lib
    mesh_proto_qos_poll();
}

