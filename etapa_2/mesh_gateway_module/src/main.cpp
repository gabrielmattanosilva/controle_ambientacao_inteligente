/**
 * @file main.cpp
 * @brief Mesh Gateway Module
 */

#include <Arduino.h>
#include <ArduinoJson.h>

#include "pins.h"
#include "ipc_uart.h"
#include "mesh_proto.h"

#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"
#define NODE_EXT_SEN    "ext-sen-00"
#define NODE_INT_SEN    "int-sen-00"
#define NODE_ACT        "act-00"

/* --------- estado interno / timers --------- */

static uint16_t      g_msg_counter = 0;
static unsigned long g_last_tele   = 0;
static unsigned long g_last_hb     = 0;

static int  g_mode        = 0;
static int  g_intake_pwm  = 0;
static int  g_exhaust_pwm = 0;
static int  g_humidifier  = 0;
static int  g_irrigation  = 0;
static int  g_led_pwm     = 0;
static char g_led_rgb[16] = "FFD480";

/* --------- helpers --------- */

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

static void send_state_now()
{
    char id[8];
    char json[256];
    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_state_act(id,
                                    (uint32_t)millis(),
                                    0,  // QoS0 para state
                                    NODE_ACT,
                                    NODE_BLYNK_GW,
                                    g_intake_pwm,
                                    g_exhaust_pwm,
                                    g_humidifier,
                                    g_led_pwm,
                                    g_led_rgb,
                                    g_irrigation,
                                    json,
                                    sizeof(json))) {
        return;
    }

    ipc_uart_send_json(json);
    Serial.print("[TX STATE] ");
    Serial.println(json);
}

/* --------- telemetria mock --------- */

static void send_tele_dump()
{
    JsonDocument doc;
    JsonObject   data;
    char id[8];
    char json[256];

    /* tele externa (ext-sen-00) */
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_EXT_SEN;
    doc["dst"]  = NODE_BLYNK_GW;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_out"]   = (float)(random(180, 350)) / 10.0f;
    data["rh_out"]  = (float)(random(300, 900)) / 10.0f;
    data["lux_out"] = random(0, 50000);

    serializeJson(doc, json, sizeof(json));
    ipc_uart_send_json(json);
    Serial.print("[TX TELE EXT] ");
    Serial.println(json);

    /* tele interna (int-sen-00) */
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_INT_SEN;
    doc["dst"]  = NODE_BLYNK_GW;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_in"]       = (float)(random(180, 350)) / 10.0f;
    data["rh_in"]      = (float)(random(300, 900)) / 10.0f;
    data["soil_moist"] = random(20, 100);
    data["lux_in"]     = random(0, 50000);

    serializeJson(doc, json, sizeof(json));
    ipc_uart_send_json(json);
    Serial.print("[TX TELE INT] ");
    Serial.println(json);

    /* state (atuadores) */
    send_state_now();
}

/* --------- heartbeat --------- */

static void send_hb()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             NODE_MSH_GW,
                             NODE_BLYNK_GW,
                             (int)(millis() / 1000),
                             -60,
                             json,
                             sizeof(json))) {
        return;
    }

    ipc_uart_send_json(json);
    Serial.print("[TX HB] ");
    Serial.println(json);
}

/* --------- aplicar CFG recebido --------- */

static void apply_cfg(const mesh_msg_t &msg)
{
    if (msg.cfg.has_mode)
        g_mode = msg.cfg.mode;

    if (msg.cfg.has_intake_pwm)
        g_intake_pwm = msg.cfg.intake_pwm;

    if (msg.cfg.has_exhaust_pwm)
        g_exhaust_pwm = msg.cfg.exhaust_pwm;

    if (msg.cfg.has_humidifier)
        g_humidifier = msg.cfg.humidifier;

    if (msg.cfg.has_irrigation)
        g_irrigation = msg.cfg.irrigation;

    if (msg.cfg.has_led_pwm)
        g_led_pwm = msg.cfg.led_pwm;

    if (msg.cfg.has_led_rgb)
        strncpy(g_led_rgb, msg.cfg.led_rgb, sizeof(g_led_rgb) - 1);

    Serial.printf("[CFG] mode=%d, intake=%d, exhaust=%d, hum=%d, irr=%d, led_pwm=%d, led_rgb=%s\n",
                  g_mode, g_intake_pwm, g_exhaust_pwm,
                  g_humidifier, g_irrigation, g_led_pwm, g_led_rgb);

    // após aplicar cfg, envia STATE imediato
    send_state_now();

    // se cfg era QoS1, envia ACK "ok" via lib
    mesh_proto_qos_send_ack_ok(&msg);
}

/* --------- handlers extras (HELLO/EVT/TIME) --------- */

static void handle_hello(const mesh_msg_t &msg)
{
    Serial.print("[HELLO RX] node_id=");
    if (msg.hello.has_node_id) Serial.print(msg.hello.node_id);
    Serial.print(" fw=");
    if (msg.hello.has_fw_ver) Serial.print(msg.hello.fw_ver);
    Serial.print(" extra=");
    if (msg.hello.has_extra) Serial.print(msg.hello.extra);
    Serial.println();
}

static void handle_evt(const mesh_msg_t &msg)
{
    Serial.print("[EVT RX] from=");
    Serial.print(msg.src);
    Serial.print(" event=");
    if (msg.evt.has_event) Serial.print(msg.evt.event);
    Serial.print(" code=");
    if (msg.evt.has_code) Serial.print(msg.evt.code);
    Serial.print(" level=");
    if (msg.evt.has_level) Serial.print(msg.evt.level);
    Serial.println();
}

static void handle_time(const mesh_msg_t &msg)
{
    Serial.print("[TIME RX] epoch=");
    if (msg.time_sync.has_epoch) Serial.print(msg.time_sync.epoch);
    Serial.print(" tz_offset_min=");
    if (msg.time_sync.has_tz_offset_min) Serial.print(msg.time_sync.tz_offset_min);
    Serial.println();

    // futuro: ajustar RTC
}

/* --------- processar JSON recebido via UART --------- */

static void handle_uart_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[RX] parse fail: ");
        Serial.println(json);
        return;
    }

    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, "*") != 0) {
        return;
    }

    switch (msg.type) {
    case MESH_MSG_CFG:
        apply_cfg(msg);
        break;

    case MESH_MSG_HELLO:
        handle_hello(msg);
        break;

    case MESH_MSG_EVT:
        handle_evt(msg);
        break;

    case MESH_MSG_TIME:
        handle_time(msg);
        break;

    case MESH_MSG_ACK:
        // se um dia esse nó enviar QoS1 para outro, ACKs serão tratados aqui:
        mesh_proto_qos_on_ack(&msg);
        break;

    default:
        Serial.print("[RX] tipo não tratado: ");
        Serial.println(msg.type);
        break;
    }
}

/* --------- setup / loop --------- */

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("[mesh_gateway_module] boot");

    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);

    // QoS também inicializado aqui (mesmo que, por enquanto, só ACK seja usado)
    mesh_proto_qos_init(ipc_uart_send_json);

    randomSeed(esp_random());
}

void loop()
{
    unsigned long now = millis();

    if (now - g_last_tele >= 60000UL) {
        g_last_tele = now;
        send_tele_dump();
    }

    if (now - g_last_hb >= 10000UL) {
        g_last_hb = now;
        send_hb();
    }

    char json[256];
    if (ipc_uart_read_json(json, sizeof(json))) {
        Serial.print("[RX JSON] ");
        Serial.println(json);
        handle_uart_json(json);
    }

    // se este nó tiver mensagens QoS1 pendentes no futuro, o retry é tratado aqui
    mesh_proto_qos_poll();
}
