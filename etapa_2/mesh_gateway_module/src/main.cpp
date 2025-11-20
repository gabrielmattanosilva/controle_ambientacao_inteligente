/**
 * @file main.cpp
 * @brief mesh_gateway_module – mock da rede mesh + bridge UART
 */

#include <Arduino.h>
#include <ArduinoJson.h>

#include "pins.h"
#include "ipc_uart.h"

/* ---------------- Node IDs -------------------- */

#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"
#define NODE_EXT_SEN    "ext-sen-00"
#define NODE_INT_SEN    "int-sen-00"
#define NODE_ACT        "act-00"

/* ---------------- Estado interno ---------------- */

static uint16_t g_msg_counter = 0;
static unsigned long g_last_tele = 0;
static unsigned long g_last_hb   = 0;

/* Atuadores reais (mock) */
static int  g_mode        = 0;
static int  g_intake_pwm  = 0;
static int  g_exhaust_pwm = 0;
static int  g_humidifier  = 0;
static int  g_irrigation  = 0;
static int  g_led_pwm     = 0;
static char g_led_rgb[16] = "FFD480";

/* ---------------- Helpers ---------------------- */

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

static void send_doc(JsonDocument &doc)
{
    char json[256];
    serializeJson(doc, json, sizeof(json));

    ipc_uart_send_json(json);

    Serial.print("[TX] ");
    Serial.println(json);
}

/* ---------------- Envio state ------------------ */

static void send_state_now()
{
    JsonDocument doc;
    JsonObject d;
    char idb[8];

    gen_msg_id(idb, sizeof(idb));
    doc["id"]   = idb;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 1;
    doc["src"]  = NODE_ACT;
    doc["dst"]  = NODE_BLYNK_GW;
    doc["type"] = "state";

    d = doc["data"].to<JsonObject>();
    d["intake_pwm"]  = g_intake_pwm;
    d["exhaust_pwm"] = g_exhaust_pwm;
    d["humidifier"]  = g_humidifier;
    d["led_brig"]    = g_led_pwm;
    d["led_rgb"]     = g_led_rgb;
    d["irrigation"]  = g_irrigation;

    send_doc(doc);
}

/* ---------------- Telemetria mock ---------------- */

static void send_tele_dump()
{
    JsonDocument doc;
    JsonObject d;
    char idb[8];

    /* externo */
    gen_msg_id(idb, sizeof(idb));
    doc.clear();
    doc["id"]=idb; doc["ts"]=(uint32_t)millis(); doc["qos"]=0;
    doc["src"]=NODE_EXT_SEN; doc["dst"]=NODE_BLYNK_GW; doc["type"]="tele";
    d = doc["data"].to<JsonObject>();
    d["t_out"]   = (float)(random(180,350))/10.0f;
    d["rh_out"]  = (float)(random(300,900))/10.0f;
    d["lux_out"] = random(0,50000);
    send_doc(doc);

    /* interno */
    gen_msg_id(idb, sizeof(idb));
    doc.clear();
    doc["id"]=idb; doc["ts"]=(uint32_t)millis(); doc["qos"]=0;
    doc["src"]=NODE_INT_SEN; doc["dst"]=NODE_BLYNK_GW; doc["type"]="tele";
    d = doc["data"].to<JsonObject>();
    d["t_in"]       = (float)(random(180,350))/10.0f;
    d["rh_in"]      = (float)(random(300,900))/10.0f;
    d["soil_moist"] = random(20,100);
    d["lux_in"]     = random(0,50000);
    send_doc(doc);

    /* state (atuadores reais) */
    send_state_now();
}

/* ---------------- Heartbeat -------------------- */

static void send_hb()
{
    JsonDocument doc;
    JsonObject d;
    char idb[8];

    gen_msg_id(idb, sizeof(idb));
    doc["id"]=idb; doc["ts"]=(uint32_t)millis(); doc["qos"]=0;
    doc["src"]=NODE_MSH_GW; doc["dst"]=NODE_BLYNK_GW; doc["type"]="hb";

    d = doc["data"].to<JsonObject>();
    d["uptime_s"] = (int)(millis()/1000);
    d["rssi_dbm"] = -60;

    send_doc(doc);
}

/* ---------------- Processar CFG ---------------- */

static void handle_cfg(JsonObject d)
{
    if (d["mode"].is<long>())
        g_mode = (int)d["mode"].as<long>();

    if (d["intake_pwm"].is<long>())
        g_intake_pwm = (int)d["intake_pwm"].as<long>();

    if (d["exhaust_pwm"].is<long>())
        g_exhaust_pwm = (int)d["exhaust_pwm"].as<long>();

    if (d["humidifier"].is<long>())
        g_humidifier = (int)d["humidifier"].as<long>();

    if (d["irrigation"].is<long>())
        g_irrigation = (int)d["irrigation"].as<long>();

    if (d["led_pwm"].is<long>())
        g_led_pwm = (int)d["led_pwm"].as<long>();

    if (d["led_rgb"].is<const char*>())
        strncpy(g_led_rgb, d["led_rgb"], sizeof(g_led_rgb)-1);

    /* STATE imediato */
    send_state_now();
}

/* ---------------- Processar JSON RX ---------------- */

static void process_json(const char *json)
{
    JsonDocument doc;
    if (deserializeJson(doc, json)) return;

    const char *dst  = doc["dst"]  | "";
    const char *type = doc["type"] | "";

    if (strcmp(dst, NODE_MSH_GW)!=0 &&
        strcmp(dst, "*")!=0)
        return;

    JsonObject data = doc["data"].as<JsonObject>();
    if (!data) return;

    if (!strcmp(type,"cfg"))
        handle_cfg(data);
}

/* ---------------- setup / loop ---------------- */

void setup()
{
    Serial.begin(115200);
    delay(500);

    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);

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
        Serial.print("[RX] ");
        Serial.println(json);
        process_json(json);
    }
}
