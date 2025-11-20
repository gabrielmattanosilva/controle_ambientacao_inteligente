/**
 * @file main.cpp
 * @brief blynk_gateway_module
 */

#include "credentials.h"
#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ArduinoJson.h>

#include "pins.h"
#include "ipc_uart.h"

/* ------------------ Node IDs --------------------- */

#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"
#define NODE_EXT_SEN    "ext-sen-00"
#define NODE_INT_SEN    "int-sen-00"
#define NODE_ACT        "act-00"

/* ------------------ Estado global ---------------- */

static BlynkTimer g_timer;
static uint16_t   g_msg_counter = 0;
static int        g_mode        = 0;

/* ------------------ Helpers ---------------------- */

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

/* Envia campo STRING */
static void send_cfg_str(const char *key, const char *val)
{
    JsonDocument doc;
    char idb[8]; gen_msg_id(idb, sizeof(idb));

    doc["id"]  = idb;
    doc["ts"]  = (uint32_t)millis();
    doc["qos"] = 1;
    doc["src"] = NODE_BLYNK_GW;
    doc["dst"] = NODE_MSH_GW;
    doc["type"] = "cfg";

    JsonObject data = doc["data"].to<JsonObject>();
    data[key] = val;

    char json[256];
    serializeJson(doc, json, sizeof(json));

    ipc_uart_send_json(json);

    Serial.print("[TX CFG] ");
    Serial.println(json);
}

/* Envia campo NUMÉRICO */
static void send_cfg_int(const char *key, int val)
{
    JsonDocument doc;
    char idb[8]; gen_msg_id(idb, sizeof(idb));

    doc["id"]  = idb;
    doc["ts"]  = (uint32_t)millis();
    doc["qos"] = 1;
    doc["src"] = NODE_BLYNK_GW;
    doc["dst"] = NODE_MSH_GW;
    doc["type"] = "cfg";

    JsonObject data = doc["data"].to<JsonObject>();
    data[key] = val;  /* IMPORTANTE: número */

    char json[256];
    serializeJson(doc, json, sizeof(json));

    ipc_uart_send_json(json);

    Serial.print("[TX CFG] ");
    Serial.println(json);
}

/* ------------------ Blynk callbacks ---------------- */

BLYNK_CONNECTED()
{
    Serial.println("[BLYNK] Connected, syncing V13..V19");
    Blynk.syncVirtual(V13, V14, V15, V16, V17, V18, V19);
}

/* MODO */
BLYNK_WRITE(V13)
{
    g_mode = param.asInt() ? 1 : 0;
    Serial.printf("[BLYNK] mode=%d\n", g_mode);
    send_cfg_int("mode", g_mode);
}

/* --- COMANDOS (alinhado ao mock python) ---
 * V14 = intake_pwm
 * V15 = exhaust_pwm
 * V16 = humidifier
 * V17 = led_pwm  (led_brig)
 * V18 = led_rgb
 * V19 = irrigation
 */

BLYNK_WRITE(V14)
{
    send_cfg_int("intake_pwm", param.asInt());
}
BLYNK_WRITE(V15)
{
    send_cfg_int("exhaust_pwm", param.asInt());
}
BLYNK_WRITE(V16)
{
    send_cfg_int("humidifier", param.asInt());
}
BLYNK_WRITE(V17)
{
    send_cfg_int("led_pwm", param.asInt());
}
BLYNK_WRITE(V18)
{
    send_cfg_str("led_rgb", param.asStr());
}
BLYNK_WRITE(V19)
{
    send_cfg_int("irrigation", param.asInt());
}

/* ------------------ Processar TELE ---------------- */

static void handle_tele(JsonObject d)
{
    if (d["t_out"].is<float>())      Blynk.virtualWrite(0, d["t_out"].as<float>());
    if (d["rh_out"].is<float>())     Blynk.virtualWrite(1, d["rh_out"].as<float>());
    if (d["lux_out"].is<long>())     Blynk.virtualWrite(2, (int)d["lux_out"].as<long>());

    if (d["t_in"].is<float>())       Blynk.virtualWrite(3, d["t_in"].as<float>());
    if (d["rh_in"].is<float>())      Blynk.virtualWrite(4, d["rh_in"].as<float>());
    if (d["soil_moist"].is<long>())  Blynk.virtualWrite(5, (int)d["soil_moist"].as<long>());
    if (d["lux_in"].is<long>())      Blynk.virtualWrite(6, (int)d["lux_in"].as<long>());
}

/* ------------------ Processar STATE ---------------- */

static void handle_state(JsonObject d)
{
    if (d["intake_pwm"].is<long>())
    Blynk.virtualWrite(7, (int)d["intake_pwm"].as<long>());

    if (d["exhaust_pwm"].is<long>())
        Blynk.virtualWrite(8, (int)d["exhaust_pwm"].as<long>());

    if (d["humidifier"].is<long>())
        Blynk.virtualWrite(9, (int)d["humidifier"].as<long>());

    if (d["led_brig"].is<long>())
        Blynk.virtualWrite(10, (int)d["led_brig"].as<long>());

    if (d["led_rgb"].is<const char*>()) {
        const char* rgb = d["led_rgb"].as<const char*>();   // <-- EXTRAÇÃO ANTES
        Blynk.virtualWrite(11, rgb);
    }
    if (d["irrigation"].is<long>())
        Blynk.virtualWrite(12, (int)d["irrigation"].as<long>());
}

/* ------------------ Processar HB ---------------- */

static void handle_hb(const char *src, JsonObject d)
{
    int up = d["uptime_s"].is<long>() ? (int)d["uptime_s"].as<long>() : 0;

    if (strcmp(src, NODE_MSH_GW)==0)      Blynk.virtualWrite(20, up);
    else if (strcmp(src, NODE_EXT_SEN)==0)Blynk.virtualWrite(21, up);
    else if (strcmp(src, NODE_INT_SEN)==0)Blynk.virtualWrite(22, up);
    else if (strcmp(src, NODE_ACT)==0)    Blynk.virtualWrite(23, up);
}

/* ------------------ Incoming JSON ---------------- */

static void handle_mesh_json(const char *json)
{
    JsonDocument doc;

    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        Serial.print("[JSON] ERROR: ");
        Serial.println(err.c_str());
        return;
    }

    const char *dst  = doc["dst"]  | "";
    const char *type = doc["type"] | "";
    const char *src  = doc["src"]  | "";

    if (strcmp(dst, NODE_BLYNK_GW) != 0 &&
        strcmp(dst, "*") != 0)
        return;

    JsonObject data = doc["data"].as<JsonObject>();
    if (!data) return;

    if (!strcmp(type,"tele"))       handle_tele(data);
    else if (!strcmp(type,"state")) handle_state(data);
    else if (!strcmp(type,"hb"))    handle_hb(src, data);
}

/* ------------------ setup / loop ---------------- */

void setup()
{
    Serial.begin(115200);
    delay(500);

    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status()!=WL_CONNECTED) { delay(250); }

    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();

    g_timer.setInterval(10000L, [](){
        long up = millis()/1000;
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
}
