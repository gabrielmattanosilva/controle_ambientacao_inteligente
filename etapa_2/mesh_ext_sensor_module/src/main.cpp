/**
 * @file main.cpp
 * @brief Nó ext-sen-00 (sensores reais)
 *
 * Baseado no firmware do int-sen-00 (sensores reais), mantendo a mesma lógica:
 *  - Envia TELE periódica (60s) para o blynk-gw
 *  - Envia HB (10s) do ext-sen-00
 *  - Envia HELLO QoS1 uma vez ao conectar na mesh
 *  - Faz mesh_proto_qos_poll() no loop (retries QoS1)
 *
 * Alterações pedidas:
 *  - Nó renomeado para "ext-sen-00"
 *  - Removido sensor de umidade do solo
 *  - TELE agora publica: t_out, rh_out, lux_out
 *
 * Sensores reais (I2C):
 *  - AHTX0 (temperatura/umidade)
 *  - BH1750 (lux)
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <BH1750.h>

#include "credentials.h"
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_EXT        "ext-sen-00"
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

// -----------------------------------------------------------------------------
// Objetos da mesh
// -----------------------------------------------------------------------------
Scheduler       userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Sensores reais
// -----------------------------------------------------------------------------
static Adafruit_AHTX0 g_aht;
static BH1750         g_bh1750;

static bool g_aht_ok = false;
static bool g_bh_ok  = false;

// -----------------------------------------------------------------------------
// Últimos valores lidos (externo)
// -----------------------------------------------------------------------------
static float g_t_out   = NAN;
static float g_rh_out  = NAN;
static int   g_lux_out = -1;   // lux

// -----------------------------------------------------------------------------
// Controle de envio
// -----------------------------------------------------------------------------
static uint16_t      g_msg_counter = 0;
static unsigned long g_last_tele   = 0;
static unsigned long g_last_hb     = 0;
static bool          g_hello_sent  = false;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

static float clampf(float v, float vmin, float vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

static int clampi(int v, int vmin, int vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

/**
 * Callback usado pela lib mesh_proto para enviar JSONs QoS1 (ACK, HELLO, TIME, etc.)
 * via rede mesh.
 */
static void mesh_send_json_cb(const char *json)
{
    mesh.sendBroadcast(json);
    Serial.print("[MESH TX] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// Leitura dos sensores reais (externo)
// -----------------------------------------------------------------------------
static void read_sensors_step()
{
    // AHT (temp/umidade)
    if (g_aht_ok) {
        sensors_event_t hum, temp;
        bool ok = g_aht.getEvent(&hum, &temp);
        if (ok) {
            g_t_out  = temp.temperature;
            g_rh_out = hum.relative_humidity;
        } else {
            Serial.println("[SENS] AHT getEvent() falhou (mantendo últimos valores)");
        }
    }

    // BH1750 (lux)
    if (g_bh_ok) {
        float lux = g_bh1750.readLightLevel();
        if (!isnan(lux) && lux >= 0.0f) {
            g_lux_out = (int)lroundf(lux);
        } else {
            Serial.println("[SENS] BH1750 readLightLevel() invalido (mantendo últimos valores)");
        }
    }

    // Sanity clamp (opcional)
    if (!isnan(g_t_out))  g_t_out  = clampf(g_t_out,  -10.0f,  80.0f);
    if (!isnan(g_rh_out)) g_rh_out = clampf(g_rh_out,   0.0f, 100.0f);
    if (g_lux_out >= 0)   g_lux_out = clampi(g_lux_out, 0, 200000);
}

// -----------------------------------------------------------------------------
// Envio de TELE / HB / HELLO
// -----------------------------------------------------------------------------
static void send_tele_dump()
{
    JsonDocument doc;
    JsonObject   data;
    char id[8];
    char json[256];

    // Atualiza sensores reais
    read_sensors_step();

    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_EXT;
    doc["dst"]  = NODE_BLYNK_GW;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    if (!isnan(g_t_out))   data["t_out"]   = g_t_out;
    if (!isnan(g_rh_out))  data["rh_out"]  = g_rh_out;
    if (g_lux_out >= 0)    data["lux_out"] = g_lux_out;

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);
}

static void send_hb()
{
    int rssi = WiFi.RSSI();
    if (rssi == 0) rssi = -60;

    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             NODE_EXT,
                             NODE_BLYNK_GW,
                             (int)(millis() / 1000),
                             rssi,
                             json,
                             sizeof(json))) {
        return;
    }

    mesh_send_json_cb(json);
}

static void send_hello()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hello(id,
                                (uint32_t)millis(),
                                1,              // QoS1
                                NODE_EXT,
                                NODE_MSH_GW,
                                NODE_EXT,
                                "1.0.0-real",
                                "external sensors (real)",
                                json,
                                sizeof(json))) {
        return;
    }

    // Registra para retry; se falhar (ex: tabela cheia), envia sem retry
    if (!mesh_proto_qos_register_and_send(id, json)) {
        mesh_send_json_cb(json);
    }
}

// -----------------------------------------------------------------------------
// Handlers de mensagens da mesh
// -----------------------------------------------------------------------------
static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[RX] parse fail: ");
        Serial.println(json ? json : "(null)");
        return;
    }

    // ACK QoS1
    if (msg.type == MESH_MSG_ACK) {
        mesh_proto_qos_on_ack(&msg);
        Serial.printf("[RX ACK] ref=%s from=%s dst=%s\n",
                      msg.ack.ref, msg.src, msg.dst);
        return;
    }

    // Aceita mensagens destinadas ao gateway mesh (msh-gw), a este nó, ou broadcast (*)
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, NODE_EXT)    != 0 &&
        strcmp(msg.dst, "*")         != 0) {
        return;
    }

    switch (msg.type) {
    case MESH_MSG_TIME:
        Serial.println("[TIME] recebido (não aplicado neste nó)");
        if (msg.qos == 1) {
            mesh_proto_qos_send_ack_ok(&msg);
        }
        break;

    case MESH_MSG_CFG:
        Serial.printf("[RX CFG] recebido dst=%s (não aplicado aqui)\n", msg.dst);
        if (msg.qos == 1) {
            mesh_proto_qos_send_ack_ok(&msg);
        }
        break;

    default:
        Serial.printf("[RX] tipo não tratado=%d dst=%s\n", msg.type, msg.dst);
        break;
    }
}

// -----------------------------------------------------------------------------
// Callbacks da painlessMesh
// -----------------------------------------------------------------------------
void mesh_receive_cb(uint32_t from, String &msg)
{
    Serial.printf("[MESH RX] from %u: %s\n", from, msg.c_str());
    handle_mesh_json(msg.c_str());
}

void mesh_new_connection_cb(uint32_t nodeId)
{
    Serial.printf("[MESH] New connection, nodeId=%u\n", nodeId);

    if (!g_hello_sent) {
        g_hello_sent = true;
        Serial.println("[MESH] Primeira conexao estabelecida, enviando HELLO QoS1...");
        send_hello();
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

    Serial.println("========================================");
    Serial.println("[ext-sen-00] Sensor Node Boot (REAL)");
    Serial.println("========================================");

    // I2C
    Wire.begin();

    // Inicializa sensores reais
    g_aht_ok = g_aht.begin();
    if (g_aht_ok) Serial.println("[SENS] AHTX0 OK");
    else          Serial.println("[SENS] AHTX0 FALHOU");

    g_bh_ok = g_bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
    if (g_bh_ok) Serial.println("[SENS] BH1750 OK");
    else         Serial.println("[SENS] BH1750 FALHOU");

    // Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    // QoS da lib (HELLO QoS1, ACK, etc.)
    mesh_proto_qos_init(mesh_send_json_cb);

    // Primeira leitura no boot
    read_sensors_step();

    Serial.printf("[SENSOR NODE] Publicando: %s (sensores reais)\n", NODE_EXT);
    Serial.println("========================================");
}

void loop()
{
    mesh.update();

    unsigned long now = millis();

    // TELE periódica a cada 60 s
    if (now - g_last_tele >= 60000UL) {
        g_last_tele = now;
        send_tele_dump();
    }

    // Heartbeat a cada 10 s
    if (now - g_last_hb >= 10000UL) {
        g_last_hb = now;
        send_hb();
    }

    // QoS (retry de mensagens QoS1, ex: HELLO)
    mesh_proto_qos_poll();
}
