/**
 * @file main.cpp
 * @brief Nó ext-sen-00 (SIMULADO)
 *
 * Mantém a mesma lógica do nó anterior:
 *  - Envia TELE periódica (60s) para o blynk-gw
 *  - Envia HB (10s)
 *  - Envia HELLO QoS1 uma vez ao conectar na mesh
 *  - Faz mesh_proto_qos_poll() no loop
 *
 * Diferença:
 *  - Não usa sensores reais: simula apenas o ext-sen-00 (t_out, rh_out, lux_out)
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <WiFi.h>

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
Scheduler userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Estado simulado (ext)
// -----------------------------------------------------------------------------
static float g_t_out  = 26.5f;   // °C
static float g_rh_out = 55.0f;   // %
static int   g_lux_out = 1200;   // lux

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
// Simulação de sensores (ext)
// -----------------------------------------------------------------------------
static void simulate_ext_step()
{
    // Variações pequenas e suaves (random walk)
    float dt = ((int)random(-12, 13)) / 100.0f;   // -0.12 .. +0.12
    float dh = ((int)random(-25, 26)) / 100.0f;   // -0.25 .. +0.25
    int   dl = random(-150, 151);                 // -150 .. +150

    g_t_out  = clampf(g_t_out + dt,  10.0f, 45.0f);
    g_rh_out = clampf(g_rh_out + dh,  0.0f, 100.0f);
    g_lux_out = clampi(g_lux_out + dl, 0, 200000);

    // Pequena “tendência” diária simulada (opcional): depende de millis()
    // (mantém simples e determinístico)
}

// -----------------------------------------------------------------------------
// Envio de TELE / HB / HELLO
// -----------------------------------------------------------------------------
static void send_tele_dump()
{
    JsonDocument doc;
    JsonObject data;
    char id[8];
    char json[256];

    simulate_ext_step();

    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_EXT;
    doc["dst"]  = NODE_BLYNK_GW;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_out"]   = g_t_out;
    data["rh_out"]  = g_rh_out;
    data["lux_out"] = g_lux_out;

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
                                "1.0.0-sim",
                                "external sensor simulated",
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

    randomSeed((uint32_t)esp_random());

    Serial.println("========================================");
    Serial.println("[ext-sen-00] Sensor Node Boot (SIM)");
    Serial.println("========================================");

    // Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    // QoS da lib (HELLO QoS1, ACK, etc.)
    mesh_proto_qos_init(mesh_send_json_cb);

    Serial.printf("[SENSOR NODE] Publicando: %s (simulado)\n", NODE_EXT);
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
