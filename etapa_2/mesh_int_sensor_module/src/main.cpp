/**
 * @file main.cpp
 * @brief Nó int-sen-00
 *
 * Lógica mantida:
 *  - Envia TELE periódica (60s) para o blynk-gw
 *  - Envia HB (10s) do int-sen-00
 *  - Envia HELLO QoS1 uma vez ao conectar na mesh
 *  - Faz mesh_proto_qos_poll() no loop (retries QoS1)
 *
 * Sensores:
 *  - AHTX0 (temperatura/umidade) via I2C
 *  - BH1750 (lux) via I2C
 *  - Umidade do solo capacitiva (analógico) no GPIO34
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <WiFi.h>
#include <Wire.h>
#include "credentials.h"
#include "mesh_proto.h"
#include "pins.h"
#include "aht10_temp_hum.h"
#include "bh1750_lux.h"
#include "soil_hum_sensor.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_INT "int-sen-00"
#define NODE_BLYNK_GW "blynk-gw"
#define NODE_MSH_GW "msh-gw"

// -----------------------------------------------------------------------------
// Objetos da mesh
// -----------------------------------------------------------------------------
Scheduler userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Sensores (wrappers)
// -----------------------------------------------------------------------------
static aht10_temp_hum_t g_aht;
static bh1750_lux_t g_bh1750;
static soil_hum_sensor_t g_soil;

static bool g_aht_ok = false;
static bool g_bh_ok = false;

// -----------------------------------------------------------------------------
// Últimos valores lidos
// -----------------------------------------------------------------------------
static float g_t_in = NAN;
static float g_rh_in = NAN;
static int g_soil_moist = -1; // %
static int g_lux_in = -1;     // lux

// -----------------------------------------------------------------------------
// Controle de envio
// -----------------------------------------------------------------------------
static uint16_t g_msg_counter = 0;
static unsigned long g_last_tele = 0;
static unsigned long g_last_hb = 0;
static bool g_hello_sent = false;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter)
        g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
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
// Leitura dos sensores (via libs)
// -----------------------------------------------------------------------------
static void read_sensors_step()
{
    // AHT (temp/umidade)
    if (g_aht_ok)
    {
        float t = NAN, rh = NAN;
        bool ok = aht10_temp_hum_read(&g_aht, &t, &rh);
        if (ok)
        {
            g_t_in = t;
            g_rh_in = rh;
        }
        else
        {
            Serial.println("[SENS] AHT getEvent() falhou (mantendo últimos valores)");
        }
    }

    // BH1750 (lux)
    if (g_bh_ok)
    {
        float lux = NAN;
        bool ok = bh1750_lux_read(&g_bh1750, &lux);
        if (ok && !isnan(lux) && lux >= 0.0f)
        {
            g_lux_in = (int)lroundf(lux);
        }
        else
        {
            Serial.println("[SENS] BH1750 readLightLevel() invalido (mantendo últimos valores)");
        }
    }

    // Umidade do solo (analógico) -> %
    g_soil_moist = soil_hum_sensor_read_percent(&g_soil);
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

    // Atualiza sensores
    read_sensors_step();

    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"] = id;
    doc["ts"] = (uint32_t)millis();
    doc["qos"] = 0;
    doc["src"] = NODE_INT;
    doc["dst"] = NODE_BLYNK_GW;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    if (!isnan(g_t_in))
        data["t_in"] = g_t_in;
    if (!isnan(g_rh_in))
        data["rh_in"] = g_rh_in;
    if (g_soil_moist >= 0)
        data["soil_moist"] = g_soil_moist;
    if (g_lux_in >= 0)
        data["lux_in"] = g_lux_in;

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);
}

static void send_hb()
{
    int rssi = WiFi.RSSI();
    if (rssi == 0)
        rssi = -60;

    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             NODE_INT,
                             NODE_BLYNK_GW,
                             (int)(millis() / 1000),
                             rssi,
                             json,
                             sizeof(json)))
    {
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
                                1, // QoS1
                                NODE_INT,
                                NODE_MSH_GW,
                                NODE_INT, // node_id
                                "1.0.0",  // fw_ver (mantenha como você já usa)
                                "int-sen-00",
                                json,
                                sizeof(json)))
    {
        return;
    }

    // Registra para retry; se falhar (sem slot), já é enviado 1x (sem retry)
    mesh_proto_qos_register_and_send(id, json);
}

// -----------------------------------------------------------------------------
// RX / handler
// -----------------------------------------------------------------------------
static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg))
    {
        Serial.println("[RX] parse falhou");
        return;
    }

    // Se for ACK, notifica QoS manager
    if (msg.type == MESH_MSG_ACK)
    {
        mesh_proto_qos_on_ack(&msg);
        return;
    }

    // Só trata mensagens endereçadas a este nó ou broadcast
    if (strcmp(msg.dst, NODE_INT) != 0 &&
        strcmp(msg.dst, "*") != 0)
    {
        return;
    }

    switch (msg.type)
    {
    case MESH_MSG_TIME:
        Serial.println("[TIME] recebido (não aplicado neste nó)");
        if (msg.qos == 1)
        {
            mesh_proto_qos_send_ack_ok(&msg);
        }
        break;

    case MESH_MSG_CFG:
        Serial.printf("[RX CFG] recebido dst=%s (não aplicado aqui)\n", msg.dst);
        if (msg.qos == 1)
        {
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

    if (!g_hello_sent)
    {
        g_hello_sent = true;
        Serial.println("[MESH] Primeira conexao estabelecida, enviando HELLO QoS1.");
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
    Serial.println("[int-sen-00] Sensor Node Boot");
    Serial.println("========================================");

    // I2C
    Wire.begin();

    // Inicializa sensores (via libs)
    g_aht_ok = aht10_temp_hum_init(&g_aht);
    if (g_aht_ok)
        Serial.println("[SENS] AHTX0 OK");
    else
        Serial.println("[SENS] AHTX0 FALHOU");

    g_bh_ok = bh1750_lux_init(&g_bh1750, BH1750::CONTINUOUS_HIGH_RES_MODE);
    if (g_bh_ok)
        Serial.println("[SENS] BH1750 OK");
    else
        Serial.println("[SENS] BH1750 FALHOU");

    // Solo (ADC) - init inclui pinMode(INPUT)
    soil_hum_sensor_init(&g_soil, SOIL_MOISTURE_SENSOR, SOIL_DRY_VALUE, SOIL_WET_VALUE);

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

    Serial.printf("[SENSOR NODE] Publicando: %s\n", NODE_INT);
    Serial.println("========================================");
}

void loop()
{
    mesh.update();

    unsigned long now = millis();

    // TELE periódica a cada 60 s
    if (now - g_last_tele >= 60000UL)
    {
        g_last_tele = now;
        send_tele_dump();
    }

    // Heartbeat a cada 10 s
    if (now - g_last_hb >= 10000UL)
    {
        g_last_hb = now;
        send_hb();
    }

    // QoS (retry de mensagens QoS1, ex: HELLO)
    mesh_proto_qos_poll();
}
