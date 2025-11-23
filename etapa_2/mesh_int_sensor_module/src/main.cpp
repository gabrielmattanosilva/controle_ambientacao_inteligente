/**
 * @file main.cpp
 * @brief Nó simulador int-sen-00
 *
 * Este firmware roda em um ESP32 separado e simula 3 nós lógicos:
 *   - "int-sen-00"  → sensores internos (t_in, rh_in, soil_moist, lux_in)
 *   - "ext-sen-00"  → sensores externos (t_out, rh_out, lux_out)
 *   - "act-00"      → atuadores (intake_pwm, exhaust_pwm, humidifier, led_brig/led_rgb, irrigation)
 *
 * Toda comunicação é feita via rede mesh (painlessMesh + mesh_proto),
 * conversando com o mesh_gateway_module, que faz a ponte até o Blynk.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include "credentials.h"
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_INT        "int-sen-00"
#define NODE_EXT        "ext-sen-00"
#define NODE_ACT        "act-00"
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

Scheduler userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Estado simulado dos atuadores (act-00)
// -----------------------------------------------------------------------------
static int  g_mode        = 0;        // 0=AUTO, 1=MANUAL, etc.
static int  g_intake_pwm  = 0;
static int  g_exhaust_pwm = 0;
static int  g_humidifier  = 0;
static int  g_irrigation  = 0;
static int  g_led_pwm     = 0;        // usado como "led_brig"
static char g_led_rgb[16] = "0x000000";

static uint16_t      g_msg_counter = 0;
static unsigned long g_last_tele   = 0;
static unsigned long g_last_hb     = 0;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

/**
 * Callback usado pela lib mesh_proto para enviar JSONs QoS1 (ACK, etc.)
 * via rede mesh.
 */
static void mesh_send_json_cb(const char *json)
{
    // Por simplicidade usamos broadcast; o campo "dst" no JSON
    // é o que define quem deve processar a mensagem.
    mesh.sendBroadcast(json);
    Serial.print("[MESH TX] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// Envio de TELE / STATE / HB
// -----------------------------------------------------------------------------

/**
 * Envia um "dump" de telemetria:
 *   - TELE de ext-sen-00
 *   - TELE de int-sen-00
 */
static void send_tele_dump()
{
    JsonDocument doc;
    JsonObject   data;
    char id[8];
    char json[256];

    // --- TELE externa (ext-sen-00) ---
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_EXT;
    doc["dst"]  = NODE_BLYNK_GW;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_out"]   = (float)(random(180, 350)) / 10.0f;   // 18.0 a 35.0 °C
    data["rh_out"]  = (float)(random(300, 900)) / 10.0f;   // 30 a 90 %
    data["lux_out"] = random(0, 50000);                   // 0 a 50k lux

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);

    // --- TELE interna (int-sen-00) ---
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_INT;
    doc["dst"]  = NODE_BLYNK_GW;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_in"]       = (float)(random(200, 320)) / 10.0f; // 20.0 a 32.0 °C
    data["rh_in"]      = (float)(random(400, 950)) / 10.0f; // 40 a 95 %
    data["soil_moist"] = random(20, 100);                  // 20 a 100 %
    data["lux_in"]     = random(0, 30000);                 // 0 a 30k lux

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);
}

/**
 * Envia STATE atual dos atuadores (act-00).
 * Usamos o builder da lib mesh_proto para manter o formato consistente.
 */
static void send_state_now()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_state_act(id,
                                    (uint32_t)millis(),
                                    0,              // QoS0
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

    mesh_send_json_cb(json);
}

/**
 * Envia heartbeat simples para monitorar o nó simulador.
 */
static void send_hb()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             NODE_INT,          // podemos usar o INT como "nó base"
                             NODE_BLYNK_GW,
                             (int)(millis() / 1000),
                             -60,               // RSSI simulado
                             json,
                             sizeof(json))) {
        return;
    }

    mesh_send_json_cb(json);
}

// -----------------------------------------------------------------------------
// Aplicar CFG recebido (comportamento de "act-00")
// -----------------------------------------------------------------------------

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

    // Após aplicar o cfg, envia STATE imediato para o Blynk
    send_state_now();

    // Se cfg era QoS1, envia ACK "ok" via lib (ack vai pela mesh, volta ao Blynk via gateway)
    mesh_proto_qos_send_ack_ok(&msg);
}

// -----------------------------------------------------------------------------
// Handlers de mensagens da mesh
// -----------------------------------------------------------------------------

/**
 * Processa uma mensagem JSON recebida pela mesh.
 * - Hoje nos importamos principalmente com CFG e TIME.
 */
static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[RX] parse fail: ");
        Serial.println(json);
        return;
    }

    // Só processamos msgs destinadas ao gateway mesh ou a este nó (compatível
    // com o cenário antigo em que dst="msh-gw").
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, NODE_ACT)    != 0 &&
        strcmp(msg.dst, "*")        != 0) {
        Serial.printf("[RX] ignorado dst=%s\n", msg.dst);
        return;
    }

    switch (msg.type) {
    case MESH_MSG_CFG:
        apply_cfg(msg);
        break;

    case MESH_MSG_TIME:
        // Opcional: poderia fazer ajuste de RTC aqui se desejar
        Serial.println("[TIME] recebido (simulado, não aplicado)");
        break;

    default:
        // Outros tipos (HELLO, EVT, TELE, STATE, HB, ACK) podem ser ignorados aqui
        Serial.printf("[RX] tipo não tratado=%d\n", msg.type);
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
    Serial.println("[int-sen-00] boot");

    randomSeed(esp_random());

    // Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    // Inicializa QoS da lib (usado para enviar ACK de CFG)
    mesh_proto_qos_init(mesh_send_json_cb);

    Serial.println("[int-sen-00] mesh inicializada");
}

void loop()
{
    mesh.update();

    unsigned long now = millis();

    if (now - g_last_tele >= 60000UL) {
        g_last_tele = now;
        send_tele_dump();
    }

    if (now - g_last_hb >= 10000UL) {
        g_last_hb = now;
        send_hb();
    }

    // Este nó só usa QoS para ACK, que é imediato; não há retry aqui.
    // Se futuramente quiser enviar EVT QoS1, basta usar:
    //   mesh_proto_qos_register_and_send(id, json);
    // e chamar mesh_proto_qos_poll() aqui no loop.
}
