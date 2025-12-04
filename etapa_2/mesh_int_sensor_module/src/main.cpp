/**
 * @file main.cpp
 * @brief Nó simulador int-sen-00 / ext-sen-00 / act-00
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

#include "credentials.h"   // MESH_PREFIX, MESH_PASSWORD, MESH_PORT
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_INT        "int-sen-00"
#define NODE_EXT        "ext-sen-00"
#define NODE_ACT        "act-00"
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

// neste simulador, todos os dados vão para o blynk-gw
#define NODE_SINK       NODE_BLYNK_GW

// -----------------------------------------------------------------------------
// Objetos da mesh
// -----------------------------------------------------------------------------
Scheduler       userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Estado simulado dos atuadores (act-00)
// -----------------------------------------------------------------------------
static int  g_mode        = 0;        // 0=AUTO, 1=MANUAL (igual ao V13 do mock Python)
static int  g_intake_pwm  = 0;
static int  g_exhaust_pwm = 0;
static int  g_humidifier  = 0;
static int  g_irrigation  = 0;
static int  g_led_pwm     = 0;        // usado como "led_brig"
static char g_led_rgb[16] = "0x000000";

// -----------------------------------------------------------------------------
// Estado simulado dos sensores (interno/externo)
// -----------------------------------------------------------------------------
static float g_t_in       = 25.0f;
static float g_rh_in      = 60.0f;
static int   g_soil_moist = 60;
static int   g_lux_in     = 5000;

static float g_t_out      = 27.0f;
static float g_rh_out     = 55.0f;
static int   g_lux_out    = 20000;

// -----------------------------------------------------------------------------
// Controle de envio / contadores
// -----------------------------------------------------------------------------
static uint16_t      g_msg_counter   = 0;
static unsigned long g_last_tele     = 0;
static unsigned long g_last_hb_all   = 0;
// STATE agora só é enviado em:
//  - primeira conexão mesh
//  - após aplicar CFG
// então não precisamos de g_last_state

static bool          g_hello_sent    = false;

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
 * Callback usado pela lib mesh_proto para enviar JSONs QoS1 (ACK, HELLO, TIME, EVT, etc.)
 * via rede mesh.
 */
static void mesh_send_json_cb(const char *json)
{
    // Usamos broadcast; o campo "dst" no JSON define quem deve processar.
    mesh.sendBroadcast(json);
    Serial.print("[MESH TX] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// Simulação de ambiente (apenas SENSORES, sem mexer em atuadores)
// -----------------------------------------------------------------------------

static void simulate_sensors_step()
{
    // Ambiente externo
    g_t_out   += (float)random(-5, 6) / 10.0f;   // ±0.5 °C
    g_rh_out  += (float)random(-8, 9) / 10.0f;   // ±0.8 %
    g_lux_out += random(-3000, 3001);            // ±3000 lux

    g_t_out   = clampf(g_t_out,   18.0f, 36.0f);
    g_rh_out  = clampf(g_rh_out,  30.0f, 90.0f);
    g_lux_out = clampi(g_lux_out,     0, 60000);

    // Ambiente interno
    g_t_in   += (float)random(-4, 5) / 10.0f;    // ±0.4 °C
    g_rh_in  += (float)random(-7, 8) / 10.0f;    // ±0.7 %
    g_t_in    = clampf(g_t_in,   20.0f, 34.0f);
    g_rh_in   = clampf(g_rh_in,  35.0f, 95.0f);

    g_soil_moist += random(-3, 4);              // ±3 %
    g_soil_moist  = clampi(g_soil_moist, 20, 100);

    g_lux_in += random(-2500, 2501);            // ±2500 lux
    g_lux_in  = clampi(g_lux_in, 0, 40000);
}

// -----------------------------------------------------------------------------
// Envio de TELE / STATE / HB / HELLO
// -----------------------------------------------------------------------------

/**
 * Envia um "dump" de telemetria:
 *   - TELE de ext-sen-00
 *   - TELE de int-sen-00
 * Chamado a cada 60 s.
 */
static void send_tele_dump()
{
    JsonDocument doc;
    JsonObject   data;
    char id[8];
    char json[256];

    // Atualiza somente os SENSORES
    simulate_sensors_step();

    // --- TELE externa (ext-sen-00) ---
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_EXT;
    doc["dst"]  = NODE_SINK;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_out"]   = g_t_out;
    data["rh_out"]  = g_rh_out;
    data["lux_out"] = g_lux_out;

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);

    // --- TELE interna (int-sen-00) ---
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_INT;
    doc["dst"]  = NODE_SINK;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_in"]       = g_t_in;
    data["rh_in"]      = g_rh_in;
    data["soil_moist"] = g_soil_moist;
    data["lux_in"]     = g_lux_in;

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);
}

/**
 * Envia STATE atual dos atuadores (act-00).
 * Usado:
 *   - logo após primeira conexão mesh
 *   - sempre que um CFG de atuador for aplicado.
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
                                    NODE_SINK,
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
 * Envia heartbeat de um nó lógico.
 */
static void send_hb_for(const char *node_id, int rssi_dbm)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             node_id,
                             NODE_SINK,
                             (int)(millis() / 1000),
                             rssi_dbm,
                             json,
                             sizeof(json))) {
        return;
    }

    mesh_send_json_cb(json);
}

/**
 * Envia heartbeat de TODOS os nós simulados:
 *   - int-sen-00
 *   - ext-sen-00
 *   - act-00
 */
static void send_hb_all()
{
    int base_rssi = -60;
    send_hb_for(NODE_INT, base_rssi + random(-3, 4));
    send_hb_for(NODE_EXT, base_rssi + random(-3, 4));
    send_hb_for(NODE_ACT, base_rssi + random(-3, 4));
}

/**
 * Envia HELLO de um nó lógico (apresentação ao mesh_gateway).
 *   - qos=1
 *   - dst="msh-gw"
 *   - vai pelo gerenciador de QoS (mesh_proto_qos_register_and_send)
 */
static void send_hello_for(const char *node_id,
                           const char *fw_ver,
                           const char *extra)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hello(id,
                                (uint32_t)millis(),
                                1,              // QoS1
                                node_id,        // src lógico
                                NODE_MSH_GW,    // dst = mesh-gw
                                node_id,
                                fw_ver,
                                extra,
                                json,
                                sizeof(json))) {
        return;
    }

    // QoS1 com retry automático
    if (!mesh_proto_qos_register_and_send(id, json)) {
        // fallback se a fila de QoS estiver cheia
        mesh_send_json_cb(json);
    }
}

/**
 * Envia HELLO de todos os nós simulados.
 * Chamado uma vez após a mesh estar conectada.
 */
static void send_hello_all()
{
    send_hello_for(NODE_INT, "1.0.0-sim", "internal sensors");
    send_hello_for(NODE_EXT, "1.0.0-sim", "external sensors");
    send_hello_for(NODE_ACT, "1.0.0-sim", "actuators");
}

// -----------------------------------------------------------------------------
// Aplicar CFG recebido (comportamento de "act-00")
// -----------------------------------------------------------------------------

/**
 * Aplica CFG recebido do Blynk (via mesh_gateway):
 *
 * - g_mode (AUTO/MANUAL) segue exatamente a lógica do mock Python:
 *   * mode = 0 (AUTO): comandos de atuadores são ignorados.
 *   * mode = 1 (MANUAL): comandos de atuadores são aplicados.
 *
 * - Após aplicar (ou ignorar) os comandos, envia STATE imediato
 *   e responde ACK "ok" se o CFG for QoS1.
 */
static void apply_cfg(const mesh_msg_t &msg)
{
    // 1) Atualiza modo primeiro, se vier no CFG
    if (msg.cfg.has_mode) {
        g_mode = msg.cfg.mode;
    }

    // 2) Só aplica comandos de atuador se estivermos em modo MANUAL (1)
    if (g_mode == 1) {
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
    } else {
        // Em AUTO ignoramos comandos de atuador (equivalente ao mock Python)
        if (msg.cfg.has_intake_pwm || msg.cfg.has_exhaust_pwm ||
            msg.cfg.has_humidifier || msg.cfg.has_irrigation ||
            msg.cfg.has_led_pwm   || msg.cfg.has_led_rgb) {
            Serial.printf("[CFG] modo=%d (AUTO), ignorando comandos de atuadores\n", g_mode);
        }
    }

    Serial.printf("[CFG] mode=%d, intake=%d, exhaust=%d, hum=%d, irr=%d, led_pwm=%d, led_rgb=%s\n",
                  g_mode, g_intake_pwm, g_exhaust_pwm,
                  g_humidifier, g_irrigation, g_led_pwm, g_led_rgb);

    // 3) Envia STATE imediato refletindo o estado atual
    send_state_now();

    // 4) Se o CFG era QoS1, envia ACK "ok" via lib (vai pela mesh)
    mesh_proto_qos_send_ack_ok(&msg);
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

    // 1) ACK: SEMPRE avisar o gerenciador de QoS, independente do dst lógico
    if (msg.type == MESH_MSG_ACK) {
        mesh_proto_qos_on_ack(&msg);
        Serial.printf("[RX ACK] ref=%s from=%s dst=%s\n",
                      msg.ack.ref, msg.src, msg.dst);
        return;
    }

    // 2) Demais tipos: aplica filtro de destino lógico
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, NODE_ACT)    != 0 &&
        strcmp(msg.dst, NODE_INT)    != 0 &&
        strcmp(msg.dst, NODE_EXT)    != 0 &&
        strcmp(msg.dst, "*")         != 0) {
        Serial.printf("[RX] ignorado dst=%s\n", msg.dst);
        return;
    }

    switch (msg.type) {
    case MESH_MSG_CFG:
        apply_cfg(msg);
        break;

    case MESH_MSG_TIME:
        Serial.println("[TIME] recebido (simulado, não aplicado)");
        break;

    default:
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

    // Só envia HELLO uma vez, na primeira conexão com a rede mesh
    if (!g_hello_sent) {
        g_hello_sent = true;
        Serial.println("[MESH] Primeira conexao estabelecida, enviando HELLO QoS1 de todos os nos simulados...");
        send_hello_all();

        // STATE inicial com todos atuadores em 0
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
    Serial.println("[int-sen-00-sim] boot");

    randomSeed(esp_random());

    // Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    // Inicializa QoS da lib (usado para HELLO QoS1, ACK de CFG, etc.)
    mesh_proto_qos_init(mesh_send_json_cb);

    // Estado inicial do ambiente (aleatoriza um pouco)
    g_t_in       += (float)random(-10, 11) / 10.0f;
    g_rh_in      += (float)random(-15, 16) / 10.0f;
    g_soil_moist += random(-10, 11);
    g_lux_in     += random(-3000, 3001);

    g_t_out      += (float)random(-10, 11) / 10.0f;
    g_rh_out     += (float)random(-15, 16) / 10.0f;
    g_lux_out    += random(-5000, 5001);

    g_soil_moist = clampi(g_soil_moist, 20, 100);
    g_lux_in     = clampi(g_lux_in, 0, 40000);
    g_lux_out    = clampi(g_lux_out, 0, 60000);

    // Atuadores já estão inicializados em 0 (estado default do mock Python)

    Serial.println("[int-sen-00-sim] mesh inicializada");
}

void loop()
{
    mesh.update();

    unsigned long now = millis();

    // TELE periódica (sensores interno + externo) a cada 60 s
    if (now - g_last_tele >= 60000UL) {   // 60 s
        g_last_tele = now;
        send_tele_dump();
    }

    // Heartbeat de todos os nós simulados a cada 10 s
    if (now - g_last_hb_all >= 10000UL) { // 10 s
        g_last_hb_all = now;
        send_hb_all();
    }

    // Gerenciador de QoS (retry de mensagens QoS1, ex: HELLO)
    mesh_proto_qos_poll();
}
