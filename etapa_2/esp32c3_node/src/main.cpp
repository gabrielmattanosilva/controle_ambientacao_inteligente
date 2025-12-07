/**
 * @file main.cpp
 * @brief Nó simulador act-00 (Atuadores)
 *
 * Este firmware roda em um ESP32-C3 SuperMini e simula o nó lógico de ATUADORES:
 *   - "act-00" → controla intake_pwm, exhaust_pwm, humidifier, led_brig, led_rgb, irrigation
 *
 * Funcionalidades:
 *   - Recebe comandos CFG do mesh_gateway (via Blynk) para alterar estado dos atuadores
 *   - Envia mensagens STATE periodicamente reportando o estado atual dos atuadores
 *   - Envia HELLO na inicialização (QoS1) para se apresentar ao mesh_gateway
 *   - Envia HB (heartbeat) periodicamente
 *   - Responde ACKs automaticamente para mensagens QoS1
 *
 * Toda comunicação é feita via rede mesh (painlessMesh + mesh_proto).
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>

#include "credentials.h"   // MESH_PREFIX, MESH_PASSWORD, MESH_PORT
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_ACT        "act-00"      // este nó (atuadores)
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

// -----------------------------------------------------------------------------
// Objetos da mesh
// -----------------------------------------------------------------------------
Scheduler       userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Estado simulado dos ATUADORES
// -----------------------------------------------------------------------------
static int   g_intake_pwm   = 0;       // PWM ventilador entrada (0-255)
static int   g_exhaust_pwm  = 0;       // PWM ventilador exaustão (0-255)
static int   g_humidifier   = 0;       // Estado umidificador (0=OFF, 1=ON)
static int   g_led_brig     = 0;       // Brilho LED (0-255)
static char  g_led_rgb[16]  = "#000000"; // Cor LED em formato #RRGGBB
static int   g_irrigation   = 0;       // Estado irrigação (0=OFF, 1=ON)

// Modo de operação (0=MANUAL, 1=AUTO) - pode ser expandido no futuro
static int   g_mode         = 0;

// -----------------------------------------------------------------------------
// Controle de envio / contadores
// -----------------------------------------------------------------------------
static uint16_t      g_msg_counter   = 0;
static unsigned long g_last_state    = 0;
static unsigned long g_last_hb       = 0;

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

static int clampi(int v, int vmin, int vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

/**
 * Callback usado pela lib mesh_proto para enviar JSONs via rede mesh.
 */
static void mesh_send_json_cb(const char *json)
{
    mesh.sendBroadcast(json);
    Serial.print("[MESH TX] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// Aplicação de comandos CFG nos atuadores
// -----------------------------------------------------------------------------

static void apply_cfg(const mesh_msg_t *msg)
{
    if (!msg || msg->type != MESH_MSG_CFG) {
        return;
    }

    bool changed = false;

    // Modo de operação
    if (msg->cfg.has_mode) {
        g_mode = msg->cfg.mode;
        Serial.printf("[CFG] mode=%d\n", g_mode);
        changed = true;
    }

    // Intake PWM
    if (msg->cfg.has_intake_pwm) {
        g_intake_pwm = clampi(msg->cfg.intake_pwm, 0, 255);
        Serial.printf("[CFG] intake_pwm=%d\n", g_intake_pwm);
        changed = true;
    }

    // Exhaust PWM
    if (msg->cfg.has_exhaust_pwm) {
        g_exhaust_pwm = clampi(msg->cfg.exhaust_pwm, 0, 255);
        Serial.printf("[CFG] exhaust_pwm=%d\n", g_exhaust_pwm);
        changed = true;
    }

    // Humidifier
    if (msg->cfg.has_humidifier) {
        g_humidifier = clampi(msg->cfg.humidifier, 0, 1);
        Serial.printf("[CFG] humidifier=%d\n", g_humidifier);
        changed = true;
    }

    // Irrigation
    if (msg->cfg.has_irrigation) {
        g_irrigation = clampi(msg->cfg.irrigation, 0, 1);
        Serial.printf("[CFG] irrigation=%d\n", g_irrigation);
        changed = true;
    }

    // LED PWM (brilho)
    if (msg->cfg.has_led_pwm) {
        g_led_brig = clampi(msg->cfg.led_pwm, 0, 255);
        Serial.printf("[CFG] led_brig=%d\n", g_led_brig);
        changed = true;
    }

    // LED RGB (cor)
    if (msg->cfg.has_led_rgb) {
        strncpy(g_led_rgb, msg->cfg.led_rgb, sizeof(g_led_rgb) - 1);
        g_led_rgb[sizeof(g_led_rgb) - 1] = '\0';
        Serial.printf("[CFG] led_rgb=%s\n", g_led_rgb);
        changed = true;
    }

    if (changed) {
        Serial.println("[CFG] Estado dos atuadores atualizado");
    }
}

// -----------------------------------------------------------------------------
// Envio de STATE / HB / HELLO
// -----------------------------------------------------------------------------

/**
 * Envia mensagem STATE com o estado atual dos atuadores.
 * Chamado a cada 30 s.
 */
static void send_state()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_state_act(id,
                                    (uint32_t)millis(),
                                    0,              // QoS0 para STATE periódico
                                    NODE_ACT,
                                    NODE_BLYNK_GW,
                                    g_intake_pwm,
                                    g_exhaust_pwm,
                                    g_humidifier,
                                    g_led_brig,
                                    g_led_rgb,
                                    g_irrigation,
                                    json,
                                    sizeof(json))) {
        return;
    }

    mesh_send_json_cb(json);
}

/**
 * Envia heartbeat do nó de atuadores.
 */
static void send_hb()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    int rssi_dbm = -65 + random(-5, 6); // Simula variação de sinal

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             NODE_ACT,
                             NODE_BLYNK_GW,
                             (int)(millis() / 1000),
                             rssi_dbm,
                             json,
                             sizeof(json))) {
        return;
    }

    mesh_send_json_cb(json);
}

/**
 * Envia HELLO do nó de atuadores (apresentação ao mesh_gateway).
 *   - qos=1
 *   - dst="msh-gw"
 */
static void send_hello()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hello(id,
                                (uint32_t)millis(),
                                1,              // QoS1
                                NODE_ACT,
                                NODE_MSH_GW,
                                NODE_ACT,
                                "1.0.0-c3",
                                "actuators ESP32-C3",
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

    // 1) ACK: SEMPRE avisar o gerenciador de QoS
    if (msg.type == MESH_MSG_ACK) {
        mesh_proto_qos_on_ack(&msg);
        Serial.printf("[RX ACK] ref=%s from=%s dst=%s\n",
                      msg.ack.ref, msg.src, msg.dst);
        return;
    }

    // 2) Filtro de destino lógico
    //    Este nó só processa mensagens endereçadas a:
    //    - act-00 (NODE_ACT)
    //    - msh-gw (NODE_MSH_GW) [ex: TIME]
    //    - broadcast "*"
    if (strcmp(msg.dst, NODE_ACT)    != 0 &&
        strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, "*")         != 0) {
        // Mensagem não é para este nó
        return;
    }

    switch (msg.type) {
    case MESH_MSG_CFG:
        Serial.printf("[RX CFG] from=%s\n", msg.src);
        apply_cfg(&msg);
        
        // Enviar ACK se QoS=1
        if (msg.qos == 1) {
            mesh_proto_qos_send_ack_ok(&msg);
        }
        
        // Enviar STATE imediatamente após CFG para confirmar mudança
        send_state();
        break;

    case MESH_MSG_TIME:
        Serial.println("[TIME] recebido (simulado, não aplicado)");
        
        // Enviar ACK se QoS=1
        if (msg.qos == 1) {
            mesh_proto_qos_send_ack_ok(&msg);
        }
        break;

    case MESH_MSG_HELLO:
        Serial.printf("[RX HELLO] from=%s node_id=%s fw=%s\n", 
                      msg.src,
                      msg.hello.has_node_id ? msg.hello.node_id : "?",
                      msg.hello.has_fw_ver ? msg.hello.fw_ver : "?");
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

    // Só envia HELLO uma vez, na primeira conexão com a rede mesh
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
    Serial.println("[act-00] Actuator Node Boot (ESP32-C3)");
    Serial.println("========================================");

    randomSeed(esp_random());

    // Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    // Inicializa QoS da lib (usado para HELLO QoS1, ACK, etc.)
    mesh_proto_qos_init(mesh_send_json_cb);

    // Estado inicial dos atuadores (tudo desligado)
    g_intake_pwm  = 0;
    g_exhaust_pwm = 0;
    g_humidifier  = 0;
    g_led_brig    = 0;
    strcpy(g_led_rgb, "#000000");
    g_irrigation  = 0;
    g_mode        = 1; //AUTO

    Serial.println("[ACTUATOR NODE] Mesh inicializada");
    Serial.printf("[ACTUATOR NODE] Node ID: %s\n", NODE_ACT);
    Serial.println("[ACTUATOR NODE] Estado inicial: todos atuadores em OFF");
    Serial.println("========================================");
}

void loop()
{
    mesh.update();

    unsigned long now = millis();

    // STATE periódico a cada 30 s
    if (now - g_last_state >= 30000UL) {  // 30 s
        g_last_state = now;
        send_state();
    }

    // Heartbeat a cada 10 s
    if (now - g_last_hb >= 10000UL) {     // 10 s
        g_last_hb = now;
        send_hb();
    }

    // Gerenciador de QoS (retry de mensagens QoS1, ex: HELLO)
    mesh_proto_qos_poll();
}