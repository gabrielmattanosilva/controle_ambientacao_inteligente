/**
 * @file main.cpp
 * @brief Nó simulador SOMENTE do act-00 (atuadores) - com RX silencioso p/ dst não relevante
 *
 * Este firmware roda em um ESP32 e simula apenas 1 nó lógico:
 *   - "act-00" → atuadores (intake_pwm, exhaust_pwm, humidifier, led_brig/led_rgb, irrigation)
 *
 * Comunicação via painlessMesh + mesh_proto, conversando com o mesh_gateway_module,
 * que faz a ponte até o Blynk.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>

#include "credentials.h"   // MESH_PREFIX, MESH_PASSWORD, MESH_PORT
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_ACT        "act-00"
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

// neste simulador, tudo vai para o blynk-gw
#define NODE_SINK       NODE_BLYNK_GW

// -----------------------------------------------------------------------------
// Objetos da mesh
// -----------------------------------------------------------------------------
Scheduler       userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Estado simulado dos atuadores (act-00)
// -----------------------------------------------------------------------------
static int  g_mode        = 0;        // 0=AUTO, 1=MANUAL
static int  g_intake_pwm  = 0;
static int  g_exhaust_pwm = 0;
static int  g_humidifier  = 0;
static int  g_irrigation  = 0;
static int  g_led_pwm     = 0;        // usado como "led_brig"
static char g_led_rgb[16] = "0x000000";

// -----------------------------------------------------------------------------
// Controle de envio / contadores
// -----------------------------------------------------------------------------
static uint16_t      g_msg_counter = 0;
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

/**
 * Callback usado pela lib mesh_proto para enviar JSONs QoS1 (HELLO, ACK, etc.)
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
// Envio de STATE / HB / HELLO
// -----------------------------------------------------------------------------

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
                                    sizeof(json)))
    {
        return;
    }

    mesh_send_json_cb(json);
}

/**
 * Envia heartbeat do act-00.
 */
static void send_hb()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    int rssi_dbm = -60 + random(-3, 4);

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             NODE_ACT,
                             NODE_SINK,
                             (int)(millis() / 1000),
                             rssi_dbm,
                             json,
                             sizeof(json)))
    {
        return;
    }

    mesh_send_json_cb(json);
}

/**
 * Envia HELLO do act-00 (apresentação ao mesh_gateway).
 *   - qos=1
 *   - dst="msh-gw"
 *   - vai pelo gerenciador de QoS (mesh_proto_qos_register_and_send)
 */
static void send_hello()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hello(id,
                                (uint32_t)millis(),
                                1,              // QoS1
                                NODE_ACT,       // src lógico
                                NODE_MSH_GW,    // dst = msh-gw
                                NODE_ACT,
                                "1.0.0-sim",
                                "actuators",
                                json,
                                sizeof(json)))
    {
        return;
    }

    // QoS1 com retry automático
    if (!mesh_proto_qos_register_and_send(id, json))
    {
        // fallback se a fila de QoS estiver cheia
        mesh_send_json_cb(json);
    }
}

// -----------------------------------------------------------------------------
// Aplicar CFG recebido (comportamento do act-00)
// -----------------------------------------------------------------------------

/**
 * Aplica CFG recebido do Blynk (via mesh_gateway):
 *
 * - mode (AUTO/MANUAL):
 *   * mode = 0 (AUTO): comandos de atuadores são ignorados.
 *   * mode = 1 (MANUAL): comandos de atuadores são aplicados.
 *
 * - Após aplicar (ou ignorar) os comandos, envia STATE imediato
 *   e responde ACK "ok" se o CFG for QoS1.
 */
static void apply_cfg(const mesh_msg_t &msg)
{
    // 1) Atualiza modo primeiro, se vier no CFG
    if (msg.cfg.has_mode)
    {
        g_mode = msg.cfg.mode;
    }

    // 2) Só aplica comandos de atuador se estivermos em modo MANUAL (1)
    if (g_mode == 1)
    {
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
    }
    else
    {
        // Em AUTO ignoramos comandos de atuador (mantém estado atual)
        // (sem log para não poluir serial)
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
    if (!mesh_proto_parse(json, &msg))
    {
        Serial.print("[RX] parse fail: ");
        Serial.println(json ? json : "(null)");
        return;
    }

    // 1) ACK: SEMPRE avisar o gerenciador de QoS, independente do dst lógico
    if (msg.type == MESH_MSG_ACK)
    {
        mesh_proto_qos_on_ack(&msg);
        Serial.printf("[RX ACK] ref=%s from=%s dst=%s\n",
                      msg.ack.ref, msg.src, msg.dst);
        return;
    }

    // 2) Demais tipos: aplica filtro de destino lógico (agora silencioso)
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, NODE_ACT)    != 0 &&
        strcmp(msg.dst, "*")         != 0)
    {
        // Silencia tráfego que não é pra este nó lógico
        return;
    }

    switch (msg.type)
    {
    case MESH_MSG_CFG:
        apply_cfg(msg);
        break;

    case MESH_MSG_TIME:
        Serial.println("[TIME] recebido (simulado, não aplicado)");
        break;

    default:
        // Tipos não usados pelo act-00 podem ser ignorados
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

    // Envia HB imediatamente ao conectar (e evita mandar de novo logo em seguida)
    unsigned long now = millis();
    if (now - g_last_hb > 2000UL) {   // throttle simples (2s)
        g_last_hb = now;
        send_hb();
    }

    // Só envia HELLO uma vez, na primeira conexão com a rede mesh
    if (!g_hello_sent)
    {
        g_hello_sent = true;
        Serial.println("[MESH] Primeira conexao estabelecida, enviando HELLO QoS1 do act-00...");
        send_hello();

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
    Serial.println("[act-00-sim] boot");

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

    Serial.println("[act-00-sim] mesh inicializada");
}

void loop()
{
    mesh.update();

    unsigned long now = millis();

    // Heartbeat do act-00 a cada 10 s
    if (now - g_last_hb >= 10000UL)
    {
        g_last_hb = now;
        send_hb();
    }

    // Gerenciador de QoS (retry de mensagens QoS1, ex: HELLO)
    mesh_proto_qos_poll();
}
