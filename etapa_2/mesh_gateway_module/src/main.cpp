/**
 * @file main.cpp
 * @brief
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>

#include "credentials.h"
#include "pins.h"
#include "ipc_uart.h"
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

Scheduler userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Funções auxiliares
// -----------------------------------------------------------------------------

/**
 * Envia um JSON recebido da mesh para o blynk_gateway via UART.
 */
static void forward_mesh_to_uart(const char *json)
{
    // Aqui não interpretamos QoS/ACK, só repassamos o quadro puro.
    ipc_uart_send_json(json);
    Serial.print("[MESH->UART] ");
    Serial.println(json);
}

/**
 * Envia um JSON recebido do blynk_gateway (UART) para a mesh.
 */
static void forward_uart_to_mesh(const char *json)
{
    mesh.sendBroadcast(json);
    Serial.print("[UART->MESH] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// Handlers de mensagens
// -----------------------------------------------------------------------------

/**
 * Trata mensagens recebidas da UART (vindas do blynk_gateway_module).
 * - Faz o parse com mesh_proto_parse para poder filtrar por dst.
 * - Repassa para a mesh apenas o que estiver endereçado a NODE_MSH_GW ou "*".
 */
static void handle_uart_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[UART RX] parse fail: ");
        Serial.println(json);
        return;
    }

    // Só processa mensagens destinadas ao gateway mesh (ou broadcast)
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, "*") != 0) {
        Serial.print("[UART RX] ignorado dst=");
        Serial.println(msg.dst);
        return;
    }

    // A ideia aqui é que QUALQUER tipo recebido do Blynk (cfg, time, etc.)
    // seja repassado para a rede mesh, para que os nós (int-sen-00, act-00, ...)
    // decidam o que fazer.
    switch (msg.type) {
    case MESH_MSG_CFG:
    case MESH_MSG_TIME:
    case MESH_MSG_EVT:
    case MESH_MSG_HELLO:
    case MESH_MSG_STATE:
    case MESH_MSG_TELE:
    case MESH_MSG_HB:
    case MESH_MSG_ACK:
        forward_uart_to_mesh(json);
        break;

    default:
        Serial.print("[UART RX] tipo não tratado: ");
        Serial.println(msg.type);
        break;
    }
}

/**
 * Trata mensagens recebidas da mesh.
 * - Faz parse com mesh_proto_parse.
 * - Repassa para UART tudo que estiver endereçado a NODE_BLYNK_GW ou "*".
 */
static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[MESH RX] parse fail: ");
        Serial.println(json);
        return;
    }

    // Só processa mensagens destinadas ao Blynk (ou broadcast)
    if (strcmp(msg.dst, NODE_BLYNK_GW) != 0 &&
        strcmp(msg.dst, "*") != 0) {
        Serial.print("[MESH RX] ignorado dst=");
        Serial.println(msg.dst);
        return;
    }

    // Aqui o gateway é “burro”: não interpreta semântica, só encaminha.
    // Quem entende TELE/STATE/HB/ACK é o blynk_gateway_module.
    forward_mesh_to_uart(json);
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
    Serial.println("[mesh_gateway_module] boot");

    // Inicializa link UART (para blynk_gateway_module)
    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);

    // Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    Serial.println("[mesh_gateway_module] mesh inicializada");
}

void loop()
{
    // Atualiza a stack da mesh
    mesh.update();

    // Verifica se chegou algo da UART
    char json[256];
    if (ipc_uart_read_json(json, sizeof(json))) {
        Serial.print("[UART RX RAW] ");
        Serial.println(json);
        handle_uart_json(json);
    }

    // Este módulo não usa QoS diretamente (apenas Blynk e int-sen-00).
    // Portanto não há necessidade de chamar mesh_proto_qos_poll() aqui.
}
