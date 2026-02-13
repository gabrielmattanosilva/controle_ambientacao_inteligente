/**
 * @file mesh_gateway.cpp
 * @brief Implementação do gateway entre rede mesh e UART (blynk_gateway_module).
 */

#include "mesh_gateway.h"

#include <Arduino.h>
#include <painlessMesh.h>
#include <time.h>

#include "credentials.h"
#include "pins.h"
#include "ipc_uart.h"
#include "mesh_proto.h"
#include "logger.h"
#include "environment_controller.h"

static const char *TAG = "MESH_GW";

// Identificadores lógicos de nós
#define NODE_INT        "int-sen-00"
#define NODE_EXT        "ext-sen-00"
#define NODE_ACT        "act-00"
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

// -----------------------------------------------------------------------------
// Objetos globais da mesh
// -----------------------------------------------------------------------------
static Scheduler    g_userScheduler;
static painlessMesh g_mesh;

// Contadores / estado interno
static uint16_t g_msg_counter = 0;
static char     g_last_hello_id[8] = {0};

// -----------------------------------------------------------------------------
// Helpers internos
// -----------------------------------------------------------------------------

/**
 * @brief Gera um ID de mensagem simples em formato hexadecimal de 4 dígitos.
 */
static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", (unsigned)g_msg_counter);
}

/**
 * @brief Envia ACK "ok" para uma mensagem QoS1 recebida pela MESH
 *        e endereçada ao mesh-gw.
 */
static void send_ack_to_mesh_if_needed(const mesh_msg_t &req)
{
    // Só responde ACK para mensagens QoS1 (e que não sejam ACK)
    if (req.qos != 1 || req.type == MESH_MSG_ACK) {
        return;
    }

    // E apenas se o destino lógico for o mesh-gw (ou broadcast "*")
    if (strcmp(req.dst, NODE_MSH_GW) != 0 &&
        strcmp(req.dst, "*") != 0) {
        return;
    }

    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_ack(id,
                              (uint32_t)millis(),
                              NODE_MSH_GW,   // src = mesh gateway
                              req.src,       // dst = nó de origem
                              req.id,        // ref = id da msg original
                              "ok",
                              json,
                              sizeof(json))) {
        LOG("MESH", "Falha ao montar ACK para nó mesh");
        return;
    }

    // Envia ACK pela própria mesh
    g_mesh.sendBroadcast(json);
    LOG("MESH", "ACK->MESH: %s", json);
}

/**
 * @brief Envia um JSON recebido da mesh para o blynk_gateway via UART.
 */
static void forward_mesh_to_uart(const char *json)
{
    ipc_uart_send_json(json);
    LOG("FRWD", "MESH->UART: %s", json);
}

/**
 * @brief Envia um JSON recebido do blynk_gateway (UART) para a mesh.
 */
static void forward_uart_to_mesh(const char *json)
{
    g_mesh.sendBroadcast(json);
    LOG("FRWD", "UART->MESH: %s", json);
}

// >>> ADICIONADO: callback de envio para o environment_controller
static void env_send_to_mesh(const char *json)
{
    // O controller monta JSON "cfg" (ex.: dst="act-00")
    // Aqui enviamos na mesh (broadcast).
    g_mesh.sendBroadcast(json);
    LOG("ENV", "AUTO->MESH: %s", json);
}
// <<<

/**
 * @brief Envia HELLO QoS1 para o blynk_gateway_module via UART.
 */
static void send_hello_to_blynk_gw()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    // guarda o ID para detectar o ACK correspondente
    strncpy(g_last_hello_id, id, sizeof(g_last_hello_id) - 1);
    g_last_hello_id[sizeof(g_last_hello_id) - 1] = '\0';

    if (!mesh_proto_build_hello(id,
                                (uint32_t)millis(),
                                1,               // QoS1
                                NODE_MSH_GW,
                                NODE_BLYNK_GW,
                                NODE_MSH_GW,
                                "1.0.0",
                                "mesh-gw boot",
                                json,
                                sizeof(json))) {
        LOG(TAG, "Falha ao montar HELLO para blynk-gw");
        return;
    }

    LOG(TAG, "Enviando HELLO QoS1 -> blynk-gw: %s", json);
    mesh_proto_qos_register_and_send(id, json);
}

/**
 * @brief Envia TIME QoS1 para o blynk_gateway_module com base no RTC interno.
 */
static void send_time_to_blynk_gw()
{
    time_t now = time(nullptr);
    const int TZ_OFFSET_MIN = -3 * 60;

    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_time(id,
                               (uint32_t)millis(),
                               1,               // QoS1
                               NODE_MSH_GW,
                               NODE_BLYNK_GW,
                               (uint32_t)now,
                               TZ_OFFSET_MIN,
                               json,
                               sizeof(json))) {
        LOG(TAG, "Falha ao montar TIME para blynk-gw");
        return;
    }

    LOG(TAG, "Enviando TIME QoS1 -> blynk-gw: %s", json);
    mesh_proto_qos_register_and_send(id, json);
}

static bool is_known_dst(const char *dst)
{
    return
        strcmp(dst, NODE_MSH_GW)   == 0 ||
        strcmp(dst, NODE_BLYNK_GW) == 0 ||
        strcmp(dst, NODE_INT)      == 0 ||
        strcmp(dst, NODE_EXT)      == 0 ||
        strcmp(dst, NODE_ACT)      == 0 ||
        strcmp(dst, "*")           == 0;
}


// -----------------------------------------------------------------------------
// Handlers de mensagens
// -----------------------------------------------------------------------------

static void handle_uart_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg))
    {
        LOG("UART", "RX parse FAIL: %s", json ? json : "(null)");
        return;
    }

    // ACK para o próprio mesh_gateway (msh-gw) => QoS HELLO/TIME
    if (msg.type == MESH_MSG_ACK && strcmp(msg.dst, NODE_MSH_GW) == 0)
    {
        mesh_proto_qos_on_ack(&msg);

        if (msg.ack.has_ref &&
            strcmp(msg.ack.ref, g_last_hello_id) == 0)
        {
            LOG("UART", "ACK do HELLO recebido, enviando TIME QoS1");
            send_time_to_blynk_gw();
        }
        return;
    }

    if (!is_known_dst(msg.dst))
    {
        LOG("UART", "RX ignorado dst=\"%s\"", msg.dst);
        return;
    }

    // Encaminha “burro” para a mesh (inclui comandos manuais pro act-00)
    forward_uart_to_mesh(json);
}

static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg))
    {
        LOG("MESH", "RX parse FAIL: %s", json ? json : "(null)");
        return;
    }

    // 1) ACK para mensagens QoS1 destinadas ao mesh-gw
    send_ack_to_mesh_if_needed(msg);

    // 2) Só encaminha para UART o que for para o blynk-gw (ou broadcast)
    if (strcmp(msg.dst, NODE_BLYNK_GW) != 0 &&
        strcmp(msg.dst, "*") != 0)
    {
        LOG("MESH", "RX nao encaminhado para UART dst=\"%s\"", msg.dst);
        return;
    }

    forward_mesh_to_uart(json);
}

// -----------------------------------------------------------------------------
// Callbacks da painlessMesh
// -----------------------------------------------------------------------------

static void mesh_receive_cb(uint32_t from, String &msg)
{
    LOG("MESH", "RX from %u: %s", (unsigned)from, msg.c_str());
    handle_mesh_json(msg.c_str());
}

static void mesh_new_connection_cb(uint32_t nodeId)
{
    LOG("MESH", "New connection: %u", (unsigned)nodeId);
}

static void mesh_changed_connections_cb()
{
    LOG("MESH", "Changed connections");
}

static void mesh_time_adjusted_cb(int32_t offset)
{
    LOG("MESH", "Time adjusted offset=%ld", (long)offset);
}

// -----------------------------------------------------------------------------
// API pública
// -----------------------------------------------------------------------------

void mesh_gateway_init()
{
    g_msg_counter = 0;
    g_last_hello_id[0] = '\0';

    LOG(TAG, "Inicializando mesh_gateway...");

    // QoS via UART
    mesh_proto_qos_init(ipc_uart_send_json);

    // Envia HELLO QoS1 para o blynk_gateway
    send_hello_to_blynk_gw();

    // Inicializa rede mesh
    g_mesh.setDebugMsgTypes(ERROR | STARTUP);
    g_mesh.init(MESH_PREFIX, MESH_PASSWORD, &g_userScheduler, MESH_PORT);
    g_mesh.onReceive(&mesh_receive_cb);
    g_mesh.onNewConnection(&mesh_new_connection_cb);
    g_mesh.onChangedConnections(&mesh_changed_connections_cb);
    g_mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    LOG(TAG, "mesh inicializada (prefix=%s, port=%d)", MESH_PREFIX, MESH_PORT);
}

void mesh_gateway_loop()
{
    g_mesh.update();

    // UART RX (blynk_gateway_module)
    char json[256];
    if (ipc_uart_read_json(json, sizeof(json)))
    {
        LOG("UART", "RX RAW: %s", json);
        handle_uart_json(json);
    }

    // QoS HELLO/TIME sobre UART
    mesh_proto_qos_poll();
}
