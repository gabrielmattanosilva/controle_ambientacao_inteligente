/**
 * @file main.cpp
 * @brief Gateway entre rede mesh e blynk_gateway_module (UART),
 *        com logging em SD e sincronização de tempo via DS1307.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>

#include "credentials.h"
#include "pins.h"
#include "ipc_uart.h"
#include "mesh_proto.h"
#include "ds1307_rtc.h"
#include "logger.h"
#include "sd_card.h"

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

static Scheduler    userScheduler;
static painlessMesh mesh;

static const char *TAG = "MAIN";

// -----------------------------------------------------------------------------
// Funções auxiliares
// -----------------------------------------------------------------------------

/**
 * Envia um JSON recebido da mesh para o blynk_gateway via UART.
 */
static void forward_mesh_to_uart(const char *json)
{
    ipc_uart_send_json(json);
    LOG("FRWD", "MESH->UART: %s", json);
}

/**
 * Envia um JSON recebido do blynk_gateway (UART) para a mesh.
 */
static void forward_uart_to_mesh(const char *json)
{
    mesh.sendBroadcast(json);
    LOG("FRWD", "UART->MESH: %s", json);
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
    if (!mesh_proto_parse(json, &msg))
    {
        LOG("UART", "RX parse FAIL: %s", json ? json : "(null)");
        return;
    }

    // Só processa mensagens destinadas ao gateway mesh (ou broadcast)
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, "*") != 0)
    {
        LOG("UART", "RX ignorado dst=\"%s\"", msg.dst);
        return;
    }

    // Repassa qualquer tipo vindo do Blynk para a mesh
    switch (msg.type)
    {
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
        LOG("UART", "RX tipo nao tratado: %d", (int)msg.type);
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
    if (!mesh_proto_parse(json, &msg))
    {
        LOG("MESH", "RX parse FAIL: %s", json ? json : "(null)");
        return;
    }

    // Só processa mensagens destinadas ao Blynk (ou broadcast)
    if (strcmp(msg.dst, NODE_BLYNK_GW) != 0 &&
        strcmp(msg.dst, "*") != 0)
    {
        LOG("MESH", "RX ignorado dst=\"%s\"", msg.dst);
        return;
    }

    // Gateway "burro": só encaminha
    forward_mesh_to_uart(json);
}

// -----------------------------------------------------------------------------
// Callbacks da painlessMesh
// -----------------------------------------------------------------------------

void mesh_receive_cb(uint32_t from, String &msg)
{
    LOG("MESH", "RX from %u: %s", (unsigned)from, msg.c_str());
    handle_mesh_json(msg.c_str());
}

void mesh_new_connection_cb(uint32_t nodeId)
{
    LOG("MESH", "New connection, nodeId=%u", (unsigned)nodeId);
}

void mesh_changed_connections_cb()
{
    LOG("MESH", "Changed connections");
}

void mesh_time_adjusted_cb(int32_t offset)
{
    LOG("MESH", "Time adjusted offset=%ld", (long)offset);
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------

void setup()
{
    // 1) Inicializa sistema de logs (RTC interno em epoch0 + SD)
    logger_init_epoch0();
    sdcard_begin();
    logger_begin();

    LOG(TAG, "[mesh_gateway_module] boot");

    // 2) Tenta sincronizar o RTC interno a partir do DS1307
    if (ds1307_rtc_sync_at_boot())
    {
        LOG(TAG, "RTC interno sincronizado a partir do DS1307");
    }
    else
    {
        LOG(TAG, "DS1307 ausente/invalido/parado; mantendo epoch0 ate ter hora valida");
    }

    // 3) Inicializa link UART (para blynk_gateway_module)
    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);
    LOG(TAG, "UART IPC inicializada (TX=%d, RX=%d)", UART_TX_PIN, UART_RX_PIN);

    // 4) Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    LOG(TAG, "mesh inicializada (prefix=\"%s\", port=%d)", MESH_PREFIX, MESH_PORT);
}

void loop()
{
    // Rotação diária do arquivo de log
    sdcard_tick_rotate();

    // Atualiza a stack da mesh
    mesh.update();

    // Verifica se chegou algo da UART
    char json[256];
    if (ipc_uart_read_json(json, sizeof(json)))
    {
        LOG("UART", "RX RAW: %s", json);
        handle_uart_json(json);
    }

    // Este módulo continua sem usar QoS diretamente.
    // Portanto não há necessidade de chamar mesh_proto_qos_poll() aqui.
}
