/**
 * @file main.cpp
 * @brief Gateway entre rede mesh e blynk_gateway_module (UART),
 *        com logging em SD e sincronização de tempo via DS1307.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <time.h>

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

static uint16_t g_msg_counter = 0;
static char     g_last_hello_id[8] = {0};

/**
 * Gera um ID de mensagem simples em formato hexadecimal de 4 dígitos.
 */
static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", (unsigned)g_msg_counter);
}

/**
 * Envia HELLO QoS1 para o blynk_gateway_module via UART.
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
 * Envia TIME QoS1 para o blynk_gateway_module com base no RTC interno
 * (já sincronizado pelo DS1307 na inicialização).
 */
static void send_time_to_blynk_gw()
{
    time_t now = time(nullptr);
    // Ajuste se quiser outro fuso; aqui usamos -3h (BRT) como exemplo.
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
 * - Trata ACK vindo do blynk_gateway para gerenciar QoS (HELLO/TIME).
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

    // Mensagens ACK referem-se tipicamente a envios QoS1 (HELLO/TIME, etc.)
    if (msg.type == MESH_MSG_ACK)
    {
        // Atualiza gerenciador de QoS interno (HELLO/TIME)
        mesh_proto_qos_on_ack(&msg);

        // Se este ACK for do nosso HELLO, dispara o envio de TIME QoS1
        if (msg.ack.has_ref &&
            strcmp(msg.ack.ref, g_last_hello_id) == 0)
        {
            LOG("UART", "ACK do HELLO recebido, enviando TIME QoS1");
            send_time_to_blynk_gw();
        }
        return; // ACK nao e encaminhado para a mesh
    }

    // Demais mensagens (CFG, EVT etc.) sao repassadas para a mesh
    forward_uart_to_mesh(json);
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
    LOG("MESH", "RX from %u: %s", (unsigned)from,
        msg.c_str());
    handle_mesh_json(msg.c_str());
}

void mesh_new_connection_cb(uint32_t nodeId)
{
    LOG("MESH", "New connection: %u", (unsigned)nodeId);
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
    // Gerenciador de QoS da lib usando o mesmo transporte UART
    mesh_proto_qos_init(ipc_uart_send_json);

    // Envia HELLO QoS1 para o blynk_gateway assim que UART/QoS estiverem prontos
    send_hello_to_blynk_gw();

    // 4) Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    LOG(TAG, "mesh inicializada (prefix=%s, port=%d)", MESH_PREFIX, MESH_PORT);
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

    // Gerenciador de QoS da lib (HELLO/TIME QoS1 sobre UART)
    mesh_proto_qos_poll();
}
