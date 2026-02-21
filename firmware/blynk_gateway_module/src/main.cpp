/**
 * @file main.cpp
 * @brief Módulo Blynk Gateway — orquestração ipc_uart + mesh_proto + blynk_client.
 *
 * Este firmware integra três camadas principais:
 *
 * - **ipc_uart**: comunicação física com o mesh_gateway_module, recebendo/enviando
 *   mensagens JSON encapsuladas em COBS + CRC16 (via ipc_uart).
 *
 * - **mesh_proto**: parser de mensagens, roteamento por destino (dst), QoS1 com ACK
 *   automático e gerenciador de pendências.
 *
 * - **blynk_client**: interface com o painel Blynk (telemetria, estado, comandos),
 *   conversão de mensagens da malha para Vpins e conversão de Vpins em mensagens CFG.
 *
 * O módulo funciona como "ponte" entre o app Blynk e a rede mesh.
 */

#include <Arduino.h>
#include "pins.h"
#include "ipc_uart.h"
#include "mesh_proto.h"
#include "blynk_client.h"

#define NODE_BLYNK_GW "blynk-gw"

/**
 * @brief Processa uma mensagem JSON vinda da UART (malha) e repassa ao Blynk.
 *
 * Fluxo:
 * 1. Faz o parse do JSON usando @ref mesh_proto_parse().
 * 2. Ignora mensagens cujo `dst` não seja "blynk-gw" ou "*".
 * 3. Se a mensagem for QoS1 (exceto ACK), envia ACK automático.
 * 4. Se for um ACK, informa o gerenciador de QoS para remover pendências.
 * 5. Para demais tipos (TELE, STATE, HB, EVT, HELLO, TIME), delega ao
 *    @ref blynk_client_handle_mesh_msg().
 *
 * @param json String JSON completa recebida da UART.
 */
static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg))
    {
        Serial.print("[MESH] JSON parse fail: ");
        Serial.println(json);
        return;
    }

    /* Só processa mensagens destinadas ao blynk-gw ou broadcast "*" */
    if (strcmp(msg.dst, NODE_BLYNK_GW) != 0 &&
        strcmp(msg.dst, "*") != 0)
    {
        return;
    }

    /* Se for QoS1 e não for ACK → envia ACK automático */
    if (msg.qos == 1 && msg.type != MESH_MSG_ACK)
    {
        mesh_proto_qos_send_ack_ok(&msg);
    }

    /* ACK recebido → remove pendência de QoS */
    if (msg.type == MESH_MSG_ACK)
    {
        mesh_proto_qos_on_ack(&msg);
        return;
    }

    /* Delegamos tratamento */
    blynk_client_handle_mesh_msg(msg);
}

/**
 * @brief Inicialização do módulo Blynk Gateway.
 *
 * Etapas:
 * - Inicia porta Serial (debug).
 * - Configura UART2 para o link IPC com o mesh_gateway_module.
 * - Inicializa o QoS da lib mesh_proto, usando UART como transporte.
 * - Inicializa WiFi, Blynk e timers via @ref blynk_client_init().
 */
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("[blynk_gateway_module] boot");

    /* ipc_uart (link com mesh_gateway_module) */
    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);

    /* QoS da lib mesh_proto usa o mesmo transporte */
    mesh_proto_qos_init(ipc_uart_send_json);

    /* WiFi + Blynk + timers (abstraído na lib blynk_client) */
    blynk_client_init();
}

/**
 * @brief Loop principal do módulo.
 *
 * A cada iteração:
 * - Executa @ref blynk_client_loop() para Blynk e timers.
 * - Lê JSONs da UART usando @ref ipc_uart_read_json().
 * - Caso um JSON completo seja recebido, imprime no Serial e
 *   encaminha ao @ref handle_mesh_json().
 * - Executa o gerenciador de QoS da lib mesh_proto com @ref mesh_proto_qos_poll().
 */
void loop()
{
    /* Cuida do Blynk e timers internos */
    blynk_client_loop();

    /* Recebe JSON da UART e encaminha */
    char json[256];
    if (ipc_uart_read_json(json, sizeof(json)))
    {
        Serial.print("[RX JSON] ");
        Serial.println(json);
        handle_mesh_json(json);
    }

    /* Gerenciador de QoS da lib mesh_proto */
    mesh_proto_qos_poll();
}
