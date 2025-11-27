/**
 * @file main.cpp
 * @brief Blynk Gateway Module (orquestração IPC + mesh + Blynk client)
 */

#include <Arduino.h>
#include "pins.h"
#include "ipc_uart.h"
#include "mesh_proto.h"
#include "blynk_client.h"

#define NODE_BLYNK_GW   "blynk-gw"

/* ------------------------------------------------------------
 * Roteia JSON vindo da UART para o blynk_client,
 * fazendo o filtro por dst e tratamento de QoS (ACK).
 * ------------------------------------------------------------ */

static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[MESH] JSON parse fail: ");
        Serial.println(json);
        return;
    }

    // Só processa mensagens destinadas ao blynk-gw ou broadcast "*"
    if (strcmp(msg.dst, NODE_BLYNK_GW) != 0 &&
        strcmp(msg.dst, "*") != 0) {
        return;
    }

    // Responde ACK automático para qualquer mensagem QoS1 recebida
    if (msg.qos == 1 && msg.type != MESH_MSG_ACK) {
        mesh_proto_qos_send_ack_ok(&msg);
    }

    // ACK recebido limpa pendências de QoS deste próprio módulo
    if (msg.type == MESH_MSG_ACK) {
        mesh_proto_qos_on_ack(&msg);
        return;
    }

    // Demais tipos delegamos para a lib Blynk (TELE, STATE, HB, EVT, HELLO, TIME)
    blynk_client_handle_mesh_msg(msg);
}

/* ------------------------------------------------------------
 * setup / loop
 * ------------------------------------------------------------ */

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println("[blynk_gateway_module] boot");

    // IPC UART (link com mesh_gateway_module)
    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);

    // QoS da lib mesh_proto usa o mesmo transporte
    mesh_proto_qos_init(ipc_uart_send_json);

    // WiFi + Blynk + timers (abstraído na lib blynk_client)
    blynk_client_init();
}

void loop()
{
    // Cuida do Blynk e timers internos
    blynk_client_loop();

    // Recebe JSON da UART e encaminha
    char json[256];
    if (ipc_uart_read_json(json, sizeof(json))) {
        Serial.print("[RX JSON] ");
        Serial.println(json);
        handle_mesh_json(json);
    }

    // Gerenciador de QoS da lib mesh_proto
    mesh_proto_qos_poll();
}
