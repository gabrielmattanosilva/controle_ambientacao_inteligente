/**
 * @file mesh_gateway.h
 * @brief Camada de orquestração da rede mesh e ponte UART <-> mesh.
 *
 * Responsabilidades:
 *  - Inicializar a painlessMesh;
 *  - Encaminhar mensagens entre mesh e blynk_gateway_module (via UART);
 *  - Gerenciar QoS (HELLO/TIME QoS1) usando mesh_proto;
 *  - Enviar HELLO e, após ACK, enviar TIME para o blynk_gateway.
 */

#ifndef MESH_GATEWAY_H
#define MESH_GATEWAY_H

/**
 * @brief Inicializa o gateway da malha.
 *
 * Requisitos ANTES de chamar:
 *  - logger_init_epoch0(), sdcard_begin(), logger_begin() já chamados;
 *  - ds1307_rtc_sync_at_boot() pode ter sido chamado (opcional, mas recomendado);
 *  - ipc_uart_begin() já chamado (UART2 pronta);
 */
void mesh_gateway_init();

/**
 * @brief Loop principal do gateway.
 *
 * Deve ser chamada a cada iteração do loop():
 *  - Atualiza a stack da mesh;
 *  - Lê JSON da UART e encaminha para a mesh quando apropriado;
 *  - Faz polling do gerenciador QoS (retries/timeout).
 */
void mesh_gateway_loop();

#endif /* MESH_GATEWAY_H */
