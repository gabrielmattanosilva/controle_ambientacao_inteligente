/**
 * @file blynk_client.h
 * @brief Camada de integração com o Blynk (WiFi, callbacks, Vpins, etc.).
 */

#ifndef BLYNK_CLIENT_H
#define BLYNK_CLIENT_H

#include <Arduino.h>
#include "mesh_proto.h"

/**
 * Inicializa WiFi + Blynk e timers internos.
 * Deve ser chamada no setup() do módulo gateway.
 */
void blynk_client_init();

/**
 * Deve ser chamada em loop(), cuida de Blynk.run() e timers.
 */
void blynk_client_loop();

/**
 * Processa uma mensagem da malha já parseada (mesh_msg_t) e
 * atualiza o painel Blynk/converte em CFG para a malha.
 *
 * - Aplica regra de aguardar HELLO do mesh_gateway antes de TELE/STATE/HB/EVT.
 * - Trata TELE, STATE, HB, EVT, HELLO, TIME.
 */
void blynk_client_handle_mesh_msg(const mesh_msg_t &msg);

#endif /* BLYNK_CLIENT_H */
