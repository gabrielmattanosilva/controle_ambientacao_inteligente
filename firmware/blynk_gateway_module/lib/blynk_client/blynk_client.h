/**
 * @file blynk_client.h
 * @brief Camada de integração com o Blynk (WiFi, callbacks, Vpins, etc.).
 *
 * Este módulo faz a ponte entre:
 * - A malha mesh (via mesh_proto / mesh_gateway);
 * - O painel Blynk (datastreams virtuais, callbacks, etc.);
 * - A conectividade WiFi do ESP32.
 *
 * Ele recebe TELE/STATE/HB/EVT/HELLO/TIME da malha e:
 * - Atualiza os Vpins correspondentes no painel Blynk;
 * - Converte comandos vindos do Blynk em mensagens CFG para a malha;
 * - Garante que apenas mensagens relevantes sejam processadas após o HELLO do mesh_gateway.
 */

#ifndef BLYNK_CLIENT_H
#define BLYNK_CLIENT_H

#include <Arduino.h>
#include "mesh_proto.h"

/**
 * @brief Inicializa WiFi, Blynk e timers internos do cliente Blynk.
 *
 * Esta função deve ser chamada no @c setup() do módulo gateway e realiza:
 * - Conexão à rede WiFi usando credenciais definidas em @c credentials.h;
 * - Configuração do cliente Blynk com o token de autenticação;
 * - Estabelecimento da conexão com o servidor Blynk;
 * - Configuração de um heartbeat simples do próprio nó em um Vpin.
 */
void blynk_client_init();

/**
 * @brief Loop principal do cliente Blynk.
 *
 * Deve ser chamada periodicamente dentro da função @c loop() do firmware.
 * Internamente:
 * - Chama @c Blynk.run();
 * - Executa @c g_timer.run() para gerenciar callbacks temporizados.
 */
void blynk_client_loop();

/**
 * @brief Processa uma mensagem da malha já parseada e reflete no Blynk.
 *
 * Esta função recebe uma @ref mesh_msg_t advinda da malha e:
 * - Atualiza Vpins de telemetria, estado de atuadores ou heartbeat;
 * - Converte determinados tipos em ações internas (ex.: TIME → ajuste de RTC);
 * - Aplica a regra de só aceitar TELE/STATE/HB/EVT depois do HELLO do mesh_gateway.
 *
 * Tipos tratados:
 * - @ref MESH_MSG_TELE
 * - @ref MESH_MSG_STATE
 * - @ref MESH_MSG_HB
 * - @ref MESH_MSG_EVT
 * - @ref MESH_MSG_HELLO
 * - @ref MESH_MSG_TIME
 *
 * @param msg Mensagem já parseada pelo @ref mesh_proto_parse().
 */
void blynk_client_handle_mesh_msg(const mesh_msg_t &msg);

#endif /* BLYNK_CLIENT_H */
