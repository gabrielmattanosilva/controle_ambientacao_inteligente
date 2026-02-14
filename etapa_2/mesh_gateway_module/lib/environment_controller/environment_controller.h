/**
 * @file environment_controller.h
 * @brief Controle automático simples do ambiente (setpoints hardcoded).
 *
 * Esta lib roda no mesh_gateway_module (msh-gw) e decide comandos "cfg" para o nó act-00.
 *
 * Requisitos atendidos:
 *  - Modo AUTO/MANUAL (V13 no Blynk continua comandando "mode" via cfg);
 *  - Controle simples de temperatura (intake/exhaust) e luminosidade (led_pwm);
 *  - Irrigação e umidificação diárias com tempo máximo por dia (budget);
 *  - Proteção de duty-cycle:
 *      - bomba d'água: 30s ON / 30s OFF (nunca > 30s contínuos)
 *      - umidificador: 30s ON / 60s OFF (nunca > 30s contínuos)
 *
 * Importante:
 *  - Não cria novos campos no Blynk. Setpoints e janelas diárias ficam hardcoded aqui.
 *  - Em MANUAL, os comandos de irrigação/umidificador vindos do Blynk são "engolidos"
 *    para aplicar o duty-cycle de segurança. (Os demais comandos passam direto.)
 */

#ifndef ENVIRONMENT_CONTROLLER_H
#define ENVIRONMENT_CONTROLLER_H

#include <stdbool.h>
#include "mesh_proto.h"

typedef void (*env_ctrl_send_cb_t)(const char *json);

/**
 * @brief Inicializa o controlador.
 * @param send_cb Callback para envio de JSON "cfg" para a mesh (normalmente broadcast).
 */
void environment_controller_init(env_ctrl_send_cb_t send_cb);

/**
 * @brief Alimenta o controlador com mensagens recebidas da MESH (TELE/STATE/etc).
 * @param msg Mensagem já parseada pelo mesh_proto.
 */
void environment_controller_on_mesh_msg(const mesh_msg_t *msg);

/**
 * @brief Processa mensagens recebidas via UART (normalmente cfg do Blynk).
 *
 * Retorno:
 *  - true  => o mesh_gateway deve encaminhar o JSON original para a mesh
 *  - false => o mesh_gateway NÃO deve encaminhar (o controller assumiu o comando)
 *
 * Uso típico:
 *  - sempre encaminhar "mode";
 *  - em modo MANUAL, engolir irrigação/umidificador para aplicar duty-cycle.
 */
bool environment_controller_on_uart_msg(const mesh_msg_t *msg);

/**
 * @brief Função periódica (chame no loop()).
 *
 * Em modo AUTO:
 *  - calcula saídas e envia cfg quando necessário.
 *
 * Em modo MANUAL:
 *  - aplica apenas o duty-cycle de segurança da bomba/umidificador para os pedidos manuais.
 */
void environment_controller_poll(void);

#endif /* ENVIRONMENT_CONTROLLER_H */
