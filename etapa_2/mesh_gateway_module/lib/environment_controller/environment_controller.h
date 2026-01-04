/**
 * @file environment_controller.h
 * @brief Controle ambiental (AUTO/MANUAL) executado no mesh_gateway_module.
 *
 * Em AUTO:
 *  - Usa TELE de int-sen-00/ext-sen-00;
 *  - Calcula saídas e envia CFG para act-00.
 *
 * Em MANUAL:
 *  - Não envia nada automaticamente (algoritmo “desligado”).
 *  - Usuário controla atuadores via CFG diretamente para act-00 (gateway só repassa).
 */
#ifndef ENVIRONMENT_CONTROLLER_H
#define ENVIRONMENT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "mesh_proto.h"

typedef void (*env_send_cb_t)(const char *json);

typedef enum {
    ENV_MODE_AUTO = 0,
    ENV_MODE_MANUAL = 1,
} env_mode_t;

void env_ctrl_init(env_send_cb_t send_cb);

/* Modo (recebido do usuário via cfg dst="msh-gw") */
void env_ctrl_set_mode(env_mode_t mode);
env_mode_t env_ctrl_get_mode(void);

/* Alimenta telemetria */
void env_ctrl_on_mesh_msg(const mesh_msg_t *msg);

/* Tick periódico (chame no loop) */
void env_ctrl_tick(uint32_t now_ms);

#endif /* ENVIRONMENT_CONTROLLER_H */
