#ifndef ENVIRONMENT_CONTROLLER_H
#define ENVIRONMENT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#include "mesh_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ENV_MODE_AUTO   = 0,
    ENV_MODE_MANUAL = 1,
} env_mode_t;

/**
 * Callback de envio (o controller monta JSON cfg e chama isso).
 * No gateway, esse callback deve mandar para a mesh (broadcast).
 */
typedef void (*env_send_json_cb_t)(const char *json);

/**
 * Inicializa o controlador.
 */
void env_ctrl_init(env_send_json_cb_t send_cb);

/**
 * Alimenta o controlador com mensagens vindas da MESH (tele/hb/state/cfg...).
 * (Importante: ele detecta HB do act-00 aqui.)
 */
void env_ctrl_on_mesh_msg(const mesh_msg_t *msg, uint32_t now_ms);

/**
 * Alimenta o controlador com mensagens vindas da UART (comandos do usuário).
 * (Importante: ele observa cfg.mode vindo do Blynk para desligar AUTO.)
 */
void env_ctrl_on_uart_msg(const mesh_msg_t *msg, uint32_t now_ms);

/**
 * Tick periódico do controlador (chamar no loop).
 */
void env_ctrl_tick(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* ENVIRONMENT_CONTROLLER_H */
