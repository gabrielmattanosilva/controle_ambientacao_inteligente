#ifndef MESH_PROTO_H
#define MESH_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum {
    MESH_MSG_UNKNOWN = 0,
    MESH_MSG_TELE,
    MESH_MSG_STATE,
    MESH_MSG_CFG,
    MESH_MSG_HB,
    MESH_MSG_EVT,
    MESH_MSG_HELLO,
    MESH_MSG_ACK,
    MESH_MSG_TIME
} mesh_msg_type_t;

typedef struct {
    char id[8];      // "000A"
    uint32_t ts;     // timestamp
    uint8_t  qos;    // 0 / 1
    char src[16];    // "blynk-gw", "msh-gw", "act-00", etc.
    char dst[16];    // "blynk-gw", "msh-gw", "*"
    mesh_msg_type_t type;

    struct {
        bool has_t_out, has_rh_out, has_lux_out;
        float t_out;
        float rh_out;
        int   lux_out;

        bool has_t_in, has_rh_in, has_soil_moist, has_lux_in;
        float t_in;
        float rh_in;
        int   soil_moist;
        int   lux_in;
    } tele;

    struct {
        bool has_intake_pwm, has_exhaust_pwm, has_humidifier;
        bool has_led_brig, has_led_rgb, has_irrigation;

        int  intake_pwm;
        int  exhaust_pwm;
        int  humidifier;
        int  led_brig;
        char led_rgb[16];
        int  irrigation;
    } state;

    struct {
        bool has_mode, has_intake_pwm, has_exhaust_pwm;
        bool has_humidifier, has_irrigation, has_led_pwm, has_led_rgb;

        int  mode;
        int  intake_pwm;
        int  exhaust_pwm;
        int  humidifier;
        int  irrigation;
        int  led_pwm;
        char led_rgb[16];
    } cfg;

    struct {
        bool has_uptime_s, has_rssi_dbm;
        int  uptime_s;
        int  rssi_dbm;
    } hb;

    struct {
        bool has_event, has_code, has_level;
        char event[16];   // nome do evento (ex.: "overheat")
        int  code;        // código numérico opcional
        int  level;       // severidade opcional
    } evt;

    struct {
        bool has_node_id, has_fw_ver, has_extra;
        char node_id[16]; // ex.: "ext-sen-00"
        char fw_ver[16];  // ex.: "1.0.0"
        char extra[32];   // info adicional
    } hello;

    struct {
        bool has_ref, has_status;
        char ref[8];      // id da msg referenciada
        char status[8];   // ex.: "ok", "err"
    } ack;

    struct {
        bool     has_epoch, has_tz_offset_min;
        uint32_t epoch;         // epoch em segundos
        int      tz_offset_min; // offset fuso em minutos
    } time_sync;

} mesh_msg_t;

/* ============================================================
 * PARSE + BUILDERS
 * ============================================================ */

bool mesh_proto_parse(const char *json_str, mesh_msg_t *out_msg);

bool mesh_proto_build_cfg_int(const char *id,
                              uint32_t ts,
                              uint8_t qos,
                              const char *src,
                              const char *dst,
                              const char *field_name,
                              int value,
                              char *out_json,
                              size_t maxlen);

bool mesh_proto_build_cfg_str(const char *id,
                              uint32_t ts,
                              uint8_t qos,
                              const char *src,
                              const char *dst,
                              const char *field_name,
                              const char *value,
                              char *out_json,
                              size_t maxlen);

bool mesh_proto_build_state_act(const char *id,
                                uint32_t ts,
                                uint8_t qos,
                                const char *src,
                                const char *dst,
                                int intake_pwm,
                                int exhaust_pwm,
                                int humidifier,
                                int led_brig,
                                const char *led_rgb,
                                int irrigation,
                                char *out_json,
                                size_t maxlen);

bool mesh_proto_build_hb(const char *id,
                         uint32_t ts,
                         const char *src,
                         const char *dst,
                         int uptime_s,
                         int rssi_dbm,
                         char *out_json,
                         size_t maxlen);

bool mesh_proto_build_ack(const char *id,
                          uint32_t ts,
                          const char *src,
                          const char *dst,
                          const char *ref,
                          const char *status,
                          char *out_json,
                          size_t maxlen);

bool mesh_proto_build_hello(const char *id,
                            uint32_t ts,
                            uint8_t qos,
                            const char *src,
                            const char *dst,
                            const char *node_id,
                            const char *fw_ver,
                            const char *extra,
                            char *out_json,
                            size_t maxlen);

bool mesh_proto_build_evt(const char *id,
                          uint32_t ts,
                          uint8_t qos,
                          const char *src,
                          const char *dst,
                          const char *event,
                          int code,
                          int level,
                          char *out_json,
                          size_t maxlen);

bool mesh_proto_build_time(const char *id,
                           uint32_t ts,
                           uint8_t qos,
                           const char *src,
                           const char *dst,
                           uint32_t epoch,
                           int tz_offset_min,
                           char *out_json,
                           size_t maxlen);

/* ============================================================
 * QoS MANAGER (dentro da lib, reaproveitável)
 * ============================================================ */

/**
 * Callback genérico para envio físico (UART, mesh, etc.).
 * Ex.: mesh_proto_qos_init(ipc_uart_send_json);
 */
typedef void (*mesh_proto_send_cb_t)(const char *json);

/**
 * Inicializa o gerenciador de QoS dentro da lib.
 * - send_cb: ponteiro para função que faz o envio real.
 */
void mesh_proto_qos_init(mesh_proto_send_cb_t send_cb);

/**
 * Registra uma mensagem QoS1 como pendente e a envia.
 * - id   : id da mensagem (doc["id"])
 * - json : payload JSON completo (já montado)
 *
 * Se não houver slot livre, envia mesmo assim, mas sem retry/ACK tracking.
 */
bool mesh_proto_qos_register_and_send(const char *id,
                                      const char *json);

/**
 * Deve ser chamada sempre que chegar um MESH_MSG_ACK parseado.
 * - msg: mensagem ACK já parseada por mesh_proto_parse()
 *
 * A lib compara msg.data.ref com pendências internas e, se bater, limpa.
 */
void mesh_proto_qos_on_ack(const mesh_msg_t *msg);

/**
 * Deve ser chamada periodicamente (no loop de cada módulo que usa QoS).
 * - Faz timeout, retry e marca falha após N tentativas.
 */
void mesh_proto_qos_poll(void);

/**
 * Envia um ACK "ok" para uma mensagem recebida com QoS=1.
 * - req_msg: mensagem original recebida (cfg, evt, etc.)
 *
 * Se req_msg->qos != 1, não envia nada.
 */
void mesh_proto_qos_send_ack_ok(const mesh_msg_t *req_msg);

#endif /* MESH_PROTO_H */
