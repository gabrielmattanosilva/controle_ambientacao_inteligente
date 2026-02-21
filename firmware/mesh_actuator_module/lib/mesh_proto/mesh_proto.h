/**
 * @file mesh_proto.h
 * @brief Declarações públicas do protocolo de mensagens da rede mesh.
 *
 * Este arquivo contém:
 * - Definições de tipos e enums para os diferentes tipos de mensagens;
 * - Declaração da estrutura principal ::mesh_msg_t (formato interno das mensagens);
 * - Protótipos das funções de parse e de construção (builders) de JSON;
 * - Protótipos das funções do gerenciador de QoS (registro, retries, ACK automático).
 */

#ifndef MESH_PROTO_H
#define MESH_PROTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Tipos de mensagens suportados pelo protocolo mesh.
 *
 * Cada valor corresponde ao campo textual "type" no JSON:
 * - "tele"  → MESH_MSG_TELE
 * - "state" → MESH_MSG_STATE
 * - "cfg"   → MESH_MSG_CFG
 * - "hb"    → MESH_MSG_HB
 * - "evt"   → MESH_MSG_EVT
 * - "hello" → MESH_MSG_HELLO
 * - "ack"   → MESH_MSG_ACK
 * - "time"  → MESH_MSG_TIME
 */
typedef enum
{
    MESH_MSG_UNKNOWN = 0, /**< Tipo desconhecido ou não mapeado. */
    MESH_MSG_TELE,        /**< Telemetria de sensores (internos/externos). */
    MESH_MSG_STATE,       /**< Estado atual de atuadores. */
    MESH_MSG_CFG,         /**< Comandos/configurações de atuadores ou modo. */
    MESH_MSG_HB,          /**< Heartbeat (uptime, RSSI, etc.). */
    MESH_MSG_EVT,         /**< Eventos (alarme, falhas, logs simbólicos). */
    MESH_MSG_HELLO,       /**< Mensagem de identificação do nó. */
    MESH_MSG_ACK,         /**< Confirmação de recebimento (QoS1). */
    MESH_MSG_TIME         /**< Sincronização de tempo. */
} mesh_msg_type_t;

/**
 * @brief Estrutura principal de mensagem do protocolo mesh (formato interno).
 *
 * Esta estrutura representa uma mensagem já parseada de JSON, contendo:
 * - Cabeçalho genérico (id, timestamp, QoS, src, dst, type);
 * - Sub-estruturas específicas para cada tipo de mensagem (tele, state, cfg, etc.).
 *
 * Campos booleanos "has_*" indicam se o respectivo dado estava presente no JSON.
 */
typedef struct
{
    char id[8];           /**< Identificador textual da mensagem, ex.: "000A". */
    uint32_t ts;          /**< Timestamp (unidade depende do produtor, ex.: ms ou s). */
    uint8_t qos;          /**< Nível de serviço QoS (0 = sem ACK, 1 = com ACK). */
    char src[16];         /**< Origem da mensagem, ex.: "blynk-gw", "msh-gw", "act-00". */
    char dst[16];         /**< Destino da mensagem, ex.: "blynk-gw", "msh-gw", "*" (broadcast). */
    mesh_msg_type_t type; /**< Tipo de mensagem (tele, cfg, state, etc.). */

    /**
     * @brief Sub-estrutura de telemetria (type = MESH_MSG_TELE, "tele").
     */
    struct
    {
        bool has_t_out;   /**< true se t_out veio no JSON. */
        bool has_rh_out;  /**< true se rh_out veio no JSON. */
        bool has_lux_out; /**< true se lux_out veio no JSON. */
        float t_out;      /**< Temperatura externa. */
        float rh_out;     /**< Umidade relativa externa. */
        int lux_out;      /**< Iluminância externa (lux). */

        bool has_t_in;       /**< true se t_in veio no JSON. */
        bool has_rh_in;      /**< true se rh_in veio no JSON. */
        bool has_soil_moist; /**< true se soil_moist veio no JSON. */
        bool has_lux_in;     /**< true se lux_in veio no JSON. */
        float t_in;          /**< Temperatura interna. */
        float rh_in;         /**< Umidade relativa interna. */
        int soil_moist;      /**< Umidade do solo (unidade conforme sensor). */
        int lux_in;          /**< Iluminância interna (lux). */
    } tele;

    /**
     * @brief Sub-estrutura de estado de atuadores (type = MESH_MSG_STATE, "state").
     */
    struct
    {
        bool has_intake_pwm;  /**< true se intake_pwm veio no JSON. */
        bool has_exhaust_pwm; /**< true se exhaust_pwm veio no JSON. */
        bool has_humidifier;  /**< true se humidifier veio no JSON. */
        bool has_led_brig;    /**< true se led_brig veio no JSON. */
        bool has_led_rgb;     /**< true se led_rgb veio no JSON. */
        bool has_irrigation;  /**< true se irrigation veio no JSON. */

        int intake_pwm;   /**< PWM do ventilador de entrada. */
        int exhaust_pwm;  /**< PWM do ventilador de exaustão. */
        int humidifier;   /**< Estado do umidificador (ex.: 0/1). */
        int led_brig;     /**< Brilho do LED (dimmer). */
        char led_rgb[16]; /**< Cor do LED em formato texto (ex.: "#RRGGBB"). */
        int irrigation;   /**< Estado da irrigação (ex.: 0/1). */
    } state;

    /**
     * @brief Sub-estrutura de configuração/comando (type = MESH_MSG_CFG, "cfg").
     *
     * Utilizada para alterar modo de operação, setpoints ou estados forçados
     * de atuadores.
     */
    struct
    {
        bool has_mode;        /**< true se mode veio no JSON. */
        bool has_intake_pwm;  /**< true se intake_pwm veio no JSON. */
        bool has_exhaust_pwm; /**< true se exhaust_pwm veio no JSON. */
        bool has_humidifier;  /**< true se humidifier veio no JSON. */
        bool has_irrigation;  /**< true se irrigation veio no JSON. */
        bool has_led_pwm;     /**< true se led_pwm veio no JSON. */
        bool has_led_rgb;     /**< true se led_rgb veio no JSON. */

        int mode;         /**< Modo de operação (ex.: AUTO/MANUAL). */
        int intake_pwm;   /**< PWM desejado para ventilador de entrada. */
        int exhaust_pwm;  /**< PWM desejado para ventilador de exaustão. */
        int humidifier;   /**< Comando para umidificador. */
        int irrigation;   /**< Comando para irrigação. */
        int led_pwm;      /**< PWM desejado para LED (brilho). */
        char led_rgb[16]; /**< Cor desejada para o LED. */
    } cfg;

    /**
     * @brief Sub-estrutura de heartbeat (type = MESH_MSG_HB, "hb").
     */
    struct
    {
        bool has_uptime_s; /**< true se uptime_s veio no JSON. */
        bool has_rssi_dbm; /**< true se rssi_dbm veio no JSON. */
        int uptime_s;      /**< Uptime do nó, em segundos. */
        int rssi_dbm;      /**< Nível de sinal (RSSI) em dBm. */
    } hb;

    /**
     * @brief Sub-estrutura de eventos (type = MESH_MSG_EVT, "evt").
     */
    struct
    {
        bool has_event; /**< true se event veio no JSON. */
        bool has_code;  /**< true se code veio no JSON. */
        bool has_level; /**< true se level veio no JSON. */

        char event[16]; /**< Nome simbólico do evento (ex.: "overheat"). */
        int code;       /**< Código numérico opcional do evento. */
        int level;      /**< Severidade ou nível (ex.: 0..3). */
    } evt;

    /**
     * @brief Sub-estrutura de identificação (type = MESH_MSG_HELLO, "hello").
     */
    struct
    {
        bool has_node_id; /**< true se node_id veio no JSON. */
        bool has_fw_ver;  /**< true se fw_ver veio no JSON. */
        bool has_extra;   /**< true se extra veio no JSON. */

        char node_id[16]; /**< Identificador do nó, ex.: "ext-sen-00". */
        char fw_ver[16];  /**< Versão de firmware, ex.: "1.0.0". */
        char extra[32];   /**< Informações adicionais livres. */
    } hello;

    /**
     * @brief Sub-estrutura de ACK (type = MESH_MSG_ACK, "ack").
     *
     * Utilizada para confirmar o recebimento de mensagens QoS1.
     */
    struct
    {
        bool has_ref;    /**< true se ref veio no JSON. */
        bool has_status; /**< true se status veio no JSON. */

        char ref[8];    /**< ID da mensagem referenciada (doc["id"]). */
        char status[8]; /**< Status textual, ex.: "ok", "err". */
    } ack;

    /**
     * @brief Sub-estrutura de sincronização de tempo (type = MESH_MSG_TIME, "time").
     */
    struct
    {
        bool has_epoch;         /**< true se epoch veio no JSON. */
        bool has_tz_offset_min; /**< true se tz_offset_min veio no JSON. */

        uint32_t epoch;    /**< Epoch em segundos (Unix time). */
        int tz_offset_min; /**< Offset de fuso horário em minutos. */
    } time_sync;

} mesh_msg_t;

/* ============================================================
 * PARSE + BUILDERS
 * ============================================================ */

/**
 * @brief Faz o parse de uma string JSON no formato do protocolo mesh.
 *
 * @param json_str  String JSON de entrada.
 * @param out_msg   Ponteiro para estrutura @ref mesh_msg_t que será preenchida.
 * @return true se o parse foi bem-sucedido; false em caso de erro de JSON ou parâmetros inválidos.
 */
bool mesh_proto_parse(const char *json_str, mesh_msg_t *out_msg);

/**
 * @brief Monta uma mensagem JSON do tipo "cfg" com um campo inteiro.
 *
 * @param id         ID da mensagem (string, ex.: "000A").
 * @param ts         Timestamp a ser inserido no campo "ts".
 * @param qos        Nível de QoS (0 ou 1).
 * @param src        Origem (campo "src").
 * @param dst        Destino (campo "dst").
 * @param field_name Nome do campo dentro de "data" (ex.: "intake_pwm").
 * @param value      Valor inteiro a ser escrito em data[field_name].
 * @param out_json   Buffer de saída para o JSON gerado.
 * @param maxlen     Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
bool mesh_proto_build_cfg_int(const char *id,
                              uint32_t ts,
                              uint8_t qos,
                              const char *src,
                              const char *dst,
                              const char *field_name,
                              int value,
                              char *out_json,
                              size_t maxlen);

/**
 * @brief Monta uma mensagem JSON do tipo "cfg" com um campo string.
 *
 * @param id         ID da mensagem (string, ex.: "000A").
 * @param ts         Timestamp a ser inserido no campo "ts".
 * @param qos        Nível de QoS (0 ou 1).
 * @param src        Origem (campo "src").
 * @param dst        Destino (campo "dst").
 * @param field_name Nome do campo dentro de "data" (ex.: "led_rgb").
 * @param value      Valor string a ser escrito em data[field_name].
 * @param out_json   Buffer de saída para o JSON gerado.
 * @param maxlen     Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
bool mesh_proto_build_cfg_str(const char *id,
                              uint32_t ts,
                              uint8_t qos,
                              const char *src,
                              const char *dst,
                              const char *field_name,
                              const char *value,
                              char *out_json,
                              size_t maxlen);

/**
 * @brief Monta uma mensagem JSON do tipo "state" para o atuador.
 *
 * @param id           ID da mensagem (string, ex.: "000A").
 * @param ts           Timestamp a ser inserido no campo "ts".
 * @param qos          Nível de QoS (0 ou 1).
 * @param src          Origem (campo "src").
 * @param dst          Destino (campo "dst").
 * @param intake_pwm   PWM do ventilador de entrada.
 * @param exhaust_pwm  PWM do ventilador de exaustão.
 * @param humidifier   Estado do umidificador.
 * @param led_brig     Brilho do LED.
 * @param led_rgb      Cor do LED em string (ex.: "#RRGGBB").
 * @param irrigation   Estado da irrigação.
 * @param out_json     Buffer de saída para o JSON gerado.
 * @param maxlen       Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
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

/**
 * @brief Monta uma mensagem JSON do tipo "hb" (heartbeat).
 *
 * @param id        ID da mensagem (string, ex.: "000A").
 * @param ts        Timestamp a ser inserido no campo "ts".
 * @param src       Origem (campo "src").
 * @param dst       Destino (campo "dst").
 * @param uptime_s  Uptime do nó em segundos.
 * @param rssi_dbm  Intensidade de sinal em dBm.
 * @param out_json  Buffer de saída para o JSON gerado.
 * @param maxlen    Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
bool mesh_proto_build_hb(const char *id,
                         uint32_t ts,
                         const char *src,
                         const char *dst,
                         int uptime_s,
                         int rssi_dbm,
                         char *out_json,
                         size_t maxlen);

/**
 * @brief Monta uma mensagem JSON do tipo "ack".
 *
 * @param id        ID da mensagem de ACK (string, ex.: "000B").
 * @param ts        Timestamp a ser inserido no campo "ts".
 * @param src       Origem (campo "src").
 * @param dst       Destino (campo "dst").
 * @param ref       ID da mensagem referenciada (campo "ref" em data).
 * @param status    Status textual (ex.: "ok", "err").
 * @param out_json  Buffer de saída para o JSON gerado.
 * @param maxlen    Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
bool mesh_proto_build_ack(const char *id,
                          uint32_t ts,
                          const char *src,
                          const char *dst,
                          const char *ref,
                          const char *status,
                          char *out_json,
                          size_t maxlen);

/**
 * @brief Monta uma mensagem JSON do tipo "hello".
 *
 * @param id        ID da mensagem (string, ex.: "000A").
 * @param ts        Timestamp a ser inserido no campo "ts".
 * @param qos       Nível de QoS (0 ou 1).
 * @param src       Origem (campo "src").
 * @param dst       Destino (campo "dst").
 * @param node_id   Identificador do nó (ex.: "ext-sen-00").
 * @param fw_ver    Versão de firmware (ex.: "1.0.0").
 * @param extra     Campo livre com informações adicionais.
 * @param out_json  Buffer de saída para o JSON gerado.
 * @param maxlen    Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
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

/**
 * @brief Monta uma mensagem JSON do tipo "evt".
 *
 * @param id        ID da mensagem (string, ex.: "000A").
 * @param ts        Timestamp a ser inserido no campo "ts".
 * @param qos       Nível de QoS (0 ou 1).
 * @param src       Origem (campo "src").
 * @param dst       Destino (campo "dst").
 * @param event     Nome simbólico do evento (ex.: "overheat").
 * @param code      Código numérico do evento.
 * @param level     Severidade/nível do evento.
 * @param out_json  Buffer de saída para o JSON gerado.
 * @param maxlen    Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
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

/**
 * @brief Monta uma mensagem JSON do tipo "time".
 *
 * @param id            ID da mensagem (string, ex.: "000A").
 * @param ts            Timestamp a ser inserido no campo "ts".
 * @param qos           Nível de QoS (0 ou 1).
 * @param src           Origem (campo "src").
 * @param dst           Destino (campo "dst").
 * @param epoch         Epoch em segundos (Unix time).
 * @param tz_offset_min Offset de fuso horário em minutos.
 * @param out_json      Buffer de saída para o JSON gerado.
 * @param maxlen        Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização foi bem-sucedida; false em caso de erro ou buffer insuficiente.
 */
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
 * QoS MANAGER
 * ============================================================ */

/**
 * @brief Callback genérico para envio físico (UART, mesh, etc.).
 *
 * Deve apontar para uma função que, de fato, envie o JSON pela camada física
 * desejada (ex.: UART, Wi-Fi, rádio mesh).
 *
 * Exemplo de uso:
 * @code
 *     mesh_proto_qos_init(ipc_uart_send_json);
 * @endcode
 *
 * @param json String JSON completa a ser enviada.
 */
typedef void (*mesh_proto_send_cb_t)(const char *json);

/**
 * @brief Inicializa o gerenciador de QoS interno da lib.
 *
 * Limpa a lista de pendências e registra o callback de envio físico.
 *
 * @param send_cb Ponteiro para função que realiza o envio real do JSON.
 */
void mesh_proto_qos_init(mesh_proto_send_cb_t send_cb);

/**
 * @brief Registra uma mensagem QoS1 como pendente e a envia.
 *
 * A mensagem é armazenada internamente para permitir retries automáticos
 * até que um ACK correspondente seja recebido ou que o número máximo de
 * tentativas seja atingido.
 *
 * @param id   ID da mensagem (campo "id" do JSON).
 * @param json Payload JSON completo já montado.
 *
 * @return true se a mensagem foi registrada para QoS (há slot livre);
 *         false se não havia slot disponível (neste caso, a mensagem é
 *         enviada uma vez, mas sem tracking/retry).
 */
bool mesh_proto_qos_register_and_send(const char *id,
                                      const char *json);

/**
 * @brief Notifica o gerenciador de QoS sobre a chegada de um ACK.
 *
 * Deve ser chamada sempre que uma mensagem do tipo MESH_MSG_ACK for recebida
 * e parseada por @ref mesh_proto_parse().
 *
 * Se o campo ack.ref corresponder a uma pendência QoS, esta é removida.
 *
 * @param msg Mensagem ACK já parseada.
 */
void mesh_proto_qos_on_ack(const mesh_msg_t *msg);

/**
 * @brief Rotina periódica de gerenciamento de timeout/retries de QoS.
 *
 * Deve ser chamada no loop principal de cada módulo que utiliza QoS1.
 * Internamente, verifica pendências e:
 * - Reenvia mensagens cujo timeout expirou (até MESH_QOS_MAX_RETRIES);
 * - Descarta pendências que excederam o número máximo de retries.
 */
void mesh_proto_qos_poll(void);

/**
 * @brief Envia automaticamente um ACK "ok" para mensagens recebidas com QoS=1.
 *
 * Troca src/dst da mensagem original, gera um novo id de ACK e envia uma
 * mensagem "ack" com data.ref = req_msg->id e data.status = "ok".
 *
 * @param req_msg Mensagem original recebida (cfg, evt, etc.).
 *
 * @note Se req_msg->qos != 1 ou se o callback de envio não estiver definido,
 *       nada é enviado.
 */
void mesh_proto_qos_send_ack_ok(const mesh_msg_t *req_msg);

#endif /* MESH_PROTO_H */
