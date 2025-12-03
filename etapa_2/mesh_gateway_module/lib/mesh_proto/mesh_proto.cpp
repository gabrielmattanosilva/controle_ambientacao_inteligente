#include "mesh_proto.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>

/**
 * @file mesh_proto.cpp
 * @brief Implementação do parser, builders e gerenciador de QoS do protocolo mesh.
 *
 * Este arquivo contém:
 * - Funções auxiliares internas de conversão/cópia;
 * - Implementação de @ref mesh_proto_parse() e dos builders JSON;
 * - Implementação do gerenciador de QoS (registro, retries, ACK automático).
 */

/* ============================================================
 * HELPERS INTERNOS
 * ============================================================ */

/**
 * @brief Converte uma string de tipo JSON no enum @ref mesh_msg_type_t.
 *
 * @param t String do campo "type" no JSON (ex.: "tele", "cfg", "hb").
 * @return Tipo de mensagem correspondente ou MESH_MSG_UNKNOWN se não reconhecido.
 */
static mesh_msg_type_t type_from_str(const char *t)
{
    if (!t || !t[0])
    {
        return MESH_MSG_UNKNOWN;
    }
    if (!strcmp(t, "tele"))
    {
        return MESH_MSG_TELE;
    }
    if (!strcmp(t, "state"))
    {
        return MESH_MSG_STATE;
    }
    if (!strcmp(t, "cfg"))
    {
        return MESH_MSG_CFG;
    }
    if (!strcmp(t, "hb"))
    {
        return MESH_MSG_HB;
    }
    if (!strcmp(t, "evt"))
    {
        return MESH_MSG_EVT;
    }
    if (!strcmp(t, "hello"))
    {
        return MESH_MSG_HELLO;
    }
    if (!strcmp(t, "ack"))
    {
        return MESH_MSG_ACK;
    }
    if (!strcmp(t, "time"))
    {
        return MESH_MSG_TIME;
    }
    return MESH_MSG_UNKNOWN;
}

/**
 * @brief Cópia segura de string com garantia de terminação em '\0'.
 *
 * Copia até @p dst_size - 1 caracteres de @p src para @p dst e garante que
 * @p dst seja sempre terminada em '\0'. Se @p src for nulo, produz string vazia.
 *
 * @param dst      Buffer de destino.
 * @param dst_size Tamanho total do buffer de destino.
 * @param src      String de origem (pode ser nula).
 */
static void safe_copy(char *dst, size_t dst_size, const char *src)
{
    if (!dst || dst_size == 0)
    {
        return;
    }

    if (!src)
    {
        dst[0] = '\0';
        return;
    }

    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = '\0';
}

/* ============================================================
 * PARSE
 * ============================================================ */

bool mesh_proto_parse(const char *json_str, mesh_msg_t *out_msg)
{
    if (!json_str || !out_msg)
    {
        return false;
    }

    memset(out_msg, 0, sizeof(*out_msg));

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json_str);
    if (err)
    {
        return false;
    }

    const char *id = doc["id"] | "";
    uint32_t ts = doc["ts"] | 0;
    uint8_t qos = doc["qos"] | 0;
    const char *src = doc["src"] | "";
    const char *dst = doc["dst"] | "";
    const char *typ = doc["type"] | "";

    safe_copy(out_msg->id, sizeof(out_msg->id), id);
    safe_copy(out_msg->src, sizeof(out_msg->src), src);
    safe_copy(out_msg->dst, sizeof(out_msg->dst), dst);
    out_msg->ts = ts;
    out_msg->qos = qos;
    out_msg->type = type_from_str(typ);

    JsonObject data = doc["data"].as<JsonObject>();
    if (!data)
    {
        return true; /* Header ok, sem data */
    }

    /* TELE */
    if (out_msg->type == MESH_MSG_TELE)
    {
        if (data["t_out"].is<float>())
        {
            out_msg->tele.has_t_out = true;
            out_msg->tele.t_out = data["t_out"].as<float>();
        }
        if (data["rh_out"].is<float>())
        {
            out_msg->tele.has_rh_out = true;
            out_msg->tele.rh_out = data["rh_out"].as<float>();
        }
        if (data["lux_out"].is<long>())
        {
            out_msg->tele.has_lux_out = true;
            out_msg->tele.lux_out = (int)data["lux_out"].as<long>();
        }
        if (data["t_in"].is<float>())
        {
            out_msg->tele.has_t_in = true;
            out_msg->tele.t_in = data["t_in"].as<float>();
        }
        if (data["rh_in"].is<float>())
        {
            out_msg->tele.has_rh_in = true;
            out_msg->tele.rh_in = data["rh_in"].as<float>();
        }
        if (data["soil_moist"].is<long>())
        {
            out_msg->tele.has_soil_moist = true;
            out_msg->tele.soil_moist = (int)data["soil_moist"].as<long>();
        }
        if (data["lux_in"].is<long>())
        {
            out_msg->tele.has_lux_in = true;
            out_msg->tele.lux_in = (int)data["lux_in"].as<long>();
        }
    }

    /* STATE */
    if (out_msg->type == MESH_MSG_STATE)
    {
        if (data["intake_pwm"].is<long>())
        {
            out_msg->state.has_intake_pwm = true;
            out_msg->state.intake_pwm = (int)data["intake_pwm"].as<long>();
        }
        if (data["exhaust_pwm"].is<long>())
        {
            out_msg->state.has_exhaust_pwm = true;
            out_msg->state.exhaust_pwm = (int)data["exhaust_pwm"].as<long>();
        }
        if (data["humidifier"].is<long>())
        {
            out_msg->state.has_humidifier = true;
            out_msg->state.humidifier = (int)data["humidifier"].as<long>();
        }
        if (data["led_brig"].is<long>())
        {
            out_msg->state.has_led_brig = true;
            out_msg->state.led_brig = (int)data["led_brig"].as<long>();
        }
        if (data["led_rgb"].is<const char *>())
        {
            out_msg->state.has_led_rgb = true;
            safe_copy(out_msg->state.led_rgb,
                      sizeof(out_msg->state.led_rgb),
                      data["led_rgb"].as<const char *>());
        }
        if (data["irrigation"].is<long>())
        {
            out_msg->state.has_irrigation = true;
            out_msg->state.irrigation = (int)data["irrigation"].as<long>();
        }
    }

    /* CFG */
    if (out_msg->type == MESH_MSG_CFG)
    {
        if (data["mode"].is<long>())
        {
            out_msg->cfg.has_mode = true;
            out_msg->cfg.mode = (int)data["mode"].as<long>();
        }
        if (data["intake_pwm"].is<long>())
        {
            out_msg->cfg.has_intake_pwm = true;
            out_msg->cfg.intake_pwm = (int)data["intake_pwm"].as<long>();
        }
        if (data["exhaust_pwm"].is<long>())
        {
            out_msg->cfg.has_exhaust_pwm = true;
            out_msg->cfg.exhaust_pwm = (int)data["exhaust_pwm"].as<long>();
        }
        if (data["humidifier"].is<long>())
        {
            out_msg->cfg.has_humidifier = true;
            out_msg->cfg.humidifier = (int)data["humidifier"].as<long>();
        }
        if (data["irrigation"].is<long>())
        {
            out_msg->cfg.has_irrigation = true;
            out_msg->cfg.irrigation = (int)data["irrigation"].as<long>();
        }
        if (data["led_pwm"].is<long>())
        {
            out_msg->cfg.has_led_pwm = true;
            out_msg->cfg.led_pwm = (int)data["led_pwm"].as<long>();
        }
        if (data["led_rgb"].is<const char *>())
        {
            out_msg->cfg.has_led_rgb = true;
            safe_copy(out_msg->cfg.led_rgb,
                      sizeof(out_msg->cfg.led_rgb),
                      data["led_rgb"].as<const char *>());
        }
    }

    /* HB */
    if (out_msg->type == MESH_MSG_HB)
    {
        if (data["uptime_s"].is<long>())
        {
            out_msg->hb.has_uptime_s = true;
            out_msg->hb.uptime_s = (int)data["uptime_s"].as<long>();
        }
        if (data["rssi_dbm"].is<long>())
        {
            out_msg->hb.has_rssi_dbm = true;
            out_msg->hb.rssi_dbm = (int)data["rssi_dbm"].as<long>();
        }
    }

    /* EVT */
    if (out_msg->type == MESH_MSG_EVT)
    {
        if (data["event"].is<const char *>())
        {
            out_msg->evt.has_event = true;
            safe_copy(out_msg->evt.event,
                      sizeof(out_msg->evt.event),
                      data["event"].as<const char *>());
        }
        if (data["code"].is<long>())
        {
            out_msg->evt.has_code = true;
            out_msg->evt.code = (int)data["code"].as<long>();
        }
        if (data["level"].is<long>())
        {
            out_msg->evt.has_level = true;
            out_msg->evt.level = (int)data["level"].as<long>();
        }
    }

    /* HELLO */
    if (out_msg->type == MESH_MSG_HELLO)
    {
        if (data["node_id"].is<const char *>())
        {
            out_msg->hello.has_node_id = true;
            safe_copy(out_msg->hello.node_id,
                      sizeof(out_msg->hello.node_id),
                      data["node_id"].as<const char *>());
        }
        if (data["fw_ver"].is<const char *>())
        {
            out_msg->hello.has_fw_ver = true;
            safe_copy(out_msg->hello.fw_ver,
                      sizeof(out_msg->hello.fw_ver),
                      data["fw_ver"].as<const char *>());
        }
        if (data["extra"].is<const char *>())
        {
            out_msg->hello.has_extra = true;
            safe_copy(out_msg->hello.extra,
                      sizeof(out_msg->hello.extra),
                      data["extra"].as<const char *>());
        }
    }

    /* ACK */
    if (out_msg->type == MESH_MSG_ACK)
    {
        if (data["ref"].is<const char *>())
        {
            out_msg->ack.has_ref = true;
            safe_copy(out_msg->ack.ref,
                      sizeof(out_msg->ack.ref),
                      data["ref"].as<const char *>());
        }
        if (data["status"].is<const char *>())
        {
            out_msg->ack.has_status = true;
            safe_copy(out_msg->ack.status,
                      sizeof(out_msg->ack.status),
                      data["status"].as<const char *>());
        }
    }

    /* TIME */
    if (out_msg->type == MESH_MSG_TIME)
    {
        if (data["epoch"].is<long>())
        {
            out_msg->time_sync.has_epoch = true;
            out_msg->time_sync.epoch = (uint32_t)data["epoch"].as<long>();
        }
        if (data["tz_offset_min"].is<long>())
        {
            out_msg->time_sync.has_tz_offset_min = true;
            out_msg->time_sync.tz_offset_min = (int)data["tz_offset_min"].as<long>();
        }
    }

    return true;
}

/* ============================================================
 * BUILDERS
 * ============================================================ */

/**
 * @brief Serializa um JsonDocument em string JSON, com checagem de tamanho.
 *
 * @param doc      Documento JSON a ser serializado.
 * @param out_json Buffer de saída.
 * @param maxlen   Tamanho máximo do buffer @p out_json.
 *
 * @return true se a serialização coube no buffer; false em caso de erro.
 */
static bool serialize_doc(JsonDocument &doc,
                          char *out_json, size_t maxlen)
{
    if (!out_json || maxlen == 0)
    {
        return false;
    }

    size_t n = serializeJson(doc, out_json, maxlen);

    if (n == 0 || n >= maxlen)
    {
        return false;
    }
    return true;
}

bool mesh_proto_build_cfg_int(const char *id,
                              uint32_t ts,
                              uint8_t qos,
                              const char *src,
                              const char *dst,
                              const char *field_name,
                              int value,
                              char *out_json,
                              size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = qos;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "cfg";

    JsonObject data = doc["data"].to<JsonObject>();
    data[field_name] = value;

    return serialize_doc(doc, out_json, maxlen);
}

bool mesh_proto_build_cfg_str(const char *id,
                              uint32_t ts,
                              uint8_t qos,
                              const char *src,
                              const char *dst,
                              const char *field_name,
                              const char *value,
                              char *out_json,
                              size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = qos;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "cfg";

    JsonObject data = doc["data"].to<JsonObject>();
    data[field_name] = value ? value : "";

    return serialize_doc(doc, out_json, maxlen);
}

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
                                size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = qos;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "state";

    JsonObject data = doc["data"].to<JsonObject>();
    data["intake_pwm"] = intake_pwm;
    data["exhaust_pwm"] = exhaust_pwm;
    data["humidifier"] = humidifier;
    data["led_brig"] = led_brig;
    data["led_rgb"] = led_rgb ? led_rgb : "";
    data["irrigation"] = irrigation;

    return serialize_doc(doc, out_json, maxlen);
}

bool mesh_proto_build_hb(const char *id,
                         uint32_t ts,
                         const char *src,
                         const char *dst,
                         int uptime_s,
                         int rssi_dbm,
                         char *out_json,
                         size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = 0;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "hb";

    JsonObject data = doc["data"].to<JsonObject>();
    data["uptime_s"] = uptime_s;
    data["rssi_dbm"] = rssi_dbm;

    return serialize_doc(doc, out_json, maxlen);
}

bool mesh_proto_build_ack(const char *id,
                          uint32_t ts,
                          const char *src,
                          const char *dst,
                          const char *ref,
                          const char *status,
                          char *out_json,
                          size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = 0;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "ack";

    JsonObject data = doc["data"].to<JsonObject>();
    data["ref"] = ref ? ref : "";
    data["status"] = status ? status : "ok";

    return serialize_doc(doc, out_json, maxlen);
}

bool mesh_proto_build_hello(const char *id,
                            uint32_t ts,
                            uint8_t qos,
                            const char *src,
                            const char *dst,
                            const char *node_id,
                            const char *fw_ver,
                            const char *extra,
                            char *out_json,
                            size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = qos;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "hello";

    JsonObject data = doc["data"].to<JsonObject>();
    data["node_id"] = node_id ? node_id : "";
    data["fw_ver"] = fw_ver ? fw_ver : "";
    data["extra"] = extra ? extra : "";

    return serialize_doc(doc, out_json, maxlen);
}

bool mesh_proto_build_evt(const char *id,
                          uint32_t ts,
                          uint8_t qos,
                          const char *src,
                          const char *dst,
                          const char *event,
                          int code,
                          int level,
                          char *out_json,
                          size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = qos;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "evt";

    JsonObject data = doc["data"].to<JsonObject>();
    data["event"] = event ? event : "";
    data["code"] = code;
    data["level"] = level;

    return serialize_doc(doc, out_json, maxlen);
}

bool mesh_proto_build_time(const char *id,
                           uint32_t ts,
                           uint8_t qos,
                           const char *src,
                           const char *dst,
                           uint32_t epoch,
                           int tz_offset_min,
                           char *out_json,
                           size_t maxlen)
{
    JsonDocument doc;
    doc["id"] = id ? id : "";
    doc["ts"] = ts;
    doc["qos"] = qos;
    doc["src"] = src ? src : "";
    doc["dst"] = dst ? dst : "";
    doc["type"] = "time";

    JsonObject data = doc["data"].to<JsonObject>();
    data["epoch"] = epoch;
    data["tz_offset_min"] = tz_offset_min;

    return serialize_doc(doc, out_json, maxlen);
}

/* ============================================================
 * QoS MANAGER
 * ============================================================ */

#define MESH_QOS_TIMEOUT_MS 1000UL /**< Timeout entre retries, em milissegundos. */
#define MESH_QOS_MAX_RETRIES 3     /**< Número máximo de tentativas de reenvio. */
#define MESH_QOS_MAX_PENDING 4     /**< Máximo de mensagens QoS pendentes. */

/**
 * @brief Estrutura interna de controle de mensagens QoS pendentes.
 */
typedef struct
{
    bool used;             /**< true se o slot está em uso. */
    char id[8];            /**< ID da mensagem (campo "id"). */
    char json[256];        /**< Buffer com o JSON completo a ser reenviado. */
    uint8_t retries;       /**< Número de retries já realizados. */
    uint32_t last_send_ms; /**< Timestamp (millis) do último envio. */
} mesh_qos_pending_t;

/**
 * @brief Lista estática de pendências QoS.
 * */
static mesh_qos_pending_t g_qos_pending[MESH_QOS_MAX_PENDING];

/**
 * @brief Callback de envio físico utilizado pelo gerenciador de QoS.
 * */
static mesh_proto_send_cb_t g_qos_send_cb = nullptr;

/**
 * @brief Contador interno para geração de IDs de ACK.
 * */
static uint16_t g_qos_id_counter = 0;

/**
 * @brief Gera um ID textual incremental em formato hexadecimal de 4 dígitos.
 *
 * @param buf Buffer de destino para o ID.
 * @param len Tamanho do buffer de destino.
 */
static void qos_gen_id(char *buf, size_t len)
{
    g_qos_id_counter++;
    if (!g_qos_id_counter)
    {
        g_qos_id_counter = 1;
    }
    snprintf(buf, len, "%04X", g_qos_id_counter);
}

/**
 * @brief Procura um slot livre na lista de pendências QoS.
 *
 * @return Ponteiro para slot livre ou nullptr se não houver.
 */
static mesh_qos_pending_t *qos_find_free_slot()
{
    for (int i = 0; i < MESH_QOS_MAX_PENDING; ++i)
    {
        if (!g_qos_pending[i].used)
        {
            return &g_qos_pending[i];
        }
    }
    return nullptr;
}

/**
 * @brief Procura uma pendência QoS pelo seu ID de mensagem.
 *
 * @param id ID da mensagem (campo "id" do JSON).
 * @return Ponteiro para a pendência encontrada ou nullptr se não houver.
 */
static mesh_qos_pending_t *qos_find_by_id(const char *id)
{
    for (int i = 0; i < MESH_QOS_MAX_PENDING; ++i)
    {
        if (g_qos_pending[i].used && !strcmp(g_qos_pending[i].id, id))
        {
            return &g_qos_pending[i];
        }
    }
    return nullptr;
}

void mesh_proto_qos_init(mesh_proto_send_cb_t send_cb)
{
    g_qos_send_cb = send_cb;
    memset(g_qos_pending, 0, sizeof(g_qos_pending));
    g_qos_id_counter = 0;
}

bool mesh_proto_qos_register_and_send(const char *id,
                                      const char *json)
{
    if (!g_qos_send_cb || !id || !json)
    {
        /* Sem callback ou parâmetros inválidos → não registra */
        return false;
    }

    mesh_qos_pending_t *slot = qos_find_free_slot();
    if (!slot)
    {
        /* Sem slot, manda mesmo assim, mas sem tracking */
        g_qos_send_cb(json);
        return false;
    }

    memset(slot, 0, sizeof(*slot));
    slot->used = true;
    safe_copy(slot->id, sizeof(slot->id), id);
    safe_copy(slot->json, sizeof(slot->json), json);
    slot->retries = 0;
    slot->last_send_ms = millis();

    g_qos_send_cb(slot->json);
    return true;
}

void mesh_proto_qos_on_ack(const mesh_msg_t *msg)
{
    if (!msg || msg->type != MESH_MSG_ACK)
    {
        return;
    }
    if (!msg->ack.has_ref)
    {
        return;
    }

    mesh_qos_pending_t *slot = qos_find_by_id(msg->ack.ref);
    if (!slot)
    {
        /* ACK de id desconhecido → ignora */
        return;
    }
    slot->used = false;
}

void mesh_proto_qos_poll(void)
{
    if (!g_qos_send_cb)
        return;

    uint32_t now = millis();
    for (int i = 0; i < MESH_QOS_MAX_PENDING; ++i)
    {
        mesh_qos_pending_t &p = g_qos_pending[i];
        if (!p.used)
        {
            continue;
        }
        if ((now - p.last_send_ms) >= MESH_QOS_TIMEOUT_MS)
        {
            if (p.retries < MESH_QOS_MAX_RETRIES)
            {
                p.retries++;
                p.last_send_ms = now;
                g_qos_send_cb(p.json);
            }
            else
            {
                /* Falha definitiva, descarta */
                p.used = false;
            }
        }
    }
}

void mesh_proto_qos_send_ack_ok(const mesh_msg_t *req_msg)
{
    if (!g_qos_send_cb || !req_msg)
    {
        return;
    }
    if (req_msg->qos != 1)
    {
        return;
    } /* Só QoS1 ganha ACK automático */

    char id[8];
    char json[256];

    qos_gen_id(id, sizeof(id));

    /* Troca src/dst */
    const char *src = req_msg->dst;
    const char *dst = req_msg->src;

    if (!mesh_proto_build_ack(id,
                              (uint32_t)millis(),
                              src,
                              dst,
                              req_msg->id,
                              "ok",
                              json,
                              sizeof(json)))
    {
        return;
    }

    g_qos_send_cb(json);
}
