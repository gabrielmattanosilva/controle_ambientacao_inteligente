/**
 * @file blynk_client.cpp
 * @brief Camada de integração com Blynk (WiFi, callbacks, Vpins, etc.).
 *
 * Este arquivo implementa:
 * - Inicialização de WiFi e Blynk;
 * - Callbacks de Vpins (BLYNK_WRITE, BLYNK_CONNECTED);
 * - Conversão de comandos do Blynk em mensagens CFG para a malha (via mesh_proto);
 * - Tratamento de mensagens vindas da malha (TELE, STATE, HB, EVT, HELLO, TIME);
 * - Aplicação de sincronismo de tempo recebido do mesh_gateway usando mensagens TIME;
 * - Lógica de modo automático/manual comandado por V13.
 */

#include "blynk_client.h"
#include "credentials.h"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <time.h>
#include <sys/time.h>
#include "mesh_proto.h"

#define NODE_BLYNK_GW "blynk-gw"
#define NODE_MSH_GW "msh-gw"
#define NODE_EXT_SEN "ext-sen-00"
#define NODE_INT_SEN "int-sen-00"
#define NODE_ACT "act-00"

static BlynkTimer g_timer;
static uint16_t g_msg_counter = 0;
static int g_mode = 0;
static bool g_time_sync_ok = false;
static bool g_mesh_hello_ok = false;

/* ============================================================
 * HELPERS INTERNOS
 * ============================================================ */

/**
 * @brief Gera um ID textual incremental para mensagens da malha.
 *
 * O ID é gerado em formato hexadecimal de 4 dígitos (ex.: "0001", "0002"...),
 * sendo usado no campo @c id das mensagens JSON enviadas via mesh_proto.
 *
 * @param buf Buffer de destino para o ID.
 * @param len Tamanho máximo do buffer @p buf.
 */
static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter)
    {
        g_msg_counter = 1;
    }
    snprintf(buf, len, "%04X", g_msg_counter);
}

/**
 * @brief Aplica o TIME recebido do mesh_gateway_module ao RTC interno do ESP32.
 *
 * A partir de um @p epoch em segundos (Unix time) e de um deslocamento de fuso
 * horário @p tz_offset_min, ajusta o relógio interno com @c settimeofday() e
 * imprime no log a data/hora resultante.
 *
 * Também:
 * - Marca @ref g_time_sync_ok como @c true para fins de debug;
 * - Garante que o painel Blynk comece em modo AUTOMÁTICO (AUTO = 0) e
 *   força V13 para 0.
 *
 * @param epoch         Tempo Unix (segundos desde 1970-01-01 00:00:00 UTC).
 * @param tz_offset_min Offset de fuso horário em minutos.
 */
static void apply_time_from_mesh(uint32_t epoch, int tz_offset_min)
{
    struct timeval tv;
    tv.tv_sec = (time_t)epoch;
    tv.tv_usec = 0;

    if (settimeofday(&tv, nullptr) != 0)
    {
        Serial.println("[TIME] settimeofday() falhou");
        return;
    }

    time_t t = (time_t)epoch;
    struct tm tm_local;
    localtime_r(&t, &tm_local);

    Serial.printf("[TIME] RTC sincronizado: %04d-%02d-%02d %02d:%02d:%02d (tz_offset_min=%d)\n",
                  tm_local.tm_year + 1900,
                  tm_local.tm_mon + 1,
                  tm_local.tm_mday,
                  tm_local.tm_hour,
                  tm_local.tm_min,
                  tm_local.tm_sec,
                  tz_offset_min);

    g_time_sync_ok = true;

    /* Garante que o painel Blynk comece em modo AUTOMATICO (AUTO = 0) */
    g_mode = 0;
    Blynk.virtualWrite(V13, 0);
}

/**
 * @brief Envia um campo inteiro de configuração (CFG) via mesh_proto com QoS1.
 *
 * Monta uma mensagem @c cfg JSON com um único campo inteiro @p field = @p value,
 * usando @ref mesh_proto_build_cfg_int(), e registra para QoS1 com
 * @ref mesh_proto_qos_register_and_send().
 *
 * @param field Nome do campo dentro de @c data no JSON (ex.: "mode").
 * @param value Valor inteiro a ser enviado.
 */
static void send_cfg_int_field(const char *field, int value)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_cfg_int(id,
                                  (uint32_t)millis(),
                                  1,
                                  NODE_BLYNK_GW,
                                  NODE_ACT,
                                  field,
                                  value,
                                  json,
                                  sizeof(json)))
    {
        return;
    }

    mesh_proto_qos_register_and_send(id, json);

    Serial.print("[TX CFG INT] ");
    Serial.println(json);
}

/**
 * @brief Envia um campo string de configuração (CFG) via mesh_proto com QoS1.
 *
 * Monta uma mensagem @c cfg JSON com um único campo string @p field = @p value,
 * usando @ref mesh_proto_build_cfg_str(), e registra para QoS1 com
 * @ref mesh_proto_qos_register_and_send().
 *
 * @param field Nome do campo dentro de @c data no JSON (ex.: "led_rgb").
 * @param value Valor string a ser enviado.
 */
static void send_cfg_str_field(const char *field, const char *value)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_cfg_str(id,
                                  (uint32_t)millis(),
                                  1,
                                  NODE_BLYNK_GW,
                                  NODE_ACT,
                                  field,
                                  value,
                                  json,
                                  sizeof(json)))
    {
        return;
    }

    mesh_proto_qos_register_and_send(id, json);

    Serial.print("[TX CFG STR] ");
    Serial.println(json);
}

/* ============================================================
 * BLYNK CALLBACKS
 * ============================================================ */

/**
 * @brief Callback invocado quando o dispositivo se conecta ao servidor Blynk.
 *
 * - Faz sync dos Vpins V14..V19 (parâmetros de atuadores);
 * - Não sincroniza V13, pois o modo é sempre comandado pelo firmware;
 * - Garante que V13 comece em modo AUTOMÁTICO (0) a cada reconexão.
 */
BLYNK_CONNECTED()
{
    Serial.println("[BLYNK] Connected, syncing V14..V19");
    Blynk.syncVirtual(V14, V15, V16, V17, V18, V19);
    g_mode = 0;
    Blynk.virtualWrite(V13, 0);
}

/**
 * @brief Callback para escrita em V13 (seleção de modo AUTO/MANUAL).
 *
 * - V13 = 0 → modo automático (g_mode = 0);
 * - V13 = 1 → modo manual   (g_mode = 1);
 * - Envia uma mensagem CFG "mode" para o mesh_gateway.
 */
BLYNK_WRITE(V13)
{
    g_mode = param.asInt() ? 1 : 0;
    Serial.printf("[BLYNK] V13 (modo) = %d\n", g_mode);
    send_cfg_int_field("mode", g_mode);
}

/**
 * @brief Callback de V14 (intake_pwm). Só atua em modo MANUAL.
 */
BLYNK_WRITE(V14)
{
    if (!g_mode)
    {
        Serial.println("[BLYNK] V14 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("intake_pwm", param.asInt());
}

/**
 * @brief Callback de V15 (exhaust_pwm). Só atua em modo MANUAL.
 */
BLYNK_WRITE(V15)
{
    if (!g_mode)
    {
        Serial.println("[BLYNK] V15 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("exhaust_pwm", param.asInt());
}

/**
 * @brief Callback de V16 (humidifier). Só atua em modo MANUAL.
 */
BLYNK_WRITE(V16)
{
    if (!g_mode)
    {
        Serial.println("[BLYNK] V16 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("humidifier", param.asInt());
}

/**
 * @brief Callback de V17 (led_pwm). Só atua em modo MANUAL.
 */
BLYNK_WRITE(V17)
{
    if (!g_mode)
    {
        Serial.println("[BLYNK] V17 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("led_pwm", param.asInt());
}

/**
 * @brief Callback de V18 (led_rgb). Só atua em modo MANUAL.
 */
BLYNK_WRITE(V18)
{
    if (!g_mode)
    {
        Serial.println("[BLYNK] V18 ignorado (modo AUTO)");
        return;
    }
    send_cfg_str_field("led_rgb", param.asStr());
}

/**
 * @brief Callback de V19 (irrigation). Só atua em modo MANUAL.
 */
BLYNK_WRITE(V19)
{
    if (!g_mode)
    {
        Serial.println("[BLYNK] V19 ignorado (modo AUTO)");
        return;
    }
    send_cfg_int_field("irrigation", param.asInt());
}

/* ============================================================
 * HANDLERS MESH -> BLYNK
 * ============================================================ */

/**
 * @brief Atualiza Vpins de telemetria a partir de uma mensagem TELE.
 *
 * Mapeamento típico:
 * - t_out   → V0
 * - rh_out  → V1
 * - lux_out → V2
 * - t_in    → V3
 * - rh_in   → V4
 * - soil_moist → V5
 * - lux_in  → V6
 *
 * Somente campos com o respectivo @c has_* = true são enviados.
 *
 * @param msg Mensagem de telemetria (@ref MESH_MSG_TELE).
 */
static void handle_tele(const mesh_msg_t &msg)
{
    if (msg.tele.has_t_out)
    {
        Blynk.virtualWrite(0, msg.tele.t_out);
    }
    if (msg.tele.has_rh_out)
    {
        Blynk.virtualWrite(1, msg.tele.rh_out);
    }
    if (msg.tele.has_lux_out)
    {
        Blynk.virtualWrite(2, msg.tele.lux_out);
    }
    if (msg.tele.has_t_in)
    {
        Blynk.virtualWrite(3, msg.tele.t_in);
    }
    if (msg.tele.has_rh_in)
    {
        Blynk.virtualWrite(4, msg.tele.rh_in);
    }
    if (msg.tele.has_soil_moist)
    {
        Blynk.virtualWrite(5, msg.tele.soil_moist);
    }
    if (msg.tele.has_lux_in)
    {
        Blynk.virtualWrite(6, msg.tele.lux_in);
    }
}

/**
 * @brief Atualiza Vpins de estado de atuadores a partir de uma mensagem STATE.
 *
 * Mapeamento típico:
 * - intake_pwm  → V7
 * - exhaust_pwm → V8
 * - humidifier  → V9
 * - led_brig    → V10
 * - led_rgb     → V11
 * - irrigation  → V12
 *
 * @param msg Mensagem de estado (@ref MESH_MSG_STATE).
 */
static void handle_state(const mesh_msg_t &msg)
{
    if (msg.state.has_intake_pwm)
    {
        Blynk.virtualWrite(7, msg.state.intake_pwm);
    }
    if (msg.state.has_exhaust_pwm)
    {
        Blynk.virtualWrite(8, msg.state.exhaust_pwm);
    }
    if (msg.state.has_humidifier)
    {
        Blynk.virtualWrite(9, msg.state.humidifier);
    }
    if (msg.state.has_led_brig)
    {
        Blynk.virtualWrite(10, msg.state.led_brig);
    }
    if (msg.state.has_led_rgb)
    {
        Blynk.virtualWrite(11, msg.state.led_rgb);
    }
    if (msg.state.has_irrigation)
    {
        Blynk.virtualWrite(12, msg.state.irrigation);
    }
}

/**
 * @brief Atualiza Vpins de heartbeat a partir de uma mensagem HB.
 *
 * Usa o @c src da mensagem para decidir qual Vpin atualizar:
 * - msh-gw   → V20
 * - ext-sen → V21
 * - int-sen → V22
 * - act-00  → V23
 *
 * @param msg Mensagem de heartbeat (@ref MESH_MSG_HB).
 */
static void handle_hb(const mesh_msg_t &msg)
{
    if (!msg.hb.has_uptime_s)
    {
        return;
    }
    int up = msg.hb.uptime_s;
    int vpin = -1;

    if (!strcmp(msg.src, NODE_MSH_GW))
    {
        vpin = 20;
    }
    else if (!strcmp(msg.src, NODE_EXT_SEN))
    {
        vpin = 21;
    }
    else if (!strcmp(msg.src, NODE_INT_SEN))
    {
        vpin = 22;
    }
    else if (!strcmp(msg.src, NODE_ACT))
    {
        vpin = 23;
    }

    if (vpin >= 0)
    {
        Blynk.virtualWrite(vpin, up);
    }
}

/**
 * @brief Tratamento genérico de eventos EVT vindos da malha.
 *
 * No momento apenas imprime no Serial: origem, nome do evento,
 * código numérico e nível (se presentes).
 *
 * @param msg Mensagem de evento (@ref MESH_MSG_EVT).
 */
static void handle_evt(const mesh_msg_t &msg)
{
    Serial.print("[EVT] from=");
    Serial.print(msg.src);
    Serial.print(" event=");
    if (msg.evt.has_event)
    {
        Serial.print(msg.evt.event);
    }
    Serial.print(" code=");
    if (msg.evt.has_code)
    {
        Serial.print(msg.evt.code);
    }
    Serial.print(" level=");
    if (msg.evt.has_level)
    {
        Serial.print(msg.evt.level);
    }
    Serial.println();
}

/**
 * @brief Trata mensagens HELLO vindas da malha.
 *
 * - Loga no Serial o @c node_id e @c fw_ver recebidos (se presentes);
 * - Se o @c node_id for o mesh_gateway (@ref NODE_MSH_GW), marca
 *   @ref g_mesh_hello_ok = true e libera o processamento de
 *   TELE/STATE/HB/EVT.
 *
 * @param msg Mensagem HELLO (@ref MESH_MSG_HELLO).
 */
static void handle_hello(const mesh_msg_t &msg)
{
    Serial.print("[HELLO] from=");
    if (msg.hello.has_node_id)
    {
        Serial.print(msg.hello.node_id);
    }
    Serial.print(" fw=");
    if (msg.hello.has_fw_ver)
    {
        Serial.print(msg.hello.fw_ver);
    }
    Serial.println();

    /* Só libera depois do HELLO vindo do mesh_gateway (msh-gw) */
    if (msg.hello.has_node_id && strcmp(msg.hello.node_id, NODE_MSH_GW) == 0)
    {
        g_mesh_hello_ok = true;
        Serial.println("[HELLO] Mesh gateway registrado, liberando TELE/STATE/HB/EVT");
    }
}

/**
 * @brief Trata mensagens TIME recebidas da malha, aplicando sincronismo de relógio.
 *
 * Se houver @c epoch válido, chama @ref apply_time_from_mesh() com o offset
 * de fuso @c tz_offset_min (ou 0 se ausente).
 *
 * @param msg Mensagem de tempo (@ref MESH_MSG_TIME).
 */
static void handle_time(const mesh_msg_t &msg)
{
    if (!msg.time_sync.has_epoch)
    {
        Serial.println("[TIME] recebido sem epoch valido");
        return;
    }

    int tz = msg.time_sync.has_tz_offset_min ? msg.time_sync.tz_offset_min : 0;
    apply_time_from_mesh(msg.time_sync.epoch, tz);
}

/* ============================================================
 * API PÚBLICA
 * ============================================================ */

void blynk_client_init()
{
    g_mode = 0;
    g_msg_counter = 0;
    g_time_sync_ok = false;
    g_mesh_hello_ok = false;

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Conectando");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(250);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("[WiFi] Conectado, IP=");
    Serial.println(WiFi.localIP());

    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();

    /* Heartbeat simples do próprio blynk-gw em V20 */
    g_timer.setInterval(10000L, []()
                        {
        long up = millis() / 1000;
        Blynk.virtualWrite(20, up); });
}

void blynk_client_loop()
{
    Blynk.run();
    g_timer.run();
}

void blynk_client_handle_mesh_msg(const mesh_msg_t &msg)
{
    /* HELLO e TIME sempre passam (mesmo antes do HELLO do mesh_gateway) */
    if (msg.type == MESH_MSG_HELLO)
    {
        handle_hello(msg);
        return;
    }

    if (msg.type == MESH_MSG_TIME)
    {
        handle_time(msg);
        return;
    }

    /* TELE/STATE/HB/EVT só depois de receber HELLO do mesh_gateway */
    if (!g_mesh_hello_ok &&
        (msg.type == MESH_MSG_TELE ||
         msg.type == MESH_MSG_STATE ||
         msg.type == MESH_MSG_HB ||
         msg.type == MESH_MSG_EVT))
    {
        Serial.println("[MESH] Ignorando TELE/STATE/HB/EVT antes de HELLO do mesh_gateway");
        return;
    }

    switch (msg.type)
    {
    case MESH_MSG_TELE:
        handle_tele(msg);
        break;
    case MESH_MSG_STATE:
        handle_state(msg);
        break;
    case MESH_MSG_HB:
        handle_hb(msg);
        break;
    case MESH_MSG_EVT:
        handle_evt(msg);
        break;
    default:
        Serial.print("[MESH] tipo não tratado em blynk_client: ");
        Serial.println(msg.type);
        break;
    }
}
