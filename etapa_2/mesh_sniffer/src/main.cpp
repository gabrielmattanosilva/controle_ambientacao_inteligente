/**
 * @file main.cpp
 * @brief Nó dummy de recepção da mesh (sniffer)
 *
 * - Entra na rede mesh (painlessMesh)
 * - Recebe todas as mensagens JSON
 * - Usa mesh_proto_parse() para decodificar
 * - Imprime no Serial
 * - GERA ACKs automáticos para mensagens QoS1
 */

#include <Arduino.h>
#include <painlessMesh.h>
#include "credentials.h"
#include "mesh_proto.h"

// -----------------------------------------------------------------------------
// Identificação local
// -----------------------------------------------------------------------------
#define NODE_NAME   "mesh-sniffer"

Scheduler userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Callback usado pela lib mesh_proto para enviar mensagens QoS (ACK, EVT, ...)
//
// A lib mesh_proto SEMPRE envia para o "send_cb" registrado em
// mesh_proto_qos_init(), então precisamos definir esse callback aqui.
// -----------------------------------------------------------------------------
static void mesh_send_json_cb(const char *json)
{
    mesh.sendBroadcast(json);
    Serial.print("[MESH TX] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// Funções de debug: impressão amigável
// -----------------------------------------------------------------------------

static void print_tele(const mesh_msg_t &m)
{
    Serial.println("  [TELE]");
    if (m.tele.has_t_out)   Serial.printf("   t_out   = %.2f\n", m.tele.t_out);
    if (m.tele.has_rh_out)  Serial.printf("   rh_out  = %.2f\n", m.tele.rh_out);
    if (m.tele.has_lux_out) Serial.printf("   lux_out = %d\n",   m.tele.lux_out);

    if (m.tele.has_t_in)       Serial.printf("   t_in       = %.2f\n", m.tele.t_in);
    if (m.tele.has_rh_in)      Serial.printf("   rh_in      = %.2f\n", m.tele.rh_in);
    if (m.tele.has_soil_moist) Serial.printf("   soil_moist = %d\n",   m.tele.soil_moist);
    if (m.tele.has_lux_in)     Serial.printf("   lux_in     = %d\n",   m.tele.lux_in);
}

static void print_state(const mesh_msg_t &m)
{
    Serial.println("  [STATE]");
    if (m.state.has_intake_pwm)  Serial.printf("   intake_pwm  = %d\n", m.state.intake_pwm);
    if (m.state.has_exhaust_pwm) Serial.printf("   exhaust_pwm = %d\n", m.state.exhaust_pwm);
    if (m.state.has_humidifier)  Serial.printf("   humidifier  = %d\n", m.state.humidifier);
    if (m.state.has_irrigation)  Serial.printf("   irrigation  = %d\n", m.state.irrigation);
    if (m.state.has_led_brig)    Serial.printf("   led_brig    = %d\n", m.state.led_brig);
    if (m.state.has_led_rgb)     Serial.printf("   led_rgb     = %s\n", m.state.led_rgb);
}

static void print_cfg(const mesh_msg_t &m)
{
    Serial.println("  [CFG]");
    if (m.cfg.has_mode)         Serial.printf("   mode        = %d\n", m.cfg.mode);
    if (m.cfg.has_intake_pwm)   Serial.printf("   intake_pwm  = %d\n", m.cfg.intake_pwm);
    if (m.cfg.has_exhaust_pwm)  Serial.printf("   exhaust_pwm = %d\n", m.cfg.exhaust_pwm);
    if (m.cfg.has_humidifier)   Serial.printf("   humidifier  = %d\n", m.cfg.humidifier);
    if (m.cfg.has_irrigation)   Serial.printf("   irrigation  = %d\n", m.cfg.irrigation);
    if (m.cfg.has_led_pwm)      Serial.printf("   led_pwm     = %d\n", m.cfg.led_pwm);
    if (m.cfg.has_led_rgb)      Serial.printf("   led_rgb     = %s\n", m.cfg.led_rgb);
}

static void print_hb(const mesh_msg_t &m)
{
    Serial.println("  [HB]");
    if (m.hb.has_uptime_s) Serial.printf("   uptime_s = %d\n", m.hb.uptime_s);
    if (m.hb.has_rssi_dbm) Serial.printf("   rssi_dbm = %d\n", m.hb.rssi_dbm);
}

static void print_evt(const mesh_msg_t &m)
{
    Serial.println("  [EVT]");
    if (m.evt.has_event) Serial.printf("   event = %s\n", m.evt.event);
    if (m.evt.has_code)  Serial.printf("   code  = %d\n", m.evt.code);
    if (m.evt.has_level) Serial.printf("   level = %d\n", m.evt.level);
}

static void print_hello(const mesh_msg_t &m)
{
    Serial.println("  [HELLO]");
    if (m.hello.has_node_id) Serial.printf("   node_id = %s\n", m.hello.node_id);
    if (m.hello.has_fw_ver)  Serial.printf("   fw_ver  = %s\n", m.hello.fw_ver);
    if (m.hello.has_extra)   Serial.printf("   extra   = %s\n", m.hello.extra);
}

static void print_ack(const mesh_msg_t &m)
{
    Serial.println("  [ACK]");
    if (m.ack.has_ref)    Serial.printf("   ref    = %s\n", m.ack.ref);
    if (m.ack.has_status) Serial.printf("   status = %s\n", m.ack.status);
}

static void print_time(const mesh_msg_t &m)
{
    Serial.println("  [TIME]");
    if (m.time_sync.has_epoch)
        Serial.printf("   epoch = %lu\n", (unsigned long)m.time_sync.epoch);
    if (m.time_sync.has_tz_offset_min)
        Serial.printf("   tz_offset_min = %d\n", m.time_sync.tz_offset_min);
}

// -----------------------------------------------------------------------------
// Handler principal
// -----------------------------------------------------------------------------

static void handle_mesh_json(const char *json)
{
    Serial.println("--------------------------------------------------");
    Serial.print("[RX RAW] ");
    Serial.println(json);

    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.println("[PARSE] ERRO");
        return;
    }

    Serial.println("[PARSE] Decodificada:");
    Serial.printf("  id=%s ts=%lu qos=%u src=%s dst=%s\n",
        msg.id, (unsigned long)msg.ts, msg.qos, msg.src, msg.dst);

    // ---------------------------------------------------------------------
    // ✔ NOVO: se for QoS1 → gerar ACK usando lib mesh_proto
    // ---------------------------------------------------------------------
    if (msg.qos == 1) {
        Serial.println("[ACK] Enviando ACK QoS1...");
        mesh_proto_qos_send_ack_ok(&msg);   // ← ACK oficial
    }

    // ---------------------------------------------------------------------
    // Impressão detalhada
    // ---------------------------------------------------------------------
    switch (msg.type) {
        case MESH_MSG_TELE:  Serial.println("  type=TELE");  print_tele(msg);  break;
        case MESH_MSG_STATE: Serial.println("  type=STATE"); print_state(msg); break;
        case MESH_MSG_CFG:   Serial.println("  type=CFG");   print_cfg(msg);   break;
        case MESH_MSG_HB:    Serial.println("  type=HB");    print_hb(msg);    break;
        case MESH_MSG_EVT:   Serial.println("  type=EVT");   print_evt(msg);   break;
        case MESH_MSG_HELLO: Serial.println("  type=HELLO"); print_hello(msg); break;
        case MESH_MSG_ACK:   Serial.println("  type=ACK");   print_ack(msg);   break;
        case MESH_MSG_TIME:  Serial.println("  type=TIME");  print_time(msg);  break;
        default:
            Serial.println("  type=UNKNOWN");
    }

    Serial.println("--------------------------------------------------");
}

// -----------------------------------------------------------------------------
// Callbacks mesh
// -----------------------------------------------------------------------------

void mesh_receive_cb(uint32_t from, String &msg)
{
    Serial.printf("[MESH RX] from=%u len=%u\n",
                  (unsigned)from,
                  (unsigned)msg.length());
    handle_mesh_json(msg.c_str());
}

void mesh_new_connection_cb(uint32_t nodeId)
{
    Serial.printf("[MESH] New connection: %u\n", (unsigned)nodeId);
}

void mesh_changed_connections_cb()
{
    Serial.println("[MESH] Changed connections");
}

void mesh_time_adjusted_cb(int32_t offset)
{
    Serial.printf("[MESH] Time adjusted offset=%ld\n", (long)offset);
}

// -----------------------------------------------------------------------------
// setup / loop
// -----------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println("==============================================");
    Serial.println("[mesh-sniffer] Boot");
    Serial.println("Sniffando toda a rede mesh...");
    Serial.println("==============================================");

    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    // ✔ importante para permitir que o sniffer envie ACKs da lib
    mesh_proto_qos_init(mesh_send_json_cb);

    Serial.println("[mesh-sniffer] Mesh inicializada.");
}

void loop()
{
    mesh.update();

    // QoS manager da lib (tratamento de retries)
    mesh_proto_qos_poll();
}
