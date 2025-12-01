/**
 * @file main.cpp
 * @brief Nó simulador int-sen-00
 *
 * Este firmware roda em um ESP32 separado e simula 3 nós lógicos:
 *   - "int-sen-00"  → sensores internos (t_in, rh_in, soil_moist, lux_in)
 *   - "ext-sen-00"  → sensores externos (t_out, rh_out, lux_out)
 *   - "act-00"      → atuadores (intake_pwm, exhaust_pwm, humidifier, led_brig/led_rgb, irrigation)
 *
 * Toda comunicação é feita via rede mesh (painlessMesh + mesh_proto),
 * conversando com o mesh_gateway_module, que faz a ponte até o Blynk.
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include "credentials.h"
#include "mesh_proto.h"

// ------------------- NOVO: BH1750 -------------------
#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;
bool bh1750_ok = false;
// ----------------------------------------------------

// -----------------------------------------------------------------------------
// Identificadores lógicos de nós
// -----------------------------------------------------------------------------
#define NODE_INT        "int-sen-00"
#define NODE_EXT        "ext-sen-00"
#define NODE_ACT        "act-00"
#define NODE_BLYNK_GW   "blynk-gw"
#define NODE_MSH_GW     "msh-gw"

// Escolha aqui quem é o "sink" principal das mensagens simuladas.
// Se ainda estiver usando o blynk-gw direto na mesh, troque para NODE_BLYNK_GW.
#define NODE_SINK       NODE_BLYNK_GW

Scheduler userScheduler;
static painlessMesh mesh;

// -----------------------------------------------------------------------------
// Estado simulado dos atuadores (act-00)
// -----------------------------------------------------------------------------
static int  g_mode        = 0;        // 0=AUTO, 1=MANUAL, etc.
static int  g_intake_pwm  = 0;
static int  g_exhaust_pwm = 0;
static int  g_humidifier  = 0;
static int  g_irrigation  = 0;
static int  g_led_pwm     = 0;        // usado como "led_brig"
static char g_led_rgb[16] = "0x000000";

// -----------------------------------------------------------------------------
// Estado simulado dos sensores (interno/externo)
// -----------------------------------------------------------------------------
static float g_t_in       = 25.0f;
static float g_rh_in      = 60.0f;
static int   g_soil_moist = 60;
static int   g_lux_in     = 5000;     // agora passa a ser alimentado pelo BH1750 quando disponível

static float g_t_out      = 27.0f;
static float g_rh_out     = 55.0f;
static int   g_lux_out    = 20000;

// -----------------------------------------------------------------------------
// Controle de envio / contadores
// -----------------------------------------------------------------------------
static uint16_t      g_msg_counter   = 0;
static unsigned long g_last_tele     = 0;
static unsigned long g_last_hb_all   = 0;
static unsigned long g_last_state    = 0;

// Novo: controla se já mandamos HELLO depois de conectar na mesh
static bool          g_hello_sent    = false;


// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

static void gen_msg_id(char *buf, size_t len)
{
    g_msg_counter++;
    if (!g_msg_counter) g_msg_counter = 1;
    snprintf(buf, len, "%04X", g_msg_counter);
}

static float clampf(float v, float vmin, float vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

static int clampi(int v, int vmin, int vmax)
{
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

/**
 * Callback usado pela lib mesh_proto para enviar JSONs QoS1 (ACK, EVT, etc.)
 * via rede mesh.
 */
static void mesh_send_json_cb(const char *json)
{
    // Por simplicidade usamos broadcast; o campo "dst" no JSON
    // é o que define quem deve processar a mensagem.
    mesh.sendBroadcast(json);
    Serial.print("[MESH TX] ");
    Serial.println(json);
}

// -----------------------------------------------------------------------------
// NOVO: Leitura de luminosidade interna (BH1750 + fallback mock)
// -----------------------------------------------------------------------------

static int read_lux_in()
{
    if (bh1750_ok) {
        float lux = lightMeter.readLightLevel(); // unidade em lux

        if (lux < 0.0f || lux > 120000.0f) {
            // leitura fora da faixa razoável, mantém último valor conhecido
            return g_lux_in;
        }

        int lux_i = (int)lux;
        g_lux_in = clampi(lux_i, 0, 40000);  // mantém mesma faixa usada antes
        return g_lux_in;
    }

    // Fallback: se BH1750 não inicializou, mantém mock antigo para não quebrar nada
    g_lux_in += random(-2500, 2501);           // ±2500 lux
    g_lux_in  = clampi(g_lux_in, 0, 40000);
    return g_lux_in;
}

// -----------------------------------------------------------------------------
// Simulação de ambiente e atuadores
// -----------------------------------------------------------------------------

/**
 * Atualiza suavemente os valores simulados de sensores.
 * A ideia é ter uma evolução "lenta", em vez de saltos totalmente aleatórios.
 *
 * OBS: lux_in agora é obtido por read_lux_in(), então não é mais
 *      atualizado diretamente aqui.
 */
static void simulate_sensors_step()
{
    // Ambiente externo
    g_t_out  += (float)random(-5, 6) / 10.0f;   // ±0.5 °C
    g_rh_out += (float)random(-8, 9) / 10.0f;   // ±0.8 %
    g_lux_out += random(-3000, 3001);           // ±3000 lux

    g_t_out   = clampf(g_t_out,   18.0f, 36.0f);
    g_rh_out  = clampf(g_rh_out,  30.0f, 90.0f);
    g_lux_out = clampi(g_lux_out,     0, 60000);

    // Ambiente interno "segue" algo próximo do externo, com ajustes
    g_t_in  += (float)random(-4, 5) / 10.0f;    // ±0.4 °C
    g_rh_in += (float)random(-7, 8) / 10.0f;    // ±0.7 %

    g_t_in   = clampf(g_t_in,   20.0f, 34.0f);
    g_rh_in  = clampf(g_rh_in,  35.0f, 95.0f);

    g_soil_moist += random(-3, 4);              // ±3 %
    g_soil_moist  = clampi(g_soil_moist, 20, 100);

    // g_lux_in agora é lido por read_lux_in(), então não mexemos nele aqui
}

/**
 * Se o modo for AUTO, ajusta atuadores em função das variáveis internas.
 * Isso deixa a simulação do act-00 mais condizente com o algoritmo
 * que você pretende testar (ventilação, umidificação, irrigação, etc.).
 */
static void simulate_actuators_auto()
{
    if (g_mode != 0) {
        // Em modo manual não mexemos nos atuadores aqui;
        // eles são comandados via CFG.
        return;
    }

    // Controle simples de temperatura (ventiladores)
    if (g_t_in > 28.0f) {
        g_intake_pwm  = 200;
        g_exhaust_pwm = 200;
    } else if (g_t_in < 24.0f) {
        g_intake_pwm  = 0;
        g_exhaust_pwm = 0;
    } else {
        g_intake_pwm  = 100;
        g_exhaust_pwm = 100;
    }

    // Controle simples de umidade (umidificador)
    if (g_rh_in < 50.0f) {
        g_humidifier = 1;
    } else if (g_rh_in > 70.0f) {
        g_humidifier = 0;
    }

    // Controle simples de irrigação pelo solo
    if (g_soil_moist < 40) {
        g_irrigation = 1;
    } else if (g_soil_moist > 70) {
        g_irrigation = 0;
    }

    // Iluminação interna proporcional ao "lux_in"
    // (exemplo simples só pra variar um pouco)
    int lux_in_now = read_lux_in(); // garante que g_lux_in esteja atualizado

    if (lux_in_now < 2000) {
        g_led_pwm = 255;
    } else if (lux_in_now > 10000) {
        g_led_pwm = 0;
    } else {
        // faixa intermediária: escala linear simples
        float factor = (10000.0f - lux_in_now) / 8000.0f; // 1.0 → 0.0
        int pwm = (int)(factor * 255.0f);
        g_led_pwm = clampi(pwm, 0, 255);
    }

    // Cor fixa só pra ter algo no campo led_rgb
    strncpy(g_led_rgb, "0x00FFAA", sizeof(g_led_rgb) - 1);
}

// -----------------------------------------------------------------------------
// Envio de TELE / STATE / HB / HELLO
// -----------------------------------------------------------------------------

/**
 * Envia um "dump" de telemetria:
 *   - TELE de ext-sen-00
 *   - TELE de int-sen-00
 */
static void send_tele_dump()
{
    JsonDocument doc;
    JsonObject   data;
    char id[8];
    char json[256];

    // Atualiza ambiente antes de enviar
    simulate_sensors_step();
    simulate_actuators_auto();

    // --- TELE externa (ext-sen-00) ---
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_EXT;
    doc["dst"]  = NODE_SINK;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_out"]   = g_t_out;
    data["rh_out"]  = g_rh_out;
    data["lux_out"] = g_lux_out;

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);

    // --- TELE interna (int-sen-00) ---
    gen_msg_id(id, sizeof(id));
    doc.clear();
    doc["id"]   = id;
    doc["ts"]   = (uint32_t)millis();
    doc["qos"]  = 0;
    doc["src"]  = NODE_INT;
    doc["dst"]  = NODE_SINK;
    doc["type"] = "tele";

    data = doc["data"].to<JsonObject>();
    data["t_in"]       = g_t_in;
    data["rh_in"]      = g_rh_in;
    data["soil_moist"] = g_soil_moist;
    data["lux_in"]     = read_lux_in();  // <<< AGORA VEM DO BH1750 (ou mock fallback)

    serializeJson(doc, json, sizeof(json));
    mesh_send_json_cb(json);
}

/**
 * Envia STATE atual dos atuadores (act-00).
 * Usamos o builder da lib mesh_proto para manter o formato consistente.
 */
static void send_state_now()
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_state_act(id,
                                    (uint32_t)millis(),
                                    0,              // QoS0
                                    NODE_ACT,
                                    NODE_SINK,
                                    g_intake_pwm,
                                    g_exhaust_pwm,
                                    g_humidifier,
                                    g_led_pwm,
                                    g_led_rgb,
                                    g_irrigation,
                                    json,
                                    sizeof(json))) {
        return;
    }

    mesh_send_json_cb(json);
}

/**
 * Envia heartbeat de um nó lógico.
 */
static void send_hb_for(const char *node_id, int rssi_dbm)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hb(id,
                             (uint32_t)millis(),
                             node_id,
                             NODE_SINK,
                             (int)(millis() / 1000),
                             rssi_dbm,
                             json,
                             sizeof(json))) {
        return;
    }

    mesh_send_json_cb(json);
}

/**
 * Envia heartbeat de TODOS os nós simulados:
 *   - int-sen-00
 *   - ext-sen-00
 *   - act-00
 */
static void send_hb_all()
{
    // Dá uma variada leve no RSSI só pra logs ficarem mais interessantes
    int base_rssi = -60;
    send_hb_for(NODE_INT, base_rssi + random(-3, 4));
    send_hb_for(NODE_EXT, base_rssi + random(-3, 4));
    send_hb_for(NODE_ACT, base_rssi + random(-3, 4));
}

/**
 * Envia HELLO de um nó lógico (apresentação ao mesh_gateway).
 * Agora:
 *   - qos=1
 *   - dst="msh-gw" (NODE_MSH_GW)
 *   - usa o gerenciador de QoS (mesh_proto_qos_register_and_send)
 */
static void send_hello_for(const char *node_id,
                           const char *fw_ver,
                           const char *extra)
{
    char id[8];
    char json[256];

    gen_msg_id(id, sizeof(id));

    if (!mesh_proto_build_hello(id,
                                (uint32_t)millis(),
                                1,              // QoS1
                                node_id,        // src lógico
                                NODE_MSH_GW,    // dst = mesh-gw
                                node_id,
                                fw_ver,
                                extra,
                                json,
                                sizeof(json))) {
        return;
    }

    // Tenta registrar no QoS (para ter retry automático).
    // Se não houver slot, faz fallback para envio direto.
    if (!mesh_proto_qos_register_and_send(id, json)) {
        mesh_send_json_cb(json);
    }
}

/**
 * Envia HELLO de todos os nós simulados.
 * Chamado uma vez após a mesh estar inicializada.
 */
static void send_hello_all()
{
    send_hello_for(NODE_INT, "1.0.0-sim", "internal sensors");
    send_hello_for(NODE_EXT, "1.0.0-sim", "external sensors");
    send_hello_for(NODE_ACT, "1.0.0-sim", "actuators");
}

// -----------------------------------------------------------------------------
// Aplicar CFG recebido (comportamento de "act-00")
// -----------------------------------------------------------------------------

static void apply_cfg(const mesh_msg_t &msg)
{
    // 1) Atualiza o modo (V13 -> cfg.mode)
    if (msg.cfg.has_mode) {
        g_mode = msg.cfg.mode;
        Serial.printf("[MODE] Agora em %s\n",
                      (g_mode == 1) ? "MANUAL (1)" : "AUTO (0)");
    }

    bool manual = (g_mode == 1);

    // 2) No mock Python, só aplica comandos de atuador se estiver em modo MANUAL
    if (manual) {
        if (msg.cfg.has_intake_pwm)
            g_intake_pwm = msg.cfg.intake_pwm;

        if (msg.cfg.has_exhaust_pwm)
            g_exhaust_pwm = msg.cfg.exhaust_pwm;

        if (msg.cfg.has_humidifier)
            g_humidifier = msg.cfg.humidifier;

        if (msg.cfg.has_irrigation)
            g_irrigation = msg.cfg.irrigation;

        if (msg.cfg.has_led_pwm)
            g_led_pwm = msg.cfg.led_pwm;

        if (msg.cfg.has_led_rgb)
            strncpy(g_led_rgb, msg.cfg.led_rgb, sizeof(g_led_rgb) - 1);
    } else {
        // Em AUTO, ignoramos campos de atuador em CFG
        // (equivalente ao mock: V7-V12 NÃO seguem V14-V19 quando V13=0)
    }

    Serial.printf("[CFG] mode=%d, intake=%d, exhaust=%d, hum=%d, irr=%d, led_pwm=%d, led_rgb=%s\n",
                  g_mode, g_intake_pwm, g_exhaust_pwm,
                  g_humidifier, g_irrigation, g_led_pwm, g_led_rgb);

    // 3) Envia STATE imediato para refletir o estado atual dos atuadores
    send_state_now();

    // 4) Se o CFG veio com QoS1, manda ACK "ok" de volta (via lib mesh_proto)
    mesh_proto_qos_send_ack_ok(&msg);
}

// -----------------------------------------------------------------------------
// Handlers de mensagens da mesh
// -----------------------------------------------------------------------------

static void handle_mesh_json(const char *json)
{
    mesh_msg_t msg;
    if (!mesh_proto_parse(json, &msg)) {
        Serial.print("[RX] parse fail: ");
        Serial.println(json ? json : "(null)");
        return;
    }

    // 1) ACK: SEMPRE avisar o gerenciador de QoS, independente do dst lógico
    if (msg.type == MESH_MSG_ACK) {
        mesh_proto_qos_on_ack(&msg);
        Serial.printf("[RX ACK] ref=%s from=%s dst=%s\n",
                      msg.ack.ref, msg.src, msg.dst);
        return;  // não precisa fazer mais nada com o ACK
    }

    // 2) Demais tipos: aplica filtro de destino
    if (strcmp(msg.dst, NODE_MSH_GW) != 0 &&
        strcmp(msg.dst, NODE_ACT)    != 0 &&
        strcmp(msg.dst, NODE_INT)    != 0 &&
        strcmp(msg.dst, NODE_EXT)    != 0 &&
        strcmp(msg.dst, "*")         != 0) {
        Serial.printf("[RX] ignorado dst=%s\n", msg.dst);
        return;
    }

    switch (msg.type) {
    case MESH_MSG_CFG:
        apply_cfg(msg);
        break;

    case MESH_MSG_TIME:
        Serial.println("[TIME] recebido (simulado, não aplicado)");
        break;

    default:
        Serial.printf("[RX] tipo não tratado=%d\n", msg.type);
        break;
    }
}


// -----------------------------------------------------------------------------
// Callbacks da painlessMesh
// -----------------------------------------------------------------------------

void mesh_receive_cb(uint32_t from, String &msg)
{
    Serial.printf("[MESH RX] from %u: %s\n", from, msg.c_str());
    handle_mesh_json(msg.c_str());
}

void mesh_new_connection_cb(uint32_t nodeId)
{
    Serial.printf("[MESH] New connection, nodeId=%u\n", nodeId);

    // Só envia HELLO uma vez, na primeira conexao com a rede mesh
    if (!g_hello_sent) {
        g_hello_sent = true;
        Serial.println("[MESH] Primeira conexao estabelecida, enviando HELLO QoS1 de todos os nos simulados...");
        send_hello_all();
        send_state_now();
    }
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
    delay(500);
    Serial.println("[int-sen-00-sim] boot");

    randomSeed(esp_random());

    // Inicializa rede mesh
    mesh.setDebugMsgTypes(ERROR | STARTUP);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
    mesh.onReceive(&mesh_receive_cb);
    mesh.onNewConnection(&mesh_new_connection_cb);
    mesh.onChangedConnections(&mesh_changed_connections_cb);
    mesh.onNodeTimeAdjusted(&mesh_time_adjusted_cb);

    // Inicializa QoS da lib (usado para enviar ACK de CFG e, se desejar, EVT QoS1)
    mesh_proto_qos_init(mesh_send_json_cb);

    // Estado inicial do ambiente (aleatoriza um pouco em torno dos defaults)
    g_t_in       += (float)random(-10, 11) / 10.0f;
    g_rh_in      += (float)random(-15, 16) / 10.0f;
    g_soil_moist += random(-10, 11);
    g_lux_in     += random(-3000, 3001);

    g_t_out      += (float)random(-10, 11) / 10.0f;
    g_rh_out     += (float)random(-15, 16) / 10.0f;
    g_lux_out    += random(-5000, 5001);

    g_soil_moist = clampi(g_soil_moist, 20, 100);
    g_lux_in     = clampi(g_lux_in, 0, 40000);
    g_lux_out    = clampi(g_lux_out, 0, 60000);

    // ------------------- NOVO: inicialização do BH1750 -------------------
    Wire.begin(21, 22); // SDA, SCL — ajuste se estiver usando outros pinos
    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
        bh1750_ok = true;
        Serial.println("[BH1750] Inicializado com sucesso");
    } else {
        bh1750_ok = false;
        Serial.println("[BH1750] Falha na inicializacao — utilizando mock para lux_in");
    }
    // ---------------------------------------------------------------------

    Serial.println("[int-sen-00-sim] mesh inicializada");
}

void loop()
{
    mesh.update();

    unsigned long now = millis();

    // TELE periódica de sensores (int + ext)
    if (now - g_last_tele >= 60000UL) {   // 60 s
        g_last_tele = now;
        send_tele_dump();
    }

    // STATE periódico dos atuadores (além do enviado em resposta ao CFG)
    if (now - g_last_state >= 30000UL) {  // 30 s
        g_last_state = now;
        send_state_now();
    }

    // Heartbeat de todos os nós simulados
    if (now - g_last_hb_all >= 10000UL) { // 10 s
        g_last_hb_all = now;
        send_hb_all();
    }

    // Gerenciador de QoS (retry de mensagens QoS1 que você venha a enviar)
    mesh_proto_qos_poll();
}
