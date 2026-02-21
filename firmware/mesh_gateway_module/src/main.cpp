/**
 * @file main.cpp
 * @brief Inicialização básica (logger, SD, RTC, UART) e delegação
 *        da lógica de gateway para lib/mesh_gateway.
 */

#include <Arduino.h>

#include "pins.h"
#include "ds1307_rtc.h"
#include "logger.h"
#include "sd_card.h"
#include "ipc_uart.h"
#include "mesh_gateway.h"

static const char *TAG = "MAIN";

void setup()
{
    // 1) Inicializa sistema de logs (RTC interno em epoch0 + SD)
    logger_init_epoch0();
    sdcard_begin();
    logger_begin();

    LOG(TAG, "[mesh_gateway_module] boot");

    // 2) Tenta sincronizar o RTC interno a partir do DS1307
    if (ds1307_rtc_sync_at_boot())
    {
        LOG(TAG, "RTC interno sincronizado a partir do DS1307");
    }
    else
    {
        LOG(TAG, "DS1307 ausente/invalido/parado; mantendo epoch0 ate ter hora valida");
    }

    // 3) Inicializa link UART (para blynk_gateway_module)
    ipc_uart_begin(&Serial2, 115200, UART_TX_PIN, UART_RX_PIN);
    LOG(TAG, "UART IPC inicializada (TX=%d, RX=%d)", UART_TX_PIN, UART_RX_PIN);

    // 4) Inicializa gateway da malha (painlessMesh + ponte UART<->mesh + QoS)
    mesh_gateway_init();
}

void loop()
{
    // Rotação diária do arquivo de log
    sdcard_tick_rotate();

    // Lógica do gateway (mesh.update, UART, QoS, etc.)
    mesh_gateway_loop();
}
