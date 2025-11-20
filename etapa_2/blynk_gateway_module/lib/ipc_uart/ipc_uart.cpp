/**
 * @file ipc_uart.cpp
 * @brief
 */

#include "ipc_uart.h"

static HardwareSerial *g_uart = NULL;

/* buffer de entrada mais amplo */
static char   g_rx_buf[512];
static size_t g_rx_len = 0;

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ---------------- init ---------------- */

void ipc_uart_begin(HardwareSerial *serial, uint32_t baud, int8_t tx_pin, int8_t rx_pin)
{
    g_uart = serial;
    g_uart->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    g_rx_len = 0;
}

/* ---------------- envio ---------------- */

void ipc_uart_send_json(const char *json)
{
    if (!g_uart) return;

    uint16_t crc = crc16_ccitt((const uint8_t*)json, strlen(json));

    char frame[600];
    snprintf(frame, sizeof(frame), "%s*%04X\n", json, (unsigned)crc);

    g_uart->print(frame);
}

/* ---------------- recepção ---------------- */

bool ipc_uart_read_json(char *out_json, size_t maxlen)
{
    if (!g_uart) return false;

    while (g_uart->available()) {
        char c = g_uart->read();

        if (c == '\n') {
            /* fim da linha → tenta interpretar */
            g_rx_buf[g_rx_len] = '\0';

            char *star = strrchr(g_rx_buf, '*');
            if (!star) {
                g_rx_len = 0;
                return false; /* sem CRC */
            }

            size_t json_len = (size_t)(star - g_rx_buf);
            if (json_len >= maxlen) {
                g_rx_len = 0;
                return false;
            }

            char json_only[512];
            memcpy(json_only, g_rx_buf, json_len);
            json_only[json_len] = '\0';

            unsigned crc_rx = 0;
            if (sscanf(star + 1, "%4X", &crc_rx) != 1) {
                g_rx_len = 0;
                return false; /* CRC inválido */
            }

            uint16_t crc_calc = crc16_ccitt((uint8_t*)json_only, json_len);
            if (crc_calc != crc_rx) {
                g_rx_len = 0;
                return false; /* CRC errado */
            }

            /* JSON válido, copia */
            strcpy(out_json, json_only);
            g_rx_len = 0;
            return true;
        }

        if (c != '\r') {
            if (g_rx_len < sizeof(g_rx_buf) - 1) {
                g_rx_buf[g_rx_len++] = c;
            }
        }
    }

    return false;
}