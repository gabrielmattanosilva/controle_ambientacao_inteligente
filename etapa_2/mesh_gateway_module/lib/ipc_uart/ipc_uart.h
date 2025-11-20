/**
 * @file ipc_uart.h
 * @brief
 */

#ifndef IPC_UART_H
#define IPC_UART_H

#include <Arduino.h>

void ipc_uart_begin(HardwareSerial *serial, uint32_t baud, int8_t tx_pin, int8_t rx_pin);

/* Retorna:
 *   true  → linha válida (JSON sem CRC)
 *   false → nada novo
 * out_json recebe apenas o JSON, sem "*CRC"
 */
bool ipc_uart_read_json(char *out_json, size_t maxlen);

/* Envia json + "*" + CRC + "\n" */
void ipc_uart_send_json(const char *json_str);

#endif /* IPC_UART_H */