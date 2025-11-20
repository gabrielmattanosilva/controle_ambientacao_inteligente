/**
 * @file ipc_uart.h
 * @brief
 */

#ifndef IPC_UART_H
#define IPC_UART_H

#include <Arduino.h>

/**
 * Inicializa a UART usada para o link IPC.
 */
void ipc_uart_begin(HardwareSerial *serial,
                    uint32_t baud,
                    int8_t tx_pin,
                    int8_t rx_pin);

/**
 * Lê uma mensagem completa da UART.
 *
 * Retorna:
 *   true  -> out_json contém o JSON (NUL-terminado)
 *   false -> nenhuma mensagem nova completa
 *
 * A função:
 *   - usa COBS para framing;
 *   - usa 0x00 como delimitador entre quadros;
 *   - valida CRC16-CCITT do payload decodificado;
 *   - retorna apenas o JSON puro em out_json.
 */
bool ipc_uart_read_json(char *out_json, size_t maxlen);

/**
 * Envia um JSON:
 *   payload = json_str (sem '\n')
 *   frame   = COBS( json_str + CRC16 ) + 0x00
 */
void ipc_uart_send_json(const char *json_str);

#endif /* IPC_UART_H */