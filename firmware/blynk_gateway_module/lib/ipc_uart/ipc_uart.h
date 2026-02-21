/**
 * @file ipc_uart.h
 * @brief Interface pública do módulo de comunicação IPC via UART.
 *
 * Esta biblioteca implementa um canal de comunicação robusto entre módulos,
 * utilizando UART com:
 *
 * - **COBS** (Consistent Overhead Byte Stuffing) para framing sem ambiguidades;
 * - **Delimitador 0x00** entre quadros codificados;
 * - **CRC16-CCITT (0xFFFF, polinômio 0x1021)** para validação de integridade;
 * - Envio e recepção de **strings JSON completas**, já validadas e prontas
 *   para processamento pelas camadas superiores (ex.: mesh_proto).
 *
 * Este arquivo contém:
 * - Funções de inicialização da UART para uso IPC;
 * - Funções para envio de JSON (com CRC + COBS);
 * - Função para leitura de quadros completos, com validação automática.
 */

#ifndef IPC_UART_H
#define IPC_UART_H

#include <Arduino.h>

/**
 * @brief Inicializa a UART usada para o link IPC.
 *
 * @param serial  Ponteiro para a instância de HardwareSerial a ser usada.
 * @param baud    Baudrate a ser configurado.
 * @param tx_pin  Pino TX da UART.
 * @param rx_pin  Pino RX da UART.
 */
void ipc_uart_begin(HardwareSerial *serial,
                    uint32_t baud,
                    int8_t tx_pin,
                    int8_t rx_pin);

/**
 * @brief Lê uma mensagem JSON completa transmitida via UART.
 *
 * Retornos:
 * - **true**  → out_json contém um JSON completo e válido (NUL-terminado);
 * - **false** → nenhuma mensagem completa disponível.
 *
 * A função realiza:
 * - Armazenamento do buffer bruto recebido;
 * - Decodificação COBS;
 * - Remoção do delimitador de quadro (0x00);
 * - Validação CRC16-CCITT;
 * - Retorno do JSON puro no buffer de saída.
 *
 * @param out_json Buffer de saída para o JSON já validado.
 * @param maxlen   Tamanho máximo do buffer out_json.
 *
 * @return true se um JSON completo foi recebido e validado.
 */
bool ipc_uart_read_json(char *out_json, size_t maxlen);

/**
 * @brief Envia uma string JSON via UART usando COBS + CRC + delimitador 0x00.
 *
 * Fluxo:
 * - payload = JSON + CRC16
 * - frame   = COBS(payload) + 0x00
 *
 * @param json_str String JSON a ser transmitida.
 */
void ipc_uart_send_json(const char *json_str);

#endif /* IPC_UART_H */
