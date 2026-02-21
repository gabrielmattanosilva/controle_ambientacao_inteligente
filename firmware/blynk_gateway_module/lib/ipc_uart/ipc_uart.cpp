/**
 * @file ipc_uart.cpp
 * @brief Implementação do canal de comunicação IPC via UART com framing COBS e CRC16.
 *
 * Este arquivo contém:
 * - Implementação do algoritmo CRC16-CCITT (0xFFFF, poly 0x1021);
 * - Implementação dos codificadores COBS encode/decode;
 * - Buffer de recepção para armazenar quadros até o delimitador 0x00;
 * - Decodificação COBS e validação CRC16;
 * - Extração do JSON limpo para camada superior;
 * - Função de envio que monta o pacote: JSON + CRC → COBS → envio + 0x00.
 *
 * Backend físico (transporte UART) para protocolos como mesh_proto.
 */

#include "ipc_uart.h"
#include <Arduino.h>

/* ============================================================
 * CRC16-CCITT (0xFFFF, poly 0x1021)
 * ============================================================ */

/**
 * @brief Calcula o CRC16-CCITT de um buffer.
 *
 * @param data Ponteiro para os dados de entrada.
 * @param len  Tamanho do buffer em bytes.
 *
 * @return Valor CRC16 calculado.
 */
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ============================================================
 * COBS ENCODE/DECODE
 * ============================================================ */

/**
 * @brief Codifica um buffer usando o algoritmo COBS.
 *
 * @param input    Ponteiro para o buffer de entrada.
 * @param length   Tamanho do buffer de entrada.
 * @param output   Ponteiro para o buffer de saída.
 * @param out_size Tamanho máximo do buffer de saída.
 *
 * @return Tamanho do frame codificado em bytes, ou 0 em caso de erro
 *         (parâmetros inválidos ou estouro de @p output).
 */
static size_t cobs_encode(const uint8_t *input, size_t length,
                          uint8_t *output, size_t out_size)
{
    if (!input || !output || out_size == 0)
        return 0;

    const uint8_t *in = input;
    const uint8_t *end = input + length;

    uint8_t *code_ptr = output; /* Posição onde será escrito o "code" */
    uint8_t *out = output + 1;  /* Onde começam os dados */
    uint8_t code = 1;

    while (in < end)
    {
        if (*in == 0)
        {
            *code_ptr = code;
            code_ptr = out++;
            code = 1;
        }
        else
        {
            if ((size_t)(out - output) >= out_size)
                return 0; /* Overflow */

            *out++ = *in;
            code++;

            if (code == 0xFF)
            {
                *code_ptr = code;
                code_ptr = out++;
                code = 1;
            }
        }
        in++;
    }

    *code_ptr = code;
    return (size_t)(out - output);
}

/**
 * @brief Decodifica um frame COBS.
 *
 * @param input    Ponteiro para o frame COBS de entrada.
 * @param length   Tamanho do frame de entrada.
 * @param output   Ponteiro para o buffer de saída decodificado.
 * @param out_size Tamanho máximo do buffer de saída.
 *
 * @return Tamanho do buffer decodificado em bytes, ou 0 em caso de erro
 *         (frame inválido ou estouro de @p output).
 */
static size_t cobs_decode(const uint8_t *input, size_t length,
                          uint8_t *output, size_t out_size)
{
    if (!input || !output)
        return 0;

    const uint8_t *in = input;
    const uint8_t *end = input + length;
    uint8_t *out = output;

    while (in < end)
    {
        uint8_t code = *in++;
        if (code == 0 || (in + code - 1) > end)
        {
            return 0; /* Inválido */
        }

        for (uint8_t i = 1; i < code; i++)
        {
            if ((size_t)(out - output) >= out_size)
                return 0; /* Overflow */

            *out++ = *in++;
        }

        if (code < 0xFF && in < end)
        {
            if ((size_t)(out - output) >= out_size)
                return 0;

            *out++ = 0;
        }
    }

    return (size_t)(out - output);
}

/* ============================================================
 * UART + BUFFER DE RECEPÇÃO
 * ============================================================ */

/**
 * @brief Ponteiro global para a UART usada pelo link IPC.
 * */
static HardwareSerial *g_uart = nullptr;

/**
 * @brief Buffer de recepção bruto (frame COBS + CRC + dados).
 * */
static uint8_t g_rx_buf[512];

/**
 * @brief Comprimento atual ocupado em @ref g_rx_buf.
 * */
static size_t g_rx_len = 0;

void ipc_uart_begin(HardwareSerial *serial,
                    uint32_t baud,
                    int8_t tx_pin,
                    int8_t rx_pin)
{
    g_uart = serial;
    if (g_uart)
        g_uart->begin(baud, SERIAL_8N1, rx_pin, tx_pin);

    g_rx_len = 0;
}

void ipc_uart_send_json(const char *json_str)
{
    if (!g_uart || !json_str)
        return;

    size_t json_len = strlen(json_str);
    if (json_len == 0)
        return;

    /* Payload = JSON + CRC16 (2 bytes) */
    uint8_t raw[512];
    if (json_len + 2 > sizeof(raw))
        return;

    memcpy(raw, json_str, json_len);

    uint16_t crc = crc16_ccitt((const uint8_t *)json_str, json_len);
    raw[json_len] = (uint8_t)((crc >> 8) & 0xFF); /* High */
    raw[json_len + 1] = (uint8_t)(crc & 0xFF);    /* Low */

    /* COBS encode */
    uint8_t enc[600];
    size_t enc_len = cobs_encode(raw, json_len + 2, enc, sizeof(enc));
    if (enc_len == 0)
        return;

    /* Envia: COBS(payload) + 0x00 como delimitador */
    g_uart->write(enc, enc_len);
    g_uart->write((uint8_t)0x00);
}

bool ipc_uart_read_json(char *out_json, size_t maxlen)
{
    if (!g_uart || !out_json || maxlen == 0)
        return false;

    while (g_uart->available() > 0)
    {
        uint8_t c = (uint8_t)g_uart->read();

        if (c == 0x00)
        {
            /* Fim de quadro */
            if (g_rx_len == 0)
            {
                /* Quadro vazio, ignora */
                continue;
            }

            /* Decodifica COBS */
            uint8_t dec[512];
            size_t dec_len = cobs_decode(g_rx_buf, g_rx_len, dec, sizeof(dec));
            g_rx_len = 0;

            if (dec_len < 3)
            {
                /* Muito pequeno: pelo menos 1 byte de dado + 2 de CRC */
                return false;
            }

            size_t payload_len = dec_len - 2;
            uint16_t crc_rx = ((uint16_t)dec[payload_len] << 8) |
                              dec[payload_len + 1];

            uint16_t crc_calc = crc16_ccitt(dec, payload_len);
            if (crc_calc != crc_rx)
            {
                /* CRC inválido */
                return false;
            }

            if (payload_len >= maxlen)
            {
                /* JSON maior que o buffer de saída */
                return false;
            }

            memcpy(out_json, dec, payload_len);
            out_json[payload_len] = '\0';
            return true;
        }
        else
        {
            if (g_rx_len < sizeof(g_rx_buf))
            {
                g_rx_buf[g_rx_len++] = c;
            }
        }
    }

    return false;
}
