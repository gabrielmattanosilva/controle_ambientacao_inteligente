/**
 * @file logger.h
 * @brief Cabeçalho para o utilitário de logging com timestamp para Serial e arquivo no SD.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

/**
 * @brief Inicializa o RTC interno em @c epoch 0 (1970-01-01 00:00:00 UTC).
 */
void logger_init_epoch0();

/**
 * @brief Inicializa o logger, garantindo a Serial e registrando mensagem "pronto".
 */
void logger_begin();

/**
 * @brief Imprime uma mensagem de log formatada com timestamp e @p tag.
 * @param tag Rótulo do subsistema/área (ex.: "MAIN", "LORA"); se @c nullptr, usa "LOG".
 * @param fmt String de formato no estilo @c printf().
 * @param ... Argumentos variáveis correspondentes a @p fmt.
 */
void logger_log(const char *tag, const char *fmt, ...) __attribute__((format(printf,2,3)));

/**
 * @brief Macro auxiliar para registro de logs formatados.
 *
 * Esta macro encapsula a chamada a @ref logger_log(), permitindo uso de
 * formato printf-like com número variável de argumentos.
 *
 * @param TAG  Rótulo (string) associado ao subsistema/origem do log.
 * @param fmt  String de formato (estilo printf).
 * @param ...  Argumentos variádicos correspondentes ao formato.
 */
#define LOG(TAG, fmt, ...)   logger_log((TAG), (fmt), ##__VA_ARGS__)

#endif /* LOGGER_H */
