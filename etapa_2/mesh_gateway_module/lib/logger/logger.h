/**
 * @file logger.h
 * @brief Cabeçalho para o utilitário de logging com timestamp para Serial e arquivo no SD.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>

void logger_init_epoch0();
void logger_begin();
void logger_log(const char *tag, const char *fmt, ...) __attribute__((format(printf,2,3)));
void logger_hexdump(const char *tag, const uint8_t *buf, size_t len);

#define LOG(TAG, fmt, ...)   logger_log((TAG), (fmt), ##__VA_ARGS__)
#define LOGHEX(TAG, B, L)    logger_hexdump((TAG), (const uint8_t*)(B), (size_t)(L))

#endif /* LOGGER_H */
