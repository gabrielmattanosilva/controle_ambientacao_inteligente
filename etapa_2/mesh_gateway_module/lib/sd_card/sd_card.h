/**
 * @file sd_card.h
 * @brief Cabeçalho para a rotina de registro em cartão SD com rotação
 *        diária e cabeçalho de sessão.
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdarg.h>

void sdcard_begin();
void sdcard_tick_rotate();
void sdcard_printf(const char *fmt, ...) __attribute__((format(printf,1,2)));
void sdcard_vprintf(const char *fmt, va_list ap);
void sdcard_flush();
void sdcard_end();

#endif /* SD_CARD_H */