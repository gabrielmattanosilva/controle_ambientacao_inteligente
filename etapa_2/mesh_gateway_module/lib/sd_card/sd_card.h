/**
 * @file sd_card.h
 * @brief Cabeçalho para a rotina de registro em cartão SD com rotação
 *        diária e cabeçalho de sessão.
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdarg.h>

/**
 * @brief Inicializa a interface com o cartão SD e abre o primeiro arquivo de log.
 */
void sdcard_begin();

/**
 * @brief Rotina periódica para avaliar rotação diária do arquivo de log.
 */
void sdcard_tick_rotate();

/**
 * @brief Versão @c vprintf para escrever linhas formatadas no arquivo de log.
 * @param fmt String de formato no estilo @c printf().
 * @param ap Lista de argumentos variável (va_list) correspondente a @p fmt.
 */
void sdcard_printf(const char *fmt, ...) __attribute__((format(printf,1,2)));

/**
 * @brief Escreve no arquivo de log usando formato @c printf() com argumentos variáveis.
 * @param fmt String de formato no estilo @c printf().
 * @param ... Argumentos variáveis para @p fmt.
 */
void sdcard_vprintf(const char *fmt, va_list ap);

/**
 * @brief Força a gravação (flush) do arquivo de log atual.
 */
void sdcard_flush();

/**
 * @brief Encerra o subsistema de SD, fechando o arquivo atual e desabilitando o uso.
 */
void sdcard_end();

#endif /* SD_CARD_H */