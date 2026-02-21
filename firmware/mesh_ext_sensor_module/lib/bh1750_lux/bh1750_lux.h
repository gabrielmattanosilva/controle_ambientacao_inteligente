/**
 * @file bh1750_lux.h
 * @brief
 */

#ifndef BH1750_LUX_H
#define BH1750_LUX_H

#include <stdbool.h>
#include <BH1750.h>

/**
 * Wrapper C-like para BH1750 (lux).
 */
typedef struct
{
    BH1750 dev;
    bool ok;
} bh1750_lux_t;

/**
 * Inicializa o BH1750 com o modo desejado.
 * @return true se OK (ctx->ok = true)
 */
bool bh1750_lux_init(bh1750_lux_t *ctx, BH1750::Mode mode);

/**
 * Lê lux (float).
 * @return true se conseguiu ler (não valida NAN/negativo aqui; isso fica no main
 *         para manter exatamente a mesma regra que você já tinha).
 */
bool bh1750_lux_read(bh1750_lux_t *ctx, float *lux_out);

/**
 * Retorna status do sensor (inicialização).
 */
static inline bool bh1750_lux_is_ok(const bh1750_lux_t *ctx)
{
    return (ctx && ctx->ok);
}

#endif /* BH1750_LUX_H */
