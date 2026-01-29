/**
 * @file aht10_temp_hum.h
 * @brief
 */

#ifndef AHT10_TEMP_HUM_H
#define AHT10_TEMP_HUM_H

#include <stdbool.h>
#include <Adafruit_AHTX0.h>

/**
 * Wrapper C-like para AHT10/AHT20 via Adafruit_AHTX0
 * Mantém responsabilidade do sensor isolada.
 */
typedef struct
{
    Adafruit_AHTX0 dev;
    bool ok;
} aht10_temp_hum_t;

/**
 * Inicializa o AHTX0.
 * @return true se OK (ctx->ok = true)
 */
bool aht10_temp_hum_init(aht10_temp_hum_t *ctx);

/**
 * Lê temperatura (°C) e umidade relativa (%).
 * @return true se leitura ok (getEvent ok)
 */
bool aht10_temp_hum_read(aht10_temp_hum_t *ctx, float *temp_c, float *rh_pct);

/**
 * Retorna status do sensor (inicialização).
 */
static inline bool aht10_temp_hum_is_ok(const aht10_temp_hum_t *ctx)
{
    return (ctx && ctx->ok);
}

#endif /* AHT10_TEMP_HUM_H */
