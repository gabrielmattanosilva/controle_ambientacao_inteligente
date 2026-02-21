/**
 * @file soil_hum_sensor.h
 * @brief
 */

#ifndef SOIL_HUM_SENSOR_H
#define SOIL_HUM_SENSOR_H

#include <stdint.h>

#define SOIL_DRY_VALUE 2521
#define SOIL_WET_VALUE 1200

/**
 * Sensor capacitivo de umidade do solo (analógico).
 * Converte leitura raw -> % usando dry_raw e wet_raw.
 */
typedef struct
{
    int pin;
    int dry_raw; // raw que representa 0%
    int wet_raw; // raw que representa 100%
} soil_hum_sensor_t;

/**
 * Inicializa sensor (define pin e calibração) e configura pinMode(INPUT).
 */
void soil_hum_sensor_init(soil_hum_sensor_t *ctx, int pin, int dry_raw, int wet_raw);

/**
 * Converte um raw (ADC) em porcentagem [0..100] (mesma regra do seu main atual).
 */
int soil_hum_sensor_raw_to_percent(const soil_hum_sensor_t *ctx, int raw);

/**
 * Lê ADC do pino e retorna % [0..100].
 */
int soil_hum_sensor_read_percent(const soil_hum_sensor_t *ctx);

#endif /* SOIL_HUM_SENSOR_H */
