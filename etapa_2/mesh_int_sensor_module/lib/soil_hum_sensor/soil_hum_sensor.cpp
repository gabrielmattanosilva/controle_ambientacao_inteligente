/**
 * @file soil_hum_sensor.cpp
 * @brief
 */

#include "soil_hum_sensor.h"
#include <Arduino.h>
#include <math.h>

static int clampi(int v, int vmin, int vmax)
{
    if (v < vmin)
    {
        return vmin;
    }

    if (v > vmax)
    {
        return vmax;
    }

    return v;
}

void soil_hum_sensor_init(soil_hum_sensor_t *ctx, int pin, int dry_raw, int wet_raw)
{
    if (!ctx)
    {
        return;
    }

    ctx->pin = pin;
    ctx->dry_raw = dry_raw;
    ctx->wet_raw = wet_raw;

    pinMode(pin, INPUT);
}

int soil_hum_sensor_raw_to_percent(const soil_hum_sensor_t *ctx, int raw)
{
    if (!ctx)
        return 0;

    // raw == dry => 0%
    // raw == wet => 100%
    if (ctx->dry_raw == ctx->wet_raw)
    {
        return 0;
    }

    float pct = (float)(ctx->dry_raw - raw) * 100.0f / (float)(ctx->dry_raw - ctx->wet_raw);
    int ipct = (int)lroundf(pct);
    return clampi(ipct, 0, 100);
}

int soil_hum_sensor_read_percent(const soil_hum_sensor_t *ctx)
{
    if (!ctx)
    {
        return 0;
    }

    int raw = analogRead(ctx->pin);
    return soil_hum_sensor_raw_to_percent(ctx, raw);
}
