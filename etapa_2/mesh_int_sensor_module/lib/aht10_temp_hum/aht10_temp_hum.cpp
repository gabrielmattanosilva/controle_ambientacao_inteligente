/**
 * @file aht10_temp_hum.cpp
 * @brief
 */

#include "aht10_temp_hum.h"

bool aht10_temp_hum_init(aht10_temp_hum_t *ctx)
{
    if (!ctx)
    {
        return false;
    }

    ctx->ok = ctx->dev.begin();
    return ctx->ok;
}

bool aht10_temp_hum_read(aht10_temp_hum_t *ctx, float *temp_c, float *rh_pct)
{
    if (!ctx || !ctx->ok || !temp_c || !rh_pct)
    {
        return false;
    }

    sensors_event_t hum, temp;
    bool ok = ctx->dev.getEvent(&hum, &temp);
    if (!ok)
    {
        return false;
    }

    *temp_c = temp.temperature;
    *rh_pct = hum.relative_humidity;
    return true;
}
