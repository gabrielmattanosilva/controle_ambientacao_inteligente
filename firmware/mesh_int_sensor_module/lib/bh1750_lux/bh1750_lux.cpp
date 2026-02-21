/**
 * @file bh1750_lux.cpp
 * @brief
 */

 #include "bh1750_lux.h"

bool bh1750_lux_init(bh1750_lux_t *ctx, BH1750::Mode mode)
{
    if (!ctx)
    {
        return false;
    }

    ctx->ok = ctx->dev.begin(mode);
    return ctx->ok;
}

bool bh1750_lux_read(bh1750_lux_t *ctx, float *lux_out)
{
    if (!ctx || !ctx->ok || !lux_out)
    {
        return false;
    }

    *lux_out = ctx->dev.readLightLevel();
    return true;
}
