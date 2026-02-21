/**
 * @file logger.cpp
 * @brief Utilitário de logging com timestamp para Serial e arquivo no SD.
 */

#include "logger.h"
#include <Arduino.h>
#include "sd_card.h"
#include <time.h>
#include <sys/time.h>

static bool g_log_ready = false;
static char s_line[256];

/* ============================================================
 * FUNÇÕES PRIVADAS
 * ============================================================ */

/**
 * @brief Formata o timestamp atual em horário local com milissegundos.
 * @param out Buffer de saída para a string de timestamp.
 * @param outlen Tamanho do buffer @p out, em bytes.
 */
static void format_timestamp(char *out, size_t outlen)
{
    time_t now = time(nullptr);
    struct tm tm_local;
    localtime_r(&now, &tm_local);
    uint32_t ms = (uint32_t)(millis() % 1000U);
    snprintf(out, outlen, "%04d/%02d/%02d %02d:%02d:%02d.%03u",
             tm_local.tm_year + 1900, tm_local.tm_mon + 1, tm_local.tm_mday,
             tm_local.tm_hour, tm_local.tm_min, tm_local.tm_sec, ms);
}

/* ============================================================
 * FUNÇÕES PÚBLICAS
 * ============================================================ */

void logger_init_epoch0()
{
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
}

void logger_begin()
{
    if (!Serial)
    {
        Serial.begin(115200);
        uint32_t t0 = millis();
        while (!Serial && (millis() - t0) < 800)
        {
        }
    }

    g_log_ready = true;
    char ts[40];
    format_timestamp(ts, sizeof(ts));

    if (Serial)
    {
        Serial.printf("%s [LOGGER] pronto\n", ts);
    }

    sdcard_printf("%s [LOGGER] pronto\n", ts);
}

void logger_log(const char *tag, const char *fmt, ...)
{
    if (!g_log_ready)
    {
        return;
    }

    const char *t = tag ? tag : "LOG";
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_line, sizeof(s_line), fmt, ap);
    va_end(ap);
    char ts[40];
    format_timestamp(ts, sizeof(ts));

    if (Serial)
    {
        Serial.print(ts);
        Serial.print(" [");
        Serial.print(t);
        Serial.print("] ");
        Serial.println(s_line);
    }

    sdcard_printf("%s [%s] %s\n", ts, t, s_line);
}
