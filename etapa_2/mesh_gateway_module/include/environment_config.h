/**
 * @file environment_config.h
 * @brief Defaults do controle ambiental (modo caixa/estufa).
 */
#ifndef ENVIRONMENT_CONFIG_H
#define ENVIRONMENT_CONFIG_H

#define ENV_ACT_HB_TIMEOUT_MS     30000UL  // considera o act-00 "offline" se ficar 30s sem HB

/* ======= Identificadores lógicos ======= */
#define ENV_NODE_INT        "int-sen-00"
#define ENV_NODE_EXT        "ext-sen-00"
#define ENV_NODE_ACT        "act-00"
#define ENV_NODE_MSH_GW     "msh-gw"

/* ======= Periodicidade ======= */
#define ENV_CONTROL_PERIOD_MS     1000UL   // 1 Hz
#define ENV_REFRESH_FORCE_MS      5000UL   // reenvia mesmo sem mudança (robustez)
#define ENV_STALE_INT_MS          30000UL  // sem t_in/rh_in/lux_in -> failsafe
#define ENV_STALE_EXT_MS          30000UL

/* ======= Fotoperíodo (RTC) =======
 * Bonsai (ex.: Ficus): use 12h ON/12h OFF como base de teste.
 * Ajuste LIGHT_ON/OFF conforme sua rotina.
 */
#define ENV_LIGHT_ON_HOUR         8   // 08:00
#define ENV_LIGHT_OFF_HOUR        20  // 20:00
#define ENV_LIGHT_RAMP_SEC        300 // 5 min sunrise/sunset

/* ======= LED por lux (BH1750) ======= */
#define ENV_LUX_TARGET_DAY        6000   // alvo interno (ajuste no teste)
#define ENV_LUX_TARGET_NIGHT      0
#define ENV_LED_PWM_MAX           255
#define ENV_LED_SLEW_PER_SEC      8      // limita variação de PWM por segundo
#define ENV_LED_KP                0.02f  // ganho proporcional (PWM por lux de erro)

/* ======= Temperatura/Umidade ======= */
#define ENV_T_SP                  27.0f
#define ENV_T_HYST                0.5f

#define ENV_RH_LOW                45.0f  // liga umidificador abaixo
#define ENV_RH_HIGH               60.0f  // desliga acima
#define ENV_RH_SP_VENT            70.0f  // começa ventilar por umidade acima
#define ENV_RH_HYST               3.0f
#define ENV_RH_MAX                75.0f  // teto duro

/* ======= Ventilação ======= */
#define ENV_PWM_MAX               255
#define ENV_EXHAUST_BIAS          10     // exaustão um pouco maior que intake
#define ENV_FAN_MIN_WHEN_LED_ON   40     // ajuda a dissipar calor com luz ligada

#define ENV_DT_MIN_COOL           0.3f   // t_out < t_in - DT => ventilar ajuda
#define ENV_DRH_MIN_DEHUM         2.0f   // rh_out < rh_in - DRH => ajuda desumidificar

/* ======= Proteção térmica do LED ======= */
#define ENV_T_LED_DERATE          30.0f
#define ENV_T_LED_CUTOFF          33.0f

/* ======= Irrigação (solo) =======
 * Ajuste conforme seu sensor (seco/úmido).
 */
#define ENV_SOIL_DRY_TH           1400
#define ENV_SOIL_WET_TH           2000
#define ENV_IRRIG_MAX_ON_MS       15000UL      // 15 s
#define ENV_IRRIG_MIN_INTERVAL_MS (20UL*60UL*1000UL) // 20 min

/* ======= EMA (filtro) ======= */
#define ENV_EMA_ALPHA_T           0.20f
#define ENV_EMA_ALPHA_RH          0.20f
#define ENV_EMA_ALPHA_LUX         0.25f
#define ENV_EMA_ALPHA_SOIL        0.20f

#endif /* ENVIRONMENT_CONFIG_H */
