/**
 * @file pins.h
 * @brief Definição dos pinos utilizados no hardware.
 */

#ifndef PINS_H
#define PINS_H

/* UART para link com blynk_gateway_module */
#define UART_TX_PIN 27
#define UART_RX_PIN 26

/* Interface I2C (DS1307) */
#define I2C_SDA     21
#define I2C_SCL     22

/* Interface SPI (cartão SD) */
#define SPI_SCK     18
#define SPI_MISO    19
#define SPI_MOSI    23

/* Módulo de Cartão SD */
#define SD_SPI_CS   4

#endif /* PINS_H */
