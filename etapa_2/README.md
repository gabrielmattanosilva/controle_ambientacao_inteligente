# Controle de Ambientação Inteligente - EmbarcaTech 2025 3° Fase

Autores: **Gabriel Mattano, Jorge Wilker e Roger Melo**
Curso: Residência Tecnológica em Sistemas Embarcados
Instituição: EmbarcaTech – HBr
Campinas, Outubro/2025 a Março/2026

---

## ✨ Etapa 2

Documentos referentes a essa etapa:

---

## 🗃️ Estrutura do Projeto

### Módulo Gateway Blynk

```
/blynk_gateway_module
  /include
    credentials.h            (privado)
    pins.h
  /lib
    /blynk_client
      blynk_client.cpp
      blynk_client.h
    /ipc_uart
      ipc_uart.cpp
      ipc_uart.h
  /src
    main.cpp
 platformio.ini
```

### Módulo Gateway Mesh

```
/mesh_gateway_module
  /include
    credentials.h            (privado)
    environment_config.h
    pins.h
  /lib
    /ds1307_rtc
      ds1307_rtc.cpp
      ds1307_rtc.h
    /environment_controller
      environment_controller.cpp
      environment_controller.h
    /ipc_uart
      ipc_uart.cpp
      ipc_uart.h
    /logger
      logger.cpp
      logger.h
    /mesh_gateway
      mesh_gateway.cpp
      mesh_gateway.h
    /sd_card
      sd_card.cpp
      sd_card.h
  /src
    main.cpp
 platformio.ini
```

### Módulo Sensor Interno

```
/int_sensor_module
  /include
    credentials.h            (privado)
    pins.h
  /lib
    /aht10_temp_hum
      aht10_temp_hum.cpp
      aht10_temp_hum.h
    /bh1750_lux
      bh1750_lux.cpp
      bh1750_lux.h
    /soil_hum_sensor
      soil_hum_sensor.cpp
      soil_hum_sensor.h
    /mesh_node
      mesh_node.cpp
      mesh_node.h
  /src
    main.cpp
 platformio.ini
```

### Módulo Sensor Externo

```
/ext_sensor_module
  /include
    credentials.h            (privado)
    pins.h
  /lib
    /aht10_temp_hum
      aht10_temp_hum.cpp
      aht10_temp_hum.h
    /bh1750_lux
      bh1750_lux.cpp
      bh1750_lux.h
    /mesh_node
      mesh_node.cpp
      mesh_node.h
  /src
    main.cpp
 platformio.ini
```

### Módulo Atuador

```
/actuator_module
  /include
    credentials.h            (privado)
    pins.h
  /lib
    /fan_driver
      fan_driver.cpp
      fan_driver.h
    /humidifier_driver
      humidifier_driver.cpp
      humidifier_driver.h
    /led_driver
      led_driver.cpp
      led_driver.h
    /water_pump_driver
      water_pump_driver.cpp
      water_pump_driver.h
    /mesh_node
      mesh_node.cpp
      mesh_node.h
  /src
    main.cpp
 platformio.ini
```

---

## 📦 Dependências de Bibliotecas Externas

| Biblioteca | Versão | Repositório |
|-------------|---------|-------------|
| Adafruit_AHTX0 | 2.0.5 | [Adafruit/Adafruit_AHTX0](https://github.com/adafruit/Adafruit_AHTX0) |
| BH1750 | 1.3.0 | [claws/BH1750](https://github.com/claws/BH1750) |
| Blynk | 1.3.2 | [blynkkk/blynk-library](https://github.com/blynkkk/blynk-library) |
| painlessMesh | 1.5.7 | [painlessMesh/painlessMesh](https://gitlab.com/painlessMesh/painlessMesh) |
| RTClib | 2.1.4 | [Adafruit/RTClib](https://github.com/adafruit/RTClib) |

---

## 📜 Licença

Distribuído sob **GNU GPL‑3.0**.

---


