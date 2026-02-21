# mesh_gateway_module

ESP32 responsável por:
- Formar/gerenciar a **rede painlessMesh**
- Receber telemetria dos sensores e encaminhar ao `blynk_gateway_module` via **UART**
- Receber comandos do Blynk (via UART) e encaminhar para os nós na malha
- (Opcional) Sincronizar tempo via **RTC DS1307** e registrar logs em **SD**
- Executar a lógica de **controle automático** (controller)

---

## 📸 Foto do módulo (placeholder)
![Foto do módulo](../../docs/img/gateway.png)

---

## ✅ Como compilar / gravar (PlatformIO)

```bash
pio run -e esp32doit-devkit-v1 -t upload
pio device monitor -b 115200
```

---

## 🔐 Configuração (mesh)
Crie `include/credentials.h` copiando o exemplo:

- `include/credentials.example.h` → `include/credentials.h`

Campos esperados:
- `MESH_PREFIX`
- `MESH_PASSWORD`
- `MESH_PORT`

> ⚠️ `credentials.h` não deve ser versionado.

---

## 🔌 Pinos (hardware)

Definidos em `include/pins.h`:

### UART (link com blynk_gateway_module)
- UART_TX_PIN: **GPIO27**
- UART_RX_PIN: **GPIO26**

### I2C (RTC DS1307)
- SDA: **GPIO21**
- SCL: **GPIO22**

### SPI (SD Card)
- SCK: **GPIO18**
- MISO: **GPIO19**
- MOSI: **GPIO23**
- CS: **GPIO5**

---

## 🔁 Identificação na malha
O gateway se identifica como `msh-gw` e espera nós:
- `ext-sen-00`
- `int-sen-00`
- `act-00`

---

## 🧪 Teste rápido
1. Grave e abra o monitor serial.
2. Ligue um sensor mesh e verifique se aparece conexão/HELLO.
3. Ligue o blynk gateway e verifique tráfego UART (telemetria/CFG).

---

## 🧯 Troubleshooting
- Nós não entram na mesh: confira `MESH_PREFIX/MESH_PASSWORD/MESH_PORT` iguais em todos
- RTC/SD não detecta: confira I2C/SPI e alimentação
- Sem UART: confira TX/RX cruzados e GND comum
