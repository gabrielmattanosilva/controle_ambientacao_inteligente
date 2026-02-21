# mesh_ext_sensor_module

Nó de sensores externos (malha):
- Temperatura/Umidade: **AHT10/AHT20**
- Luminosidade: **BH1750 (I2C)**

Publica telemetria para o `mesh_gateway_module`.

---

## 📸 Foto do módulo
![Foto do módulo](../../docs/img/sensor_externo.png)

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

---

## 🔌 Pinos / wiring (I2C)
Definidos em `include/pins.h`:
- SDA: **GPIO21**
- SCL: **GPIO22**

Conecte ambos os sensores no mesmo barramento I2C.

---

## ⏱️ Periodicidade
- TELE: **a cada 5 minutos**
- HB (heartbeat): **a cada 1 minuto**

(ajustável via `TELE_PERIOD_MS` e `HB_PERIOD_MS` no código)

---

## 🧪 Teste rápido
1. Grave e abra o serial monitor.
2. Confirme inicialização dos sensores.
3. Ligue o mesh gateway e verifique se o nó entra na malha.
4. No Blynk, confirme atualização de V0..V2 e heartbeat V21.

---

## 🧯 Troubleshooting
- Sensor não inicializa: confira endereço I2C, SDA/SCL e alimentação
- Sem dados no Blynk: verifique se o gateway mesh está ligado e recebendo
