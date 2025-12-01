# 📡 mesh-sniffer — Módulo Dummy de Recepção Mesh  
### Sniffer universal para redes painlessMesh + mesh_proto

Este projeto implementa um **nó passivo de recepção** para redes ESP32 baseadas em **painlessMesh** usando o protocolo padronizado **mesh_proto**.  

Ele serve como uma ferramenta de **debug, inspeção e análise de tráfego** da sua rede mesh, permitindo:

- Receber **todas as mensagens JSON** enviadas pela mesh  
- Decodificar **telemetria, estados, heartbeats, comandos, eventos, hello e sincronização de tempo**  
- Exibir tudo no Serial, de forma estruturada  
- **Responder automaticamente ACKs** para mensagens com `qos=1`  
- Funcionar como um cliente “sniffer” sem participar da lógica de automação  

Ideal para desenvolvimento, debugging, validação e entendimento do comportamento dos módulos reais da rede (sensores internos, externos, atuadores e gateway).

---

# ✨ Funcionalidades

### ✔ Recepção universal da mesh
Captura qualquer JSON recebido via painlessMesh.

### ✔ Decodificação completa com `mesh_proto`
Suporta todos os tipos:

| Tipo | Descrição |
|------|-----------|
| `tele` | Telemetria interna/externa |
| `state` | Estado dos atuadores |
| `cfg` | Comandos de configuração |
| `hb` | Heartbeat |
| `evt` | Eventos gerais |
| `hello` | Apresentação de nó |
| `ack` | Confirmações QoS |
| `time` | Sincronização de tempo |

### ✔ ACK automático (QoS1)
Se uma mensagem chega com `qos=1`, o sniffer responde com:

```json
{
  "type": "ack",
  "ref": "<id>",
  "status": "ok"
}
```

Usando a função oficial da lib:

```cpp
mesh_proto_qos_send_ack_ok(&msg);
```

### ✔ Compatível com qualquer arquitetura da rede
O sniffer não interfere no tráfego — ele apenas observa e responde ACKs.

### ✔ Códigos prontos para PlatformIO
Integração fácil com ESP32 / Arduino Framework.

---

# 🛠️ Estrutura do projeto

```
mesh-sniffer/
│
├── include/
│   └── credentials.h
│
├── lib/
│   └── mesh_proto/
│
├── src/
│   └── main.cpp
│
└── platformio.ini
```

---

# ⚙️ Configuração — `platformio.ini`

```ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino

build_flags =
    -Iinclude
    -DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE

lib_deps =
    painlessmesh/painlessMesh@^1.5.7
    bblanchon/ArduinoJson@^7.0.0
```

---

# 🔑 Configuração da rede — `credentials.h`

```cpp
#ifndef CREDENTIALS_H
#define CREDENTIALS_H

#define MESH_PREFIX     "ambientacao_mesh"
#define MESH_PASSWORD   "12345678"
#define MESH_PORT       5555

#endif
```

---

# 📂 Código principal do sniffer — `main.cpp`

O sniffer:

- Inicializa painlessMesh
- Registra callback de envio para a lib mesh_proto (`mesh_send_json_cb`)
- Chama `mesh_proto_qos_init(mesh_send_json_cb)`
- Recebe mensagens → decodifica → imprime
- Responde ACKs quando necessário
- Mantém compatibilidade total com demais módulos  

---

# 🎯 Objetivo do projeto

Este módulo é ideal para:

- Debug da malha mesh  
- Desenvolvimento e validação da `mesh_proto`  
- Testes de integração com o Blynk e gateway  
- Análise de estabilidade  
- Diagnóstico de problemas de QoS, HELLO e CFG  

