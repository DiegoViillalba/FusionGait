# Guía de Firmware — FusionGait

Describe los tres sketches de Arduino disponibles, cuándo usar cada uno y cómo subirlos.

---

## Hardware soportado

| Placa | IMU | `IMU_REV2` en `config.h` |
|---|---|---|
| Arduino Nano 33 BLE Sense (original) | LSM9DS1 | `0` |
| Arduino Nano 33 BLE Sense **Rev2** | BMI270 | `1` |

> Puedes ver la versión en la serigrafía del PCB. Si pone "Rev2" → usa `1`.

---

## Modos de operación

### Modo A — USB directo (recomendado para empezar)

Todos los Arduinos conectados al PC por cable USB. Es el modo más sencillo y estable.

```
PC ──USB── Arduino 1 (pelvis)
PC ──USB── Arduino 2 (thigh)
PC ──USB── Arduino 3 (ankle)
```

**Firmware:** `firmware/data_logger_node`

**Script Python:** `acquisition/gait_gui.py` (GUI con etiquetado por fases)

---

### Modo B — BLE inalámbrico con hub

Un Arduino actúa como hub BLE: se conecta a los esclavos por BLE y al PC por BLE.
Solo el hub necesita cable USB al PC.

```
GaitNode_2 ──BLE──┐
                   ├── GaitHub ──BLE── PC
GaitNode_3 ──BLE──┘
```

**Firmware hub:** `firmware/data_logger_ble_host` → solo al Arduino 1 (pelvis)  
**Firmware esclavo:** `firmware/data_logger_ble` → Arduinos 2 y 3

**Script Python:** `acquisition/ble_hub_logger.py` o `acquisition/live_plot.py --ble-hub`

> ⚠️ En macOS el stack BLE puede ser inestable. Usa el Modo A si tienes problemas.

---

## Firmware 1 — `data_logger_node` (Modo A / USB)

**Ubicación:** `firmware/data_logger_node/`

**Función:** Lee el IMU a 100 Hz y envía datos CSV por Serial USB. Responde a comandos de texto (`STATUS`, `START`, `STOP`, `SET_TRIAL:`, `SET_SUBJECT:`).

**Formato de salida por Serial (una línea por muestra):**
```
timestamp_ms,sensor_id,placement,ax,ay,az,gx,gy,gz
1234,1,pelvis,0.12,-0.03,1.01,2.1,-0.5,0.3
```

### Configuración antes de subir

Edita `firmware/data_logger_node/config.h`:

```cpp
#define SENSOR_ID  1          // 1=pelvis, 2=thigh, 3=ankle
#define PLACEMENT  "pelvis"   // nombre del segmento corporal
#define IMU_REV2   1          // 0=LSM9DS1, 1=BMI270 (Rev2)
```

### Subir con el script automático

```bash
# Primero edita los puertos en scripts/upload_nodes.sh
bash scripts/upload_nodes.sh wired
```

### Subir manualmente (un nodo)

```bash
# Parchear config.h
sed -i '' 's/^#define SENSOR_ID.*/#define SENSOR_ID  1/' firmware/data_logger_node/config.h
sed -i '' 's/^#define PLACEMENT.*/#define PLACEMENT  "pelvis"/' firmware/data_logger_node/config.h

# Compilar y subir
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble firmware/data_logger_node
arduino-cli upload  --fqbn arduino:mbed_nano:nano33ble \
    --port /dev/cu.usbmodem11201 firmware/data_logger_node
```

Repetir para cada nodo cambiando `SENSOR_ID`, `PLACEMENT` y `--port`.

| Nodo | `SENSOR_ID` | `PLACEMENT` | Puerto típico |
|---|---|---|---|
| 1 | 1 | `"pelvis"` | `/dev/cu.usbmodem11201` |
| 2 | 2 | `"thigh"` | `/dev/cu.usbmodem11301` |
| 3 | 3 | `"ankle"` | `/dev/cu.usbmodem11401` |

> **Ver puertos disponibles:** `arduino-cli board list`

---

## Firmware 2 — `data_logger_ble` (Modo B / esclavo BLE)

**Ubicación:** `firmware/data_logger_ble/`

**Función:** Anuncia como `GaitNode_N` por BLE. El hub se conecta a él, se suscribe a las notificaciones IMU y le envía comandos START/STOP.

### Configuración antes de subir

Edita `firmware/data_logger_ble/config.h`:

```cpp
#define SENSOR_ID       2          // 2 o 3 (nunca 1, ese es el hub)
#define PLACEMENT       "thigh"    // "thigh" o "ankle"
#define BLE_DEVICE_NAME "GaitNode_2"  // debe coincidir con SENSOR_ID
#define IMU_REV2        1
```

### Subir

```bash
# Script automático (modo hub sube hub + esclavos a la vez)
bash scripts/upload_nodes.sh hub

# O manualmente para el nodo 2
sed -i '' 's/^#define SENSOR_ID.*/#define SENSOR_ID  2/' firmware/data_logger_ble/config.h
sed -i '' 's/^#define PLACEMENT.*/#define PLACEMENT  "thigh"/' firmware/data_logger_ble/config.h
sed -i '' 's/^#define BLE_DEVICE_NAME.*/#define BLE_DEVICE_NAME  "GaitNode_2"/' firmware/data_logger_ble/config.h

arduino-cli compile --fqbn arduino:mbed_nano:nano33ble firmware/data_logger_ble
arduino-cli upload  --fqbn arduino:mbed_nano:nano33ble \
    --port /dev/cu.usbmodem11301 firmware/data_logger_ble
```

---

## Firmware 3 — `data_logger_ble_host` (Modo B / hub BLE)

**Ubicación:** `firmware/data_logger_ble_host/`

**Función:** Solo va al Arduino 1 (pelvis). Actúa simultáneamente como:
- **BLE Central** → busca y conecta con `GaitNode_2` y `GaitNode_3`
- **BLE Peripheral** → anuncia como `GaitHub` al PC

**Máquina de estados del hub:**
```
Arranca → SCAN_S2 (10 s) → SCAN_S3 (10 s) → READY (LED azul) → CAPTURING
```
Si no encuentra un esclavo en 10 s, lo salta y continúa.  
En estado READY el hub anuncia por BLE y espera que el PC se conecte.

**Orden de arranque obligatorio:**
1. Encender esclavos (GaitNode_2, GaitNode_3) → LED azul
2. Encender hub → LED rojo → esperar ~20 s → LED azul (READY)
3. Conectar desde Python

### Configuración

`firmware/data_logger_ble_host/config.h` no necesita cambios manuales — los UUIDs y nombres están fijos.

### Subir

```bash
# Script automático (recomendado)
bash scripts/upload_nodes.sh hub

# O manualmente solo el hub
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble firmware/data_logger_ble_host
arduino-cli upload  --fqbn arduino:mbed_nano:nano33ble \
    --port /dev/cu.usbmodem11201 firmware/data_logger_ble_host
```

---

## Resumen de comandos Serial (todos los firmwares)

Puedes enviar estos comandos por el Serial Monitor (115200 baud) o desde Python:

| Comando | Respuesta | Efecto |
|---|---|---|
| `STATUS` | `STATUS:sensor_id=1,...` | Consulta estado actual |
| `START` | `ACK:START` | Inicia captura / streaming |
| `STOP` | `ACK:STOP:<n_muestras>` | Detiene captura |
| `SET_TRIAL:t001` | `ACK:SET_TRIAL:t001` | Asigna ID de trial |
| `SET_SUBJECT:s001` | `ACK:SET_SUBJECT:s001` | Asigna ID de sujeto |

---

## Flujo completo — Modo A (USB, recomendado)

```bash
# 1. Verificar puertos
arduino-cli board list

# 2. Subir firmware a los 2-3 Arduinos
bash scripts/upload_nodes.sh wired

# 3. Lanzar GUI de adquisición
python3.11 acquisition/gait_gui.py
```

En la GUI:
- Ajusta Sujeto / Trial
- Clic en **Conectar** (auto-detecta puertos)
- Selecciona fase con teclas **1 / 2 / 3 / 4**
- **Space** para grabar / detener
- Los CSV se guardan en `data/raw/`

---

## Flujo completo — Modo B (BLE hub, 2 nodos)

```bash
# 1. Subir firmware hub a nodo 1 y esclavo a nodo 2
bash scripts/upload_nodes.sh hub

# 2. Arrancar en orden:
#    a) Encender GaitNode_2 (powerbank) → LED azul
#    b) Encender GaitHub (powerbank o USB) → esperar LED azul (~20 s)

# 3. Verificar que GaitHub es visible
python3.11 -c "
import asyncio; from bleak import BleakScanner
async def s():
    devs = await BleakScanner.discover(timeout=10)
    [print(d.name, d.address) for d in devs if d.name]
asyncio.run(s())
"

# 4. Lanzar adquisición
python3.11 acquisition/ble_hub_logger.py --subject_id s001 --trial_id t001
# o live plot:
python3.11 acquisition/live_plot.py --ble-hub
```

---

## Troubleshooting

| Síntoma | Causa probable | Solución |
|---|---|---|
| `No device found on cu.usbmodemXXX` | Puerto ocupado por Serial Monitor | Cerrar terminal con monitor abierto |
| `ERROR:IMU_NOT_FOUND` en Serial | `IMU_REV2` incorrecto en `config.h` | Cambiar `0 ↔ 1` y re-subir |
| Arduino no aparece en `board list` | Cable USB sin datos | Cambiar por cable con datos |
| GaitHub no aparece en BLE scan | Hub en ciclo de reconexión | Reiniciar hub después de los esclavos |
| ODR << 100 Hz en CSV | Gaps en Serial USB | Cerrar otras aplicaciones; reducir carga del PC |
