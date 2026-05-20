# Guía de Inferencia en Tiempo Real — FusionGait

Pasos para desplegar el modelo TinyML entrenado en los dos Arduinos y
visualizar la clasificación de fases de marcha en tiempo real.

---

## Arquitectura del sistema

```
Arduino 2 (pierna)          Arduino 1 (tobillo)          PC
 ─────────────────           ─────────────────────       ────────────────
 Lee BMI270 propio   ──BLE──► Recibe datos pierna        inference_gui.py
                              Lee BMI270 propio           ▲
                              Normaliza (scaler.h)        │ USB Serial
                              Ring buffer 50×12           │ (INFER lines)
                              TFLite int8 (9 KB)   ───────┘
                              → INFER,swing,0.01,...
```

- **Slave** (`firmware/inference_slave`): anuncia IMU de la pierna vía BLE a 100 Hz.
- **Master** (`firmware/inference_master`): BLE central + TFLite. Corre inferencia
  cada 25 muestras (~4 veces/s) y envía el resultado al PC por Serial USB.
- **GUI** (`inference/inference_gui.py`): visualiza fase + probabilidades + timeline.

---

## Requisitos

### Arduino
- Arduino Nano 33 BLE Sense **Rev2** (nRF52840 + BMI270)  
  *(o Rev1 con `IMU_REV2 0` en ambos `config.h`)*
- Arduino IDE 2.x con las siguientes librerías instaladas:

| Librería | Dónde instalar |
|---|---|
| `ArduinoBLE` | Gestor de librerías |
| `Arduino_BMI270_BMM150` | Gestor de librerías |
| `Arduino_TensorFlowLite` | Gestor de librerías |

> **Placa:** `Tools → Board → Arduino Mbed OS Nano Boards → Arduino Nano 33 BLE`

### Python
```bash
source .venv/bin/activate   # o python3.11 -m venv .venv && source .venv/bin/activate
pip install pyserial pyqtgraph PyQt6 numpy
```

---

## Paso 0 — Entrenar el modelo

Si aún no has corrido el notebook:

```bash
cd /Users/diegovillaba/Code/FusionGait
source .venv/bin/activate
jupyter notebook training/notebooks/gait_tinyml.ipynb
# → Run All
# Genera: training/models/gait_model_2s.h
#          training/models/scaler_params_2s.npz
```

---

## Paso 1 — Exportar artefactos al firmware

```bash
python3 scripts/export_to_arduino.py
```

Salida esperada:
```
✓ Modelo copiado  : firmware/inference_master/gait_model_2s.h  (9.3 KB)
✓ Scaler exportado: firmware/inference_master/scaler.h
✅  Listo. Siguientes pasos: ...
```

Esto copia `gait_model_2s.h` y genera `scaler.h` con los valores exactos de
media y desviación estándar calculados durante el entrenamiento.

---

## Paso 2 — Subir firmware al Esclavo (Arduino 2 / pierna)

1. Conecta **solo** el Arduino 2 por USB.
2. Abre `firmware/inference_slave/inference_slave.ino` en Arduino IDE.
3. Verifica `config.h`:
   ```cpp
   #define SENSOR_ID   2
   #define PLACEMENT   "pierna"
   #define IMU_REV2    1          // 0 si tienes Rev1
   #define BLE_DEVICE_NAME "GaitSlave_2"
   ```
4. `Sketch → Upload` (o `Ctrl+U`).
5. Al terminar: LED **azul** fijo → anunciando por BLE.

---

## Paso 3 — Subir firmware al Master (Arduino 1 / tobillo)

1. Desconecta el esclavo. Conecta **solo** el Arduino 1 por USB.
2. Abre `firmware/inference_master/inference_master.ino` en Arduino IDE.
3. Verifica `config.h`:
   ```cpp
   #define SENSOR_ID   1
   #define PLACEMENT   "tobillo"
   #define IMU_REV2    1
   #define SLAVE_NAME  "GaitSlave_2"
   #define TENSOR_ARENA_KB  72   // aumentar a 96 si ves ERROR:ARENA_TOO_SMALL
   ```
4. `Sketch → Upload`.
5. Abre el Serial Monitor (115200 baud) para verificar:
   ```
   IMU OK — acc 100.00 Hz
   ARENA:38240 B
   INPUT_SHAPE:(1,50,12,1)
   STATUS:SCANNING for GaitSlave_2
   ```

---

## Paso 4 — Arranque correcto

> ⚠️ El orden importa: el Master busca al Esclavo activamente.

1. **Alimenta el Esclavo primero** (USB o powerbank) → LED azul.
2. **Alimenta el Master** → LED rojo (escaneando BLE) → en ~5 s LED azul → verde.
3. Si el Master no encuentra al Esclavo en 30 s:
   - Reinicia el Esclavo primero, luego el Master.
   - Verifica que `BLE_DEVICE_NAME` coincida entre ambos `config.h`.

Serial del Master cuando todo funciona:
```
STATUS:FOUND GaitSlave_2
STATUS:SLAVE_CONNECTED
INFER,swing,0.02,0.05,0.03,0.90
INFER,swing,0.01,0.04,0.02,0.93
INFER,loading,0.78,0.12,0.06,0.04
...
```

---

## Paso 5 — Lanzar la GUI

Mantén el Master conectado al PC por USB:

```bash
source .venv/bin/activate
python3 inference/inference_gui.py
```

O especificando el puerto directamente:

```bash
python3 inference/inference_gui.py --port /dev/cu.usbmodem11201
```

### Controles de la GUI

| Elemento | Descripción |
|---|---|
| Selector de puerto | Auto-detecta Arduinos. Refrescar con **↺** |
| **Conectar** | Abre Serial y empieza a recibir inferencias |
| Recuadro grande | Fase actual con color (rojo/naranja/verde/azul) |
| % confianza | Probabilidad de la clase ganadora |
| Barras de probabilidad | Distribución softmax de las 4 fases |
| Timeline | Historial scrolling de los últimos 12 s |
| Hz (esquina superior derecha) | Tasa real de inferencia medida |

---

## Troubleshooting

| Síntoma | Causa | Solución |
|---|---|---|
| `ERROR:ARENA_TOO_SMALL` | Arena insuficiente | Aumentar `TENSOR_ARENA_KB` en `config.h` del Master (prueba 96) |
| `ERROR:TFLITE_SCHEMA_MISMATCH` | Versión de librería incompatible | Actualizar `Arduino_TensorFlowLite` |
| `INPUT_SHAPE:(1,50,12)` en vez de `(1,50,12,1)` | Modelo 1D subido en lugar de 2D | Re-exportar con `scripts/export_to_arduino.py` tras correr el notebook con `ARCH='cnn2d'` |
| Master no encuentra al Esclavo | BLE scan timeout | Reiniciar Esclavo primero, luego Master |
| GUI no muestra datos | Puerto incorrecto o Master no conectado | Verificar Serial Monitor primero; seleccionar puerto correcto |
| Fase siempre "swing" | Scaler con valores incorrectos | Verificar que `scaler.h` fue generado por `export_to_arduino.py` y no es el placeholder |
| ODR del Esclavo < 100 Hz | BLE congestionado | Reducir otras apps BLE en el entorno |

---

## Protocolo Serial (referencia)

El Master emite las siguientes líneas (115200 baud, `\n`):

```
INFER,<fase>,<p0>,<p1>,<p2>,<p3>   # inferencia: fase ganadora + 4 probs
STATUS:<mensaje>                    # estado BLE / arranque
ARENA:<bytes> B                     # uso de arena TFLite (al arrancar)
INPUT_SHAPE:(1,50,12,1)             # forma del tensor de entrada
ERROR:<mensaje>                     # error fatal
```

El orden de las probabilidades sigue el `PHASE2ID` del notebook:
`p0=loading, p1=midstance, p2=terminal, p3=swing`

---

## Extender a 3 sensores

1. En el notebook: `align_sensors(df, sensor_ids=[1,2,3])` y `N_SENSORS=3` → re-entrenar.
2. Agregar un segundo esclavo `firmware/inference_slave/` con `SENSOR_ID=3`, `BLE_DEVICE_NAME="GaitSlave_3"`.
3. En `inference_master.ino`: agregar segundo `BLECharacteristic imu_char2` y segundo callback.
4. En `config.h`: `N_FEATURES 18` y actualizar `scaler.h` con 18 valores.
