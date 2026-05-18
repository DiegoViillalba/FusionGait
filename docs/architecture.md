# Arquitectura del Sistema FusionGait

## 1. Diagrama lógico de componentes

```
╔══════════════════════════════════════════════════════════════════════════╗
║                         FASE A — ADQUISICIÓN                            ║
║                                                                          ║
║  ┌───────────────┐   ┌───────────────┐   ┌───────────────┐             ║
║  │  Arduino #1   │   │  Arduino #2   │   │  Arduino #3   │             ║
║  │  (pelvis)     │   │  (thigh)      │   │  (ankle)      │             ║
║  │               │   │               │   │               │             ║
║  │ LSM9DS1 IMU   │   │ LSM9DS1 IMU   │   │ LSM9DS1 IMU   │             ║
║  │ ax,ay,az      │   │ ax,ay,az      │   │ ax,ay,az      │             ║
║  │ gx,gy,gz      │   │ gx,gy,gz      │   │ gx,gy,gz      │             ║
║  │ millis()      │   │ millis()      │   │ millis()      │             ║
║  └──────┬────────┘   └──────┬────────┘   └──────┬────────┘             ║
║         │ USB Serial         │ USB Serial         │ USB Serial           ║
║         └──────────┬─────────┘───────────────────┘                      ║
║                    │                                                      ║
║         ┌──────────▼───────────┐                                         ║
║         │   Host Computer      │                                         ║
║         │  serial_logger.py    │                                         ║
║         │  ble_logger.py       │                                         ║
║         └──────────┬───────────┘                                         ║
║                    │                                                      ║
║         ┌──────────▼───────────┐                                         ║
║         │   data/raw/*.csv     │  ← CSVs crudos por trial                ║
║         │   data/labels/*.csv  │  ← Archivos de etiquetas                ║
║         └──────────────────────┘                                         ║
╚══════════════════════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════════════════════╗
║                      FASE B — ENTRENAMIENTO OFFLINE                     ║
║                                                                          ║
║  data/raw/*.csv  ──►  preprocess.py  ──►  windowing.py                  ║
║                                               │                          ║
║                                    ┌──────────▼──────────┐              ║
║                                    │  data/processed/     │              ║
║                                    │  windows_train.npz   │              ║
║                                    │  windows_val.npz     │              ║
║                                    │  windows_test.npz    │              ║
║                                    └──────────┬───────────┘              ║
║                                               │                          ║
║             ┌────────────────────────────────┼──────────┐               ║
║             │                                │          │               ║
║     train_baselines.py               train_cnn1d.py   ...               ║
║    (LogReg, RF, SVM, MLP)           (CNN 1D, LSTM)                      ║
║             │                                │                          ║
║             └────────────────┬───────────────┘                          ║
║                              │                                           ║
║                    evaluate.py  →  reports/                              ║
║                    (accuracy, F1, confusión, RAM, Flash)                 ║
╚══════════════════════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════════════════════╗
║                       FASE C — DESPLIEGUE TinyML                        ║
║                                                                          ║
║   mejor_modelo.h5                                                        ║
║         │                                                                ║
║   export_tflite.py  (cuantización INT8)                                  ║
║         │                                                                ║
║   model.tflite  ──►  xxd -i  ──►  model_data.h  (C array)              ║
║         │                                                                ║
║  ┌──────▼────────┐   ┌───────────────┐   ┌───────────────┐             ║
║  │  Arduino #1   │   │  Arduino #2   │   │  Arduino #3   │             ║
║  │  sensor_node  │   │  sensor_node  │   │  sensor_node  │             ║
║  │               │   │               │   │               │             ║
║  │ IMU → buffer  │   │ IMU → buffer  │   │ IMU → buffer  │             ║
║  │ ventaneo      │   │ ventaneo      │   │ ventaneo      │             ║
║  │ TFLite Micro  │   │ TFLite Micro  │   │ TFLite Micro  │             ║
║  │ predicción    │   │ predicción    │   │ predicción    │             ║
║  │ confianza     │   │ confianza     │   │ confianza     │             ║
║  └───────────────┘   └───────────────┘   └───────────────┘             ║
╚══════════════════════════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════════════════════════╗
║                        FASE D — FUSIÓN MULTISENSOR                      ║
║                                                                          ║
║  ┌───────────────┐   ┌───────────────┐                                  ║
║  │  sensor_node  │   │  sensor_node  │                                  ║
║  │  (pelvis)     │   │  (thigh)      │                                  ║
║  │               │   │               │         BLE Notify               ║
║  │ {id, t, cls,  │   │ {id, t, cls,  │ ──────────────────────►          ║
║  │  probs[3]}    │   │  probs[3]}    │                        │         ║
║  └───────────────┘   └───────────────┘                        │         ║
║                                                                │         ║
║  ┌───────────────┐                               ┌────────────▼───────┐ ║
║  │  sensor_node  │  BLE Notify                   │   host_node        │ ║
║  │  (ankle)      │ ─────────────────────────────►│   (Arduino o PC)   │ ║
║  │ {id, t, cls,  │                               │                    │ ║
║  │  probs[3]}    │                               │  voting_fusion()   │ ║
║  └───────────────┘                               │  → global_class    │ ║
║                                                  │  → confidence      │ ║
║                                                  └────────────────────┘ ║
╚══════════════════════════════════════════════════════════════════════════╝
```

---

## 2. Responsabilidad de cada componente

### Arduino en modo data_logger_node (Fase A)

| Responsabilidad | Detalle |
|----------------|---------|
| Inicializar IMU | Configurar LSM9DS1 a la tasa elegida (p. ej. 119 Hz ODR) |
| Leer IMU | Lectura síncrona en cada ciclo del loop |
| Timestamping local | `millis()` desde el encendido o desde el comando START |
| Formatear mensaje | Línea CSV por muestra: `ts,id,ax,ay,az,gx,gy,gz` |
| Enviar por Serial | 115200 baud, newline como delimitador |
| Responder comandos | `START`, `STOP`, `SET_TRIAL`, `SET_SUBJECT` por Serial |
| LED de estado | Parpadeo durante captura, fijo en espera |

### Arduino en modo sensor_node (Fase C/D)

| Responsabilidad | Detalle |
|----------------|---------|
| Todo lo del data_logger | Lectura IMU + timestamp |
| Buffer circular | Almacena T muestras para la ventana actual |
| Preprocesamiento | Normalización Z-score (media y desviación guardadas en Flash) |
| Inferencia TFLite | Corre el modelo cuantizado INT8 |
| Producir predicción | Clase local + vector de probabilidades softmax |
| Emitir por BLE | GATT Characteristic Notify con paquete de predicción |

### Arduino en modo host_node (Fase D)

| Responsabilidad | Detalle |
|----------------|---------|
| Central BLE | Se conecta a los 3 sensor_nodes como Central BLE |
| Buffer de predicciones | Almacena última predicción de cada nodo |
| Sincronización de ventanas | Agrupa predicciones por índice de ventana compatible |
| Votación | Implementa majority voting y weighted voting |
| Salida global | Serial o pantalla OLED / LED RGB con clase global |
| Fallback | Si un nodo no responde, continúa con los disponibles |

### Host Computer (Fase A y B)

| Responsabilidad | Detalle |
|----------------|---------|
| serial_logger.py | Recibe datos Serial, asigna timestamps del PC, guarda CSV |
| ble_logger.py | Alternativo: recibe por BLE y guarda CSV |
| live_plot.py | Grafica en tiempo real los 6 canales por sensor |
| sync_tools.py | Alineación post-hoc entre señales de distintos sensores |
| Pipeline de entrenamiento | Python + Keras/TF + scikit-learn |
| export_tflite.py | Cuantización y generación del header C |

---

## 3. Flujos de datos

### Flujo 1: IMU → CSV

```
Sensor físico (LSM9DS1)
  │  lectura registros I2C cada ~8.4 ms (119 Hz)
  ▼
Arduino (data_logger_node)
  │  formateo: "1234,1,pelvis,0.12,-0.05,9.81,0.03,-0.01,0.02\n"
  │  envío USB Serial @115200 baud
  ▼
serial_logger.py (Python)
  │  readline() → parseo → añadir timestamp_PC + metadatos trial
  │  append a CSV abierto
  ▼
data/raw/subject_001_trial_001_ankle.csv
```

### Flujo 2: CSV → Entrenamiento offline

```
data/raw/*.csv  +  data/labels/*.csv
  │
load_data.py       → validación, limpieza de filas corruptas
  │
preprocess.py      → filtro paso bajo opcional (Butterworth 20 Hz)
                   → interpolación gaps < 5 ms
                   → normalización (media/std calculadas solo en train)
  │
windowing.py       → ventanas de T muestras con overlap S
                   → asignación de label por mayoría en ventana
                   → descarte de ventanas con >20% de transición
  │
data/processed/
  windows_train.npz  → X:(N_train, T, 6),  y:(N_train,)
  windows_val.npz    → X:(N_val,   T, 6),  y:(N_val,)
  windows_test.npz   → X:(N_test,  T, 6),  y:(N_test,)
  stats.json         → media, std por canal (para normalizar en Arduino)
  │
train_baselines.py / train_cnn1d.py
  │
models/
  logistic_regression.pkl
  random_forest.pkl
  mlp_sklearn.pkl
  cnn1d_best.h5
  │
evaluate.py  →  reports/comparison_table.csv
               reports/confusion_matrices/
               reports/per_subject_accuracy.csv
```

### Flujo 3: Modelo entrenado → TinyML en Arduino

```
models/cnn1d_best.h5  (Keras float32)
  │
export_tflite.py
  │  tf.lite.TFLiteConverter.from_keras_model()
  │  representative_dataset_gen()  →  calibración INT8
  │  optimizations = [OPTIMIZE_FOR_SIZE]
  │  inference_input_type  = INT8
  │  inference_output_type = INT8
  ▼
models/cnn1d_int8.tflite  (~3–8 KB típico)
  │
  xxd -i models/cnn1d_int8.tflite > firmware/common/model_data.h
  │
firmware/sensor_node/  incluye model_data.h
  │  TensorFlow Lite Micro runtime
  │  MicroInterpreter
  │  tensor_arena[ARENA_SIZE]
  ▼
Arduino ejecuta inferencia local
```

### Flujo 4: Inferencia local → Clasificación global

```
sensor_node (pelvis)    sensor_node (thigh)    sensor_node (ankle)
  │  clase local           │  clase local           │  clase local
  │  probs[3]              │  probs[3]              │  probs[3]
  │  window_idx            │  window_idx            │  window_idx
  │  sensor_id=1           │  sensor_id=2           │  sensor_id=3
  │                        │                        │
  └──────── BLE Notify ────┴──────── BLE Notify ────┘
                                    │
                            host_node / PC
                                    │
                            voting_fusion()
                                    │
                          global_class + confidence
```

### Flujo 5: Modelo offline → Dataset de fusión avanzada (Fase D tardía)

```
sensor_node envía:
  probs_s1[3], probs_s2[3], probs_s3[3]
  + features_stats[6]  (opcional)
  → vector de entrada [9..15 features]

host entrena:
  LogisticRegression o MLP pequeño
  sobre este vector
  → modelo de fusión (puede vivir en PC o en Arduino host)
```

---

## 4. Estrategia de sincronización temporal

### Problema
Cada Arduino tiene su propio oscilador. El reloj `millis()` puede derivar
varios milisegundos por minuto entre dispositivos. Además, el inicio de cada
Arduino no está coordinado.

### Opción A: Timestamp del PC (recomendada para Fase A / USB)

- El host PC registra `time.monotonic_ns()` en el momento de recibir cada línea.
- El Arduino envía su propio `millis()` como referencia local.
- El CSV almacena ambos: `timestamp_pc_ms` y `timestamp_arduino_ms`.
- La alineación entre sensores se hace usando el `timestamp_pc_ms`.
- **Limitación**: latencia variable en el driver USB/Serial introduce jitter
  de ~1–5 ms. Aceptable para segmentación de ventanas de 500 ms.

### Opción B: Señal de inicio común (START broadcast)

- El PC envía comando `START` simultáneamente a los 3 puertos Serial.
- Cada Arduino inicializa su contador local en ese momento.
- Los timestamps son relativos al mismo evento de inicio.
- **Ventaja**: timestamps internos más consistentes.
- **Limitación**: el `START` no llega exactamente al mismo ciclo en los 3 Arduinos.
  Error típico: <20 ms.

### Opción C: Evento de calibración inicial (recomendado complementar con A)

- Al inicio de cada trial, el sujeto da un golpe fuerte con el pie (heel tap).
- El impacto produce un pico claro en aceleración en todos los sensores.
- En postprocesamiento, se detecta el pico en cada señal y se alinean.
- `sync_tools.py` implementa esta alineación por correlación cruzada.
- **Ventaja**: corrección post-hoc robusta.
- **Costo**: requiere un evento claro al inicio.

### Opción D: Correlación cruzada post-hoc (fallback)

- Si los sensores capturan señales similares (p. ej. norma del acelerómetro),
  se puede estimar el desplazamiento temporal por correlación cruzada.
- Funciona bien en señales rítmicas como la marcha.
- `sync_tools.py` expone `align_by_xcorr(sig1, sig2, fs)`.

### Recomendación práctica

**Para Fase A**: usar Opción A + Opción C.
- Timestamp del PC como referencia principal.
- Heel tap al inicio de cada trial para corrección post-hoc.
- Guardar ambos timestamps en el CSV.

---

## 5. Alternativas de comunicación

### Alternativa 1: USB Serial (cableada)

| Aspecto | Descripción |
|---------|-------------|
| Velocidad | 115200 baud → ~14 KB/s por Arduino |
| Latencia | < 2 ms típico |
| Fiabilidad | Muy alta, sin pérdidas de paquetes |
| Sincronización | Sencilla: PC asigna timestamp al recibir |
| Setup | 3 cables USB, hub USB o 3 puertos del PC |
| Limitaciones | Cables restringen movimiento; ángulo de marcha limitado |
| Debugging | `Serial.print()` directo, muy fácil |
| Dependencia de librería | Solo `Serial.begin()` — zero dependencies |

Throughput estimado para 3 sensores @ 100 Hz:
- Una línea ≈ 60 bytes
- 3 sensores × 100 Hz × 60 bytes = 18 KB/s → muy por debajo del límite

### Alternativa 2: BLE (inalámbrica)

| Aspecto | Descripción |
|---------|-------------|
| Velocidad | BLE 4.2: ~250 KB/s teórico; práctico con Notify: ~20–40 KB/s |
| Latencia | Connection interval típico: 7.5–100 ms |
| Fiabilidad | Posibles retransmisiones, mayor jitter |
| Sincronización | Difícil; requiere sincronización por software |
| Setup | Sin cables, pero gestión de conexiones GATT central/peripheral |
| Limitaciones | Paquete BLE máximo en ATT: 247 bytes (DLE); 20 bytes sin DLE |
| Debugging | Más complejo; necesitar sniffer BLE o app móvil |
| Dependencia | ArduinoBLE library |

Throughput estimado:
- Paquete predicción ≈ 16 bytes (id:1 + ts:4 + class:1 + probs:3×4=12)
- 3 sensores × 4 ventanas/s × 16 bytes = 192 bytes/s → muy manejable
- Datos crudos por BLE es más problemático: 3 × 100 Hz × 28 bytes = 8.4 KB/s

### Comparativa

| Criterio                  | USB Serial       | BLE                    |
|---------------------------|------------------|------------------------|
| Complejidad inicial       | Baja             | Alta                   |
| Fiabilidad de datos       | Muy alta         | Media (pérdidas)       |
| Movilidad del sujeto      | Restringida      | Libre                  |
| Sincronización            | Simple (PC time) | Requiere protocolo     |
| Debugging                 | Muy fácil        | Complejo               |
| Streaming datos crudos    | Excelente        | Marginal               |
| Envío de predicciones     | Bueno            | Excelente              |
| Adecuado para Fase A      | ✓ Recomendado    | No recomendado         |
| Adecuado para Fase C/D    | Posible          | ✓ Recomendado          |

---

## 6. Recomendación para el primer prototipo

**Fase A (adquisición): USB Serial con 3 cables**

Justificación:
1. La calidad y fiabilidad de los datos es la prioridad máxima en Fase A.
2. USB Serial es determinístico: sin pérdidas, sin jitter BLE.
3. Permite sesiones largas sin preocuparse por reconexiones.
4. La restricción de movimiento es aceptable en un pasillo de marcha de 5–10 m.
5. El debugging es trivial: abre un terminal y ves los datos crudos.
6. Migrar a BLE en Fase C es independiente del pipeline de adquisición.

**Fase C/D (TinyML + fusión): BLE**

Una vez validado el pipeline offline y compilado el modelo TinyML,
migrar a BLE permite evaluar el sistema en condiciones más realistas.
El volumen de datos a transmitir en Fase D es pequeño (solo predicciones),
lo que hace que BLE sea perfectamente adecuado.

---

## 7. Restricciones de hardware Arduino Nano 33 BLE Sense

| Recurso | Disponible | Presupuesto recomendado |
|---------|-----------|------------------------|
| Flash   | 1 MB      | Modelo TFLite < 50 KB  |
| SRAM    | 256 KB    | tensor_arena < 50 KB   |
| CPU     | 64 MHz (Cortex-M4F) | Inferencia < 50 ms |
| IMU ODR | hasta 952 Hz (accel) | Usar 100–119 Hz |
| BLE     | Bluetooth 5.0 | Connection interval min 7.5 ms |

### Estimación de memoria para CNN 1D mínima

```
Arquitectura: Input[50,6] → Conv1D(16,k=5) → Pool(2) → Conv1D(32,k=3)
              → GAP → Dense(16) → Dense(3) → Softmax

Parámetros:
  Conv1D #1:  5 * 6 * 16 + 16        =   496
  Conv1D #2:  3 * 16 * 32 + 32       = 1,568
  Dense #1:   32 * 16 + 16           =   528
  Dense #2:   16 * 3 + 3             =    51
  TOTAL float32: ~2,643 params × 4B  ≈ 10.6 KB
  TOTAL INT8:    ~2,643 params × 1B  ≈  2.6 KB Flash

Activaciones (INT8):
  Post-Conv1: (50-4) × 16 = 736 bytes
  Post-Pool:   23 × 16    = 368 bytes
  Post-Conv2: (23-2) × 32 = 672 bytes
  GAP output:  32 bytes
  tensor_arena necesario: ~4–8 KB (con overhead TFLite Micro)
```

**Conclusión**: el modelo cabe cómodamente en el hardware disponible.
