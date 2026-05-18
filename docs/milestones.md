# Plan de Implementación por Milestones

## Principios generales

- Cada milestone tiene un criterio de éxito binario: funciona o no funciona.
- No pasar al siguiente milestone sin verificar el actual.
- Los riesgos se mitigan antes de que bloqueen el progreso.
- El código de cada milestone vive en una rama Git separada.

---

## Milestone 1 — Lectura de IMU y Serial print básico

### Objetivo
Verificar que el Arduino Nano 33 BLE Sense puede leer la IMU correctamente
y enviar los 6 canales por Serial al PC.

### Archivos a crear/modificar
```
firmware/data_logger_node/data_logger_node.ino   ← nuevo
firmware/common/config.h                          ← nuevo (SENSOR_ID, PLACEMENT)
```

### Comportamiento esperado del firmware
```
// Loop a ~100 Hz
// Serial.println: "12345,1,ankle,0.12,-0.05,9.81,0.03,-0.01,0.02"
```

### Prueba mínima
1. Abrir Serial Monitor a 115200 baud.
2. Inclinar el Arduino: ax/ay/az deben cambiar coherentemente.
3. Girar el Arduino: gx/gy/gz deben cambiar.
4. En reposo con el eje Z vertical: az ≈ 1.0 g, ax ≈ ay ≈ 0.

### Criterios de éxito
- [ ] Datos visibles en Serial Monitor sin cortes ni basura.
- [ ] ODR estimado ≥ 90 Hz (contar líneas por segundo).
- [ ] az ≈ 1.0 g en reposo (gravedad en eje vertical).
- [ ] Sin errores de IMU en setup.

### Riesgos técnicos
| Riesgo | Probabilidad | Mitigación |
|--------|-------------|-----------|
| IMU no inicializa | Baja | Verificar librería `Arduino_LSM9DS1`, revisar I2C |
| Serial demasiado lento para 100 Hz | Media | Reducir decimales (4→2), usar formato compacto |
| Jitter en el ODR por Serial | Media | Medir con timestamp, ajustar SAMPLE_PERIOD_US |

---

## Milestone 2 — Data logger con un Arduino, guardado en CSV

### Objetivo
Capturar datos de un Arduino por Serial y guardarlos en un CSV bien formateado
usando Python. El pipeline completo: Arduino → PC → archivo.

### Archivos a crear/modificar
```
firmware/data_logger_node/data_logger_node.ino   ← añadir comandos START/STOP
acquisition/serial_logger.py                      ← nuevo
data/raw/                                         ← destino de CSVs
```

### Comportamiento esperado
```bash
python acquisition/serial_logger.py \
  --port /dev/ttyACM0 --sensor_id 3 --placement ankle \
  --subject_id s001 --trial_id t001 \
  --output data/raw/s001_t001_sensor3_ankle.csv
# Arduino espera START
# Script envía START, comienza grabación
# ENTER → STOP → cierra CSV → imprime QC
```

### Prueba mínima
1. Grabar 30 segundos caminando en línea recta.
2. Abrir el CSV en Excel/Python y verificar:
   - 3000 filas ≈ (30s × 100 Hz).
   - Todos los valores en rango físico plausible.
   - Sin filas corruptas (NaN, strings en columnas numéricas).

### Criterios de éxito
- [ ] CSV generado correctamente con todas las columnas.
- [ ] ODR estimado: 95–105 Hz.
- [ ] Gaps > 20 ms: < 1% de las muestras.
- [ ] Script de QC muestra "OK" en todos los checks.
- [ ] El CSV puede cargarse con `pd.read_csv()` sin errores.

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| Buffer overflow Arduino (Serial TX) | Reducir frecuencia o usar `Serial.availableForWrite()` |
| Pérdida de líneas por latencia USB | Aumentar buffer del OS; verificar con conteo de líneas |
| Encoding de caracteres roto | Forzar `encode("utf-8", errors="replace")` en Python |

---

## Milestone 3 — Visualización en vivo de señales

### Objetivo
Poder visualizar en tiempo real los 6 canales de la IMU para verificar la
calidad de la señal antes y durante una sesión de captura.

### Archivos a crear/modificar
```
acquisition/live_plot.py   ← nuevo
```

### Comportamiento esperado
- Ventana matplotlib con 2 subplots: acelerómetro (ax,ay,az) y giroscopio (gx,gy,gz).
- Actualización en tiempo real a ~10 fps (no necesita ser frame-perfect).
- Muestra los últimos 5 segundos de señal (ventana deslizante).

### Implementación sugerida
```python
# Usar matplotlib con animation.FuncAnimation o
# simplemente plt.pause(0.05) en un loop.
# Alternativa más rápida: pyqtgraph (pip install pyqtgraph).
```

### Prueba mínima
1. Iniciar `live_plot.py` mientras caminas.
2. Verificar que se ven picos rítmicos de aceleración (~1–2 Hz para marcha normal).
3. Verificar que el giroscopio responde al movimiento del tobillo.

### Criterios de éxito
- [ ] Gráfica actualizada en tiempo real sin lag significativo.
- [ ] Señal rítmica visible durante la marcha.
- [ ] Sin excepciones durante 2 minutos continuos.

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| matplotlib demasiado lento | Usar pyqtgraph o reducir tasa de actualización |
| El plot bloquea el Serial reader | Usar threading para separar lectura y plot |

---

## Milestone 4 — Data logger multisensor con tres Arduino

### Objetivo
Capturar datos simultáneamente de los tres Arduino (pelvis, thigh, ankle),
guardarlos en CSVs separados con metadatos correctos, y verificar la
sincronización temporal entre sensores.

### Archivos a crear/modificar
```
firmware/data_logger_node/data_logger_node.ino   ← verificar que funciona igual en los 3
firmware/common/config.h                          ← SENSOR_ID diferente por dispositivo
acquisition/serial_logger.py                      ← añadir modo --multi
config/acquisition_config.yaml                    ← puertos de los 3 Arduinos
acquisition/sync_tools.py                         ← nuevo: alineación por xcorr
```

### Procedimiento de configuración
1. Grabar en `config.h` de cada Arduino su `SENSOR_ID` y `PLACEMENT` específicos.
2. Conectar los 3 Arduinos y verificar que el SO asigna `/dev/ttyACM0`, `ACM1`, `ACM2`.
3. Actualizar `acquisition_config.yaml` con los puertos correctos.
4. Ejecutar `serial_logger.py --multi`.

### Prueba mínima
1. Capturar 60 segundos de marcha normal con los 3 sensores.
2. Cargar los 3 CSVs en Python y verificar que los timestamps están alineados
   (diferencia < 50 ms entre el primer y el último sensor en iniciar).
3. Calcular correlación cruzada de `acc_norm` entre los 3 sensores:
   el lag estimado debe ser < 100 ms.

### Criterios de éxito
- [ ] 3 CSVs generados simultáneamente.
- [ ] Cada CSV tiene el `sensor_id` y `placement` correcto.
- [ ] ODR individual: 95–105 Hz en los 3 sensores.
- [ ] `sync_tools.py align_by_xcorr()` converge (no devuelve NaN o lag imposible).
- [ ] Al visualizar los 3 `acc_norm` superpuestos, los picos de marcha se alinean
      visualmente dentro de ±50 ms.

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| El OS no identifica bien los puertos COM | Usar `udev rules` en Linux; en Windows verificar COM# en Device Manager |
| Un Arduino es más lento que los otros | Verificar que todos tienen el mismo firmware compilado |
| Desincronización temporal creciente | Implementar heel-tap sync y corrección por xcorr en postpro |
| Hub USB introduce latencia variable | Usar hub USB 3.0 con alimentación propia; o conectar directo al PC |

---

## Milestone 5 — Protocolo de etiquetado e integración con CSV

### Objetivo
Etiquetar al menos 2 trials de datos capturados y verificar que los labels
se integran correctamente con los CSVs crudos.

### Archivos a crear/modificar
```
data/labels/s001_t001_labels.csv   ← creado manualmente o por auto_label
data/labels/s001_t002_labels.csv
labeling/auto_label_from_ankle.py  ← nuevo (Estrategia 2)
labeling/visualize_labels.py       ← nuevo
```

### Procedimiento
1. Correr `auto_label_from_ankle.py` sobre el CSV del sensor de tobillo.
2. Visualizar el resultado con `visualize_labels.py`.
3. Corregir manualmente los errores en el CSV de labels.
4. Verificar que la distribución de clases es razonable:
   - stance: ~55–65% del tiempo de marcha.
   - swing: ~35–45%.
   - default: presente pero no dominante.

### Prueba mínima
Cargar CSV crudo + labels en Python y verificar que la función
`assign_label_to_window()` produce labels válidos para ventanas de 500 ms.

```python
labels_df = pd.read_csv("data/labels/s001_t001_labels.csv")
# Verificar: no hay gaps > 100ms sin label entre intervalos consecutivos
gaps = labels_df["start_ms"].iloc[1:].values - labels_df["end_ms"].iloc[:-1].values
assert (gaps < 100).all(), "Hay gaps sin etiquetar"
# Verificar distribución
print(labels_df.groupby("label")["duration"].sum() / labels_df["duration"].sum())
```

### Criterios de éxito
- [ ] Al menos 2 trials etiquetados correctamente.
- [ ] Distribución de clases plausible (stance ~60%, swing ~35%, default >5%).
- [ ] Función `assign_label_to_window()` funciona sin errores en los trials de prueba.
- [ ] Visualización muestra alineación coherente entre labels y señal.

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| Auto-detección de peaks falla en señal ruidosa | Ajustar parámetros de `find_peaks`; validar con video corto |
| Labels muy cortos (< 300 ms) que no caben en una ventana | Marcar como default si < 80% de la ventana |
| Distribución muy desbalanceada | Planificar sobre-muestreo o capturar más de la clase escasa |

---

## Milestone 6 — Pipeline offline de segmentación y dataset procesado

### Objetivo
Construir el pipeline completo: CSV crudo + labels → ventanas procesadas → .npz
listo para entrenamiento.

### Archivos a crear/modificar
```
training/src/load_data.py      ← nuevo
training/src/preprocess.py     ← nuevo
training/src/windowing.py      ← nuevo
data/processed/                ← destino
```

### Parámetros del pipeline

| Parámetro | Valor | Justificación |
|-----------|-------|---------------|
| `fs` | 100 Hz | Estándar |
| `window_size` | 50 muestras (500 ms) | ~0.5 ciclos de marcha |
| `overlap` | 50% (25 muestras) | 4 predicciones/segundo |
| `min_label_coverage` | 70% | Ventanas limpias |
| `channels` | 6 (ax,ay,az,gx,gy,gz) por sensor | Completo |

### Split de datos (sin fuga)

```
Estrategia: Leave-One-Subject-Out o split por trial_id

Ejemplo con 3 sujetos × 4 trials:
  train: s001_t001, s001_t002, s001_t003, s002_t001, s002_t002
  val:   s001_t004, s002_t003
  test:  s003_t001, s003_t002  ← sujeto completamente nuevo

NUNCA mezclar ventanas del mismo trial en train y test.
```

### Formato del archivo .npz

```python
np.savez(
    "data/processed/windows_train.npz",
    X=X_train,          # shape: (N, 50, 6) float32
    y=y_train,          # shape: (N,)       int8  [0=stance, 1=swing, 2=default]
    subject_ids=...,    # shape: (N,)       str
    trial_ids=...,      # shape: (N,)       str
    sensor_ids=...,     # shape: (N,)       int8
)
```

### Criterios de éxito
- [ ] Los .npz se cargan sin errores.
- [ ] X.shape[-1] == 6, X.shape[1] == 50.
- [ ] Sin NaN en X ni en y.
- [ ] No hay overlap entre subject_ids de train y test.
- [ ] Distribución de clases imprimida y plausible.
- [ ] `stats.json` contiene media y std por canal (calculados solo sobre train).

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| Señales con gaps interrumpen ventanas | Descartar ventanas que crucen un gap > 50 ms |
| Normalización calculada sobre todo el dataset | SIEMPRE calcular stats solo sobre train |
| Dataset demasiado pequeño para 3 clases | Capturar más datos antes de Milestone 7 |

---

## Milestone 7 — Entrenamiento y comparación de modelos offline

### Objetivo
Entrenar al menos 4 modelos (baseline por reglas, LogReg/RF, MLP, CNN 1D)
y producir una tabla comparativa de métricas.

### Archivos a crear/modificar
```
training/src/features.py          ← nuevo
training/src/train_baselines.py   ← nuevo
training/src/train_cnn1d.py       ← nuevo
training/src/evaluate.py          ← nuevo
training/notebooks/comparison.ipynb ← nuevo
training/reports/                 ← destino de tablas y figuras
```

### Modelos a comparar

| Modelo | Entrada | Librería |
|--------|---------|---------|
| Reglas por umbral | acc_norm, gyro_norm stats | manual |
| Logistic Regression | features manuales [18 features] | sklearn |
| Random Forest | features manuales [18 features] | sklearn |
| MLP (sklearn) | features manuales o ventana aplanada | sklearn |
| CNN 1D | ventana [50, 6] | TensorFlow/Keras |

### Features manuales por canal (×6 canales = 36 features)
```python
features = [mean, std, min, max, energy, rms]
# + features derivadas:
features += [acc_norm_mean, acc_norm_std, gyro_norm_mean, gyro_norm_std]
# Total por sensor: ~40 features
```

### Métricas a reportar

| Métrica | Por qué |
|---------|---------|
| Accuracy global | Referencia básica |
| F1 macro | Justo con clases desbalanceadas |
| F1 por clase | ¿Cuál clase es problemática? |
| Confusión matrix | Visualizar errores sistemáticos |
| Tamaño del modelo (KB) | Relevante para TinyML |
| Latencia inferencia (ms) | Relevante para tiempo real |
| Accuracy por sujeto | ¿Generaliza a nuevos sujetos? |
| Accuracy por sensor | ¿Cuál sensor aporta más? |

### Experimentos adicionales
```
Experimento A: sensor_ankle solo vs sensor_pelvis solo vs sensor_thigh solo
Experimento B: fusión temprana (18 canales) vs mejor sensor individual
Experimento C: fusión tardía (promedio de probabilidades de 3 CNN locales)
```

### Criterios de éxito
- [ ] Al menos 4 modelos entrenados y evaluados.
- [ ] Tabla comparativa generada en `reports/model_comparison.csv`.
- [ ] Matrices de confusión guardadas en `reports/confusion_matrices/`.
- [ ] CNN 1D alcanza F1 macro > 0.75 en el set de test.
- [ ] Se identifica qué sensor individual es más informativo.
- [ ] Se documenta si la fusión multisensor mejora sobre el mejor sensor individual.

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| Overfitting severo en CNN | Usar dropout, batch normalization, data augmentation |
| Clase default muy difícil | Revisar labels, añadir más datos de default, ajustar umbral |
| Generalización pobre entre sujetos | Estrategia LOSO; normalización por sujeto en test |
| CNN peor que Random Forest | Posible con dataset pequeño; RF puede ser el modelo TinyML |

---

## Milestone 8 — Cuantización y exportación a TFLite Micro

### Objetivo
Exportar el mejor modelo a formato TensorFlow Lite con cuantización INT8,
verificar que la precisión no degrada significativamente, y generar el
header C para incluir en Arduino.

### Archivos a crear/modificar
```
training/src/export_tflite.py   ← nuevo
training/models/cnn1d_int8.tflite
firmware/common/model_data.h    ← generado por xxd
```

### Pipeline de exportación

```python
# export_tflite.py — esquema

import tensorflow as tf
import numpy as np

def export_int8(keras_model, representative_data_gen, output_path):
    converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_data_gen
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type  = tf.int8
    converter.inference_output_type = tf.int8

    tflite_model = converter.convert()

    with open(output_path, "wb") as f:
        f.write(tflite_model)

    print(f"Modelo exportado: {len(tflite_model)/1024:.1f} KB")
    return tflite_model

def verify_accuracy(tflite_path, X_test, y_test, stats_json):
    """
    Correr inferencia con TFLite interpreter en Python y comparar
    accuracy vs el modelo Keras original.
    """
    import json
    stats = json.load(open(stats_json))

    interpreter = tf.lite.Interpreter(model_path=tflite_path)
    interpreter.allocate_tensors()
    inp = interpreter.get_input_details()[0]
    out = interpreter.get_output_details()[0]

    # Cuantizar entrada
    scale, zp = inp["quantization"]
    X_int8 = (X_test / scale + zp).astype(np.int8)

    preds = []
    for i in range(len(X_int8)):
        interpreter.set_tensor(inp["index"], X_int8[i:i+1])
        interpreter.invoke()
        raw = interpreter.get_tensor(out["index"])
        # Descuantizar salida
        out_scale, out_zp = out["quantization"]
        probs = (raw.astype(np.float32) - out_zp) * out_scale
        preds.append(np.argmax(probs))

    acc = np.mean(np.array(preds) == y_test)
    print(f"Accuracy TFLite INT8: {acc:.3f}")
    return acc
```

### Criterios de éxito
- [ ] Modelo TFLite generado < 50 KB.
- [ ] Accuracy TFLite INT8 ≥ (Accuracy Keras float32) - 0.03 (degradación < 3%).
- [ ] `model_data.h` generado correctamente (verificar con `wc -l`).
- [ ] Tamaño del tensor_arena estimado con `MicroInterpreter` en test de escritorio.

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| Degradación > 5% por cuantización | Más datos en representative_dataset; revisar capas con alta varianza |
| Operaciones no soportadas por TFLite Micro | Verificar ops compatibles antes de diseñar la arquitectura |
| model_data.h demasiado grande para Flash | Simplificar modelo; verificar con `avr-nm` o `arm-none-eabi-size` |

---

## Milestone 9 — Inferencia TinyML local en un Arduino

### Objetivo
El sensor_node ejecuta el modelo cuantizado localmente, produce predicciones
en tiempo real y las imprime por Serial.

### Archivos a crear/modificar
```
firmware/sensor_node/sensor_node.ino   ← nuevo
firmware/common/model_data.h           ← del M8
firmware/common/normalization.h        ← media/std desde stats.json
```

### Arquitectura del sensor_node

```
loop():
  1. Leer IMU → sample[6]
  2. Escribir en circular_buffer[T][6]
  3. Si buffer lleno (cada 25 muestras con overlap 50%):
       a. Normalizar: x_norm = (x - mean) / std  (operación float)
       b. Cuantizar:  x_int8 = (x_norm / input_scale + input_zero_point)
       c. Copiar a input_tensor
       d. interpreter->Invoke()
       e. Leer output_tensor → probs[3]
       f. predicted_class = argmax(probs)
       g. Serial.print o BLE.notify(predicción)
```

### Configuración de memoria

```cpp
// sensor_node.ino
const int TENSOR_ARENA_SIZE = 8 * 1024;  // 8 KB — ajustar según modelo
uint8_t tensor_arena[TENSOR_ARENA_SIZE];

const int WINDOW_SIZE   = 50;
const int NUM_CHANNELS  = 6;
const int WINDOW_STEP   = 25;  // overlap 50%

float window_buffer[WINDOW_SIZE][NUM_CHANNELS];
int   buffer_idx = 0;
```

### Criterios de éxito
- [ ] Firmware compila sin errores.
- [ ] Inferencia completa en < 50 ms (medir con `micros()` antes y después).
- [ ] Predicciones impresas por Serial: "stance", "swing", "default".
- [ ] Predicciones son razonables: durante caminata se ven alternancia stance/swing.
- [ ] RAM libre después de cargar el modelo: > 50 KB (verificar con `freeMemory()`).

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| `tensor_arena` insuficiente | Aumentar tamaño; usar `RecordingMicroAllocator` para medir exacto |
| Modelo no carga (magic number error) | Verificar que el .h fue generado correctamente |
| Inferencia > 100 ms | Simplificar modelo (menos filtros, menos capas) |
| Predicciones siempre "default" | Verificar normalización; escala de cuantización |

---

## Milestone 10 — Comunicación sensor_node → host por Serial/BLE

### Objetivo
El sensor_node envía su predicción local estructurada a un host (PC o Arduino).
Separar la predicción de los datos crudos.

### Formato del paquete de predicción

```
// Formato texto (Serial, para debugging):
"PRED:sensor_id,window_idx,predicted_class,prob_stance,prob_swing,prob_default\n"

// Ejemplo:
"PRED:3,42,0,0.82,0.12,0.06\n"
//        |  |  |   ^--- prob_stance=0.82
//        |  |  +-------- predicted_class=0 (stance)
//        |  +------------ window_idx=42
//        +--------------- sensor_id=3

// Formato binario (BLE Notify, 16 bytes):
// [uint8:sensor_id][uint32:window_idx][uint8:class][float32:ps][float32:psw][float32:pd]
```

### Archivos a crear/modificar
```
firmware/sensor_node/sensor_node.ino   ← añadir BLE advertising y notify
firmware/host_node/host_node.ino       ← nuevo (o script Python como host)
acquisition/prediction_receiver.py    ← nuevo (host en Python para debugging)
```

### Criterios de éxito
- [ ] host_node o prediction_receiver.py recibe paquetes de predicción.
- [ ] No hay pérdida de paquetes > 5% en 60 segundos.
- [ ] Latencia de predicción a recepción < 100 ms.
- [ ] Formato de paquete parseado correctamente en el host.

### Riesgos técnicos
| Riesgo | Mitigación |
|--------|-----------|
| BLE: connection drops durante movimiento | Añadir reconexión automática; decrementar connection interval |
| BLE: MTU insuficiente para el paquete | Habilitar DLE (Data Length Extension); o comprimir paquete |
| Desincronización de window_idx entre sensores | Usar timestamp en lugar de contador relativo |

---

## Milestone 11 — Tres sensores enviando predicciones al host

### Objetivo
Los 3 sensor_nodes envían predicciones simultáneamente al host_node.
El host acumula predicciones de los 3 sensores correctamente.

### Archivos a crear/modificar
```
firmware/host_node/host_node.ino   ← añadir gestión de 3 conexiones BLE
```

### Desafíos de coordinación
- Arduino BLE Central puede gestionar múltiples periféricos simultáneamente
  (limitado por la pila nRF52840: hasta ~8 conexiones).
- El host debe identificar cada paquete por `sensor_id`.
- Si un sensor no ha enviado predicción en los últimos 2 ventanas, marcar como ausente.

### Criterios de éxito
- [ ] host_node muestra predicciones de sensores 1, 2 y 3 en tiempo real.
- [ ] Tasa de predicción de cada sensor: ~4/segundo.
- [ ] El host detecta correctamente cuando un sensor se desconecta.
- [ ] Sin crashes ni reinicios durante 5 minutos continuos.

---

## Milestone 12 — Votación simple en el host

### Objetivo
Implementar majority voting y weighted voting en el host_node.
Producir una clasificación global de la fase de marcha.

### Algoritmos

```python
# Majority voting simple
def majority_vote(preds):
    from collections import Counter
    votes = Counter(preds)
    most_common = votes.most_common(1)[0]
    if most_common[1] >= 2:  # mayoría simple (2 de 3)
        return most_common[0]
    else:
        return "default"

# Weighted voting (por confianza softmax)
def weighted_vote(preds, probs):
    # probs[i] = [prob_stance, prob_swing, prob_default]
    sums = [0.0, 0.0, 0.0]
    for i, p in enumerate(probs):
        conf = max(p)  # confianza = máxima probabilidad
        sums[preds[i]] += conf
    best = np.argmax(sums)
    if sums[best] > THRESHOLD:
        return best
    else:
        return 2  # default

# Regla de fallback
# Si algún sensor tiene confianza < 0.5, reducir peso de su voto.
# Si ningún sensor tiene confianza > 0.6, devolver default.
```

### Archivos a crear/modificar
```
firmware/host_node/host_node.ino         ← añadir voting_fusion()
firmware/host_node/voting_fusion.h       ← nuevo
acquisition/prediction_receiver.py      ← añadir voting en modo test
```

### Criterios de éxito
- [ ] La salida global alterna coherentemente entre stance y swing durante la marcha.
- [ ] La tasa de "default" es < 20% durante marcha normal en línea recta.
- [ ] Cuando se desconecta un sensor, el sistema continúa con degradación graceful.
- [ ] Latencia total (IMU → clasificación global): < 600 ms.

---

## Milestone 13 — Fusión avanzada con probabilidades

### Objetivo
Evaluar si un segundo clasificador entrenado sobre el vector de probabilidades
de los 3 sensores mejora sobre la votación simple.

### Metodología

1. Grabar un nuevo dataset con los 3 sensor_nodes corriendo.
2. Guardar el vector `[ps1,pw1,pd1, ps2,pw2,pd2, ps3,pw3,pd3]` junto con el label real.
3. Entrenar un meta-clasificador offline: LogisticRegression o MLP pequeño.
4. Evaluar si mejora sobre el majority voting del M12.
5. Si mejora, exportar a TFLite y ejecutar en el host_node.

### Criterios de éxito
- [ ] Meta-clasificador entrenado con datos reales de los 3 sensores.
- [ ] Mejora de F1 macro ≥ 0.03 sobre majority voting en test set.
- [ ] Meta-clasificador exportable a TFLite (< 5 KB).
- [ ] Documentado en `docs/experiments.md`.

### Nota
Este milestone es **condicional**: si el majority voting del M12 ya alcanza
resultados clínicamente aceptables (F1 macro > 0.90), el M13 puede posponerse
en favor de extender el sistema hacia más clases de marcha (heel strike, etc.).

---

## Resumen visual de milestones

```
M1  ──► M2  ──► M3    Fase A: adquisición 1 sensor
             │
             ▼
            M4          Fase A: adquisición 3 sensores
             │
             ▼
            M5          Etiquetado
             │
             ▼
            M6          Dataset offline
             │
             ▼
            M7          Comparación de modelos
             │
             ▼
            M8          Exportación TFLite
             │
             ▼
            M9          TinyML 1 Arduino    Fase C
             │
             ▼
           M10          Comunicación predicción
             │
             ▼
           M11          3 sensores al host  Fase D
             │
             ▼
           M12          Votación simple
             │
             ▼
           M13          Fusión avanzada (condicional)
```
