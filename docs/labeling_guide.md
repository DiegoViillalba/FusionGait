# Guía de Etiquetado de Fases de la Marcha

## Definición de clases

| Clase     | Criterio biomecánico                                              |
|-----------|------------------------------------------------------------------|
| `stance`  | El pie está en contacto con el suelo                             |
| `swing`   | El pie está en el aire (desde toe-off hasta heel strike)         |
| `default` | Transición ambigua, pausa, giro, arranque, frenada, o estático   |

**Regla general**: ante la duda, asignar `default`.
El modelo debe aprender a ignorar casos difíciles, no a sobre-ajustarse a ellos.

---

## Estrategia 1: Etiquetado manual con video sincronizado

### Procedimiento

1. **Grabación**: colocar una cámara lateral a ~2 m del sujeto, con campo visual
   que cubra al menos un ciclo completo de marcha.

2. **Sincronización temporal**:
   - Al inicio de cada trial, el sujeto da una palmada fuerte.
   - La palmada produce un pico de aceleración visible en los sensores
     Y un sonido sincronizable en el audio del video.
   - Se usa este evento para alinear el video con los datos de la IMU.

3. **Análisis de video**:
   - Herramienta recomendada: **ELAN** (gratuito, open-source, multiplataforma)
     o simplemente VLC con notas de tiempo.
   - El analista marca con timestamps los eventos:
     - heel strike (HS): cuando el talón toca el suelo
     - toe off (TO): cuando los dedos del pie se despegan del suelo
   - Cada intervalo HS→TO = `stance`
   - Cada intervalo TO→HS = `swing`
   - Todo lo demás = `default`

4. **Exportar anotaciones**: como CSV con columnas `start_ms, end_ms, label`.

### Pros y contras

| Pro | Contra |
|-----|--------|
| Alta precisión biomecánica | Requiere revisión manual frame a frame |
| Puede cubrir ambos pies | Lento: ~1h de análisis por 5 min de marcha |
| Permite validar el sensor | Necesita cámara + software de anotación |

---

## Estrategia 2: Etiquetado semiautomático por señal del tobillo

El sensor del tobillo (sensor_id=3) es el que mejor captura los eventos de
impacto (heel strike) y despegue (toe off) porque está más cerca del pie.

### Algoritmo de detección automática de eventos

```python
# labeling/auto_label_from_ankle.py

import pandas as pd
import numpy as np
from scipy.signal import find_peaks, butter, filtfilt

def detect_gait_events(df_ankle: pd.DataFrame,
                       fs: float = 100.0) -> pd.DataFrame:
    """
    Detecta heel strikes y toe-offs en la señal del sensor de tobillo.

    Retorna DataFrame con columnas: event_type, timestamp_ms
    """

    # 1. Calcular norma de aceleración
    df_ankle["acc_norm"] = np.sqrt(
        df_ankle.ax**2 + df_ankle.ay**2 + df_ankle.az**2
    )

    # 2. Filtro paso bajo para suavizar ruido de alta frecuencia
    b, a = butter(4, 20 / (fs/2), btype="low")
    acc_smooth = filtfilt(b, a, df_ankle["acc_norm"].values)

    # 3. Heel strike: pico de aceleración (impacto del talón)
    hs_peaks, _ = find_peaks(
        acc_smooth,
        height=1.5,      # mínimo 1.5g
        distance=int(0.4 * fs),  # mínimo 400ms entre picos (máx 2.5 pasos/s)
        prominence=0.5
    )

    # 4. Toe off: mínimo local de aceleración antes de cada HS
    # (el pie se aligera antes de despegarse)
    toe_offs = []
    for hs in hs_peaks:
        search_start = max(0, hs - int(0.3 * fs))
        local_segment = acc_smooth[search_start:hs]
        if len(local_segment) > 0:
            to_local = np.argmin(local_segment)
            toe_offs.append(search_start + to_local)

    # 5. Construir DataFrame de eventos
    events = []
    ts = df_ankle["timestamp_pc_ms"].values

    for idx in hs_peaks:
        events.append({"event_type": "heel_strike",
                       "timestamp_ms": ts[idx], "sample_idx": idx})
    for idx in toe_offs:
        events.append({"event_type": "toe_off",
                       "timestamp_ms": ts[idx], "sample_idx": idx})

    events_df = pd.DataFrame(events).sort_values("timestamp_ms").reset_index(drop=True)
    return events_df


def events_to_labels(events_df: pd.DataFrame,
                     total_duration_ms: float) -> pd.DataFrame:
    """
    Convierte una lista de eventos en intervalos etiquetados.
    Retorna DataFrame con columnas: start_ms, end_ms, label
    """
    labels = []
    events = events_df.to_dict("records")

    for i in range(len(events) - 1):
        e_curr = events[i]
        e_next = events[i+1]

        if e_curr["event_type"] == "heel_strike":
            # HS → TO: stance
            if e_next["event_type"] == "toe_off":
                labels.append({
                    "start_ms": e_curr["timestamp_ms"],
                    "end_ms":   e_next["timestamp_ms"],
                    "label":    "stance"
                })
        elif e_curr["event_type"] == "toe_off":
            # TO → HS: swing
            if e_next["event_type"] == "heel_strike":
                labels.append({
                    "start_ms": e_curr["timestamp_ms"],
                    "end_ms":   e_next["timestamp_ms"],
                    "label":    "swing"
                })

    # Gaps entre eventos = default
    all_intervals = sorted(labels + [
        {"start_ms": 0, "end_ms": 0, "label": "boundary"},
        {"start_ms": total_duration_ms, "end_ms": total_duration_ms, "label": "boundary"}
    ], key=lambda x: x["start_ms"])

    return pd.DataFrame(labels).sort_values("start_ms").reset_index(drop=True)
```

### Pros y contras

| Pro | Contra |
|-----|--------|
| Rápido: automático una vez programado | Requiere validación manual posterior |
| Reproducible | Errores en detección de picos = labels incorrectos |
| Escalable a muchos trials | Asume que el tobillo está instrumentado |
| Bueno para marcha en línea recta | Puede fallar en giros o terreno irregular |

**Recomendación**: usar como primer intento; validar con video para las primeras
sesiones y ajustar los parámetros de `find_peaks`.

---

## Estrategia 3: Botón de marcado manual en tiempo real

### Procedimiento

Un operador (o el propio sujeto con un botón adicional) presiona un botón
físico conectado al Arduino del tobillo en los momentos de transición.

```cpp
// En data_logger_node.ino — añadir botón
const int BTN_PIN = 2;
volatile bool btn_event = false;

void setup() {
  // ...
  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), on_button, FALLING);
}

void on_button() {
  btn_event = true;
}

void loop() {
  // ...
  if (btn_event) {
    btn_event = false;
    Serial.print("EVENT:BUTTON:");
    Serial.println(millis());
  }
}
```

El script Python registra los timestamps del botón. Luego se asignan
labels a intervalos alternos: stance → swing → stance → swing, etc.,
partiendo del primer evento.

### Pros y contras

| Pro | Contra |
|-----|--------|
| Sin video necesario | Requiere segundo operador o sujeto habilidoso |
| Tiempo real | Latencia humana (~200 ms) introduce error |
| Simple hardware | No distingue cuál pie |

---

## Estrategia recomendada para primera versión

**Estrategia 2 + validación manual** es la más práctica:

1. Capturar datos con el data_logger.
2. Correr `auto_label_from_ankle.py` para generar labels automáticos.
3. Visualizar con `live_plot.py` superponiendo labels y señal.
4. Corregir manualmente los errores evidentes en el CSV de labels.
5. Usar el CSV corregido para el entrenamiento.

---

## Formato del archivo de labels

```
data/labels/s001_t001_labels.csv
```

```csv
start_ms,end_ms,label,confidence,notes
0,480,default,high,inicio del trial - sujeto estático
480,1230,stance,high,apoyo derecho
1230,1650,swing,high,oscilación derecha
1650,2390,stance,high,apoyo izquierdo
2390,2810,swing,high,oscilación izquierda
2810,2950,default,low,transición - posible giro
2950,3600,stance,medium,apoyo con pequeño artefacto en ms 3200
```

Columnas opcionales: `confidence` (high/medium/low) y `notes` (texto libre).

---

## Asignación de labels a ventanas temporales

Una vez que tenemos el CSV de labels por intervalos, debemos asignar
una etiqueta a cada ventana del pipeline offline.

```python
# training/src/windowing.py — fragmento de asignación de labels

def assign_label_to_window(window_start_ms: float,
                           window_end_ms: float,
                           labels_df: pd.DataFrame,
                           min_coverage: float = 0.7) -> str:
    """
    Asigna una etiqueta a una ventana temporal.

    Reglas:
    1. Si una clase cubre >= min_coverage (70%) de la ventana → esa clase.
    2. Si no hay clase dominante → "default".
    3. Si la ventana está en una transición → "default".
    """
    window_len = window_end_ms - window_start_ms
    coverage = {"stance": 0.0, "swing": 0.0, "default": 0.0}

    for _, row in labels_df.iterrows():
        overlap_start = max(window_start_ms, row["start_ms"])
        overlap_end   = min(window_end_ms,   row["end_ms"])
        overlap = max(0, overlap_end - overlap_start)
        if overlap > 0:
            coverage[row["label"]] = coverage.get(row["label"], 0) + overlap

    # Normalizar
    total = sum(coverage.values())
    if total == 0:
        return "default"

    for k in coverage:
        coverage[k] /= window_len

    best_label  = max(coverage, key=coverage.get)
    best_coverage = coverage[best_label]

    if best_coverage >= min_coverage:
        return best_label
    else:
        return "default"  # ventana de transición
```

### Visualización de labels sobre señal

```python
# training/notebooks/visualize_labels.ipynb (esquema)
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

COLORS = {"stance": "blue", "swing": "green", "default": "gray"}

fig, axes = plt.subplots(2, 1, figsize=(15, 6), sharex=True)

# Panel superior: acc_norm
df["acc_norm"] = np.sqrt(df.ax**2 + df.ay**2 + df.az**2)
axes[0].plot(df["timestamp_pc_ms"], df["acc_norm"], "k-", lw=0.8)
axes[0].set_ylabel("acc_norm (g)")

# Panel inferior: gyro_norm
df["gyro_norm"] = np.sqrt(df.gx**2 + df.gy**2 + df.gz**2)
axes[1].plot(df["timestamp_pc_ms"], df["gyro_norm"], "purple", lw=0.8)
axes[1].set_ylabel("gyro_norm (°/s)")
axes[1].set_xlabel("Tiempo (ms)")

# Sombreado de intervalos por label
for _, row in labels_df.iterrows():
    color = COLORS.get(row["label"], "gray")
    for ax in axes:
        ax.axvspan(row["start_ms"], row["end_ms"],
                   alpha=0.3, color=color)

plt.tight_layout()
plt.savefig("reports/label_visualization.png", dpi=150)
```

---

## Estructura de archivos completa

```
data/
├── raw/
│   ├── s001_t001_sensor1_pelvis.csv
│   ├── s001_t001_sensor2_thigh.csv
│   ├── s001_t001_sensor3_ankle.csv
│   ├── s001_t002_sensor1_pelvis.csv
│   ├── s001_t002_sensor2_thigh.csv
│   ├── s001_t002_sensor3_ankle.csv
│   ├── s002_t001_sensor1_pelvis.csv
│   └── ...
├── labels/
│   ├── s001_t001_labels.csv
│   ├── s001_t002_labels.csv
│   ├── s002_t001_labels.csv
│   └── ...
└── processed/
    ├── windows_train.npz   ← arrays X:(N,T,C) y:(N,) meta:(N,)
    ├── windows_val.npz
    ├── windows_test.npz
    └── stats.json          ← {"mean": [...], "std": [...]} por canal
```

---

## Manejo de la clase Default

La clase `default` es un "catch-all" que absorbe:

| Situación | Por qué es default |
|-----------|--------------------|
| Inicio/fin de trial (sujeto estático) | No es marcha |
| Transición entre stance y swing | Ventana mixta |
| Giro de 180° al final del pasillo | Patrón atípico |
| Artefactos de movimiento de cables | Señal corrupta |
| Colocación del pie fuera de protocolo | Patrón inusual |
| Sentarse/levantarse | No es marcha normal |

**Importancia del default como clase de seguridad**: si el modelo no tiene
`default`, clasificará todo como stance o swing aunque la señal sea ambigua.
Esto puede generar falsos positivos que distorsionen el análisis clínico.

**Balanceo de clases**: en la práctica, `default` puede ser escaso si los
trials son largas caminatas continuas. Estrategias:
1. Incluir explícitamente segmentos estáticos en el protocolo experimental.
2. Sobre-muestrear `default` con data augmentation (ruido gaussiano, escala).
3. Usar `class_weight` en el entrenamiento para compensar el desbalance.
