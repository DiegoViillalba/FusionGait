# Protocolo de Adquisición de Datos

## 1. Parámetros de captura

### Frecuencia de muestreo

| Parámetro | Valor recomendado | Justificación |
|-----------|------------------|---------------|
| ODR IMU   | 119 Hz (LSM9DS1 nativo) | Mayor ODR nativo ≤ 200 Hz sin overshooting |
| ODR efectivo | 100 Hz (decimación × 1.19) | Número redondo, estándar en literatura de marcha |
| Duración ciclo de marcha | ~1000–1200 ms | ~100–120 muestras/ciclo completo |
| Duración stance | ~600–700 ms | ~60–70 muestras |
| Duración swing  | ~400–500 ms | ~40–50 muestras |
| Ventana de clasificación | 500 ms = 50 muestras @ 100 Hz | Captura ~0.5 ciclos |
| Overlap de ventana | 50% = 250 ms = 25 muestras | 4 predicciones/segundo |

**Nota sobre la IMU LSM9DS1**: el ODR del acelerómetro y giroscopio se configura
independientemente. Usar `IMU.begin()` con Arduino_LSM9DS1 library configura por
defecto acelerómetro a 119 Hz y giroscopio a 119 Hz.

### Rangos de medición sugeridos

| Sensor | Rango | Resolución efectiva |
|--------|-------|-------------------|
| Acelerómetro | ±4 g (default) | 0.122 mg/LSB |
| Giroscopio   | ±2000 °/s      | 70 mdps/LSB   |

Para marcha normal: ±4 g y ±2000 °/s son suficientes.

---

## 2. Formato de mensaje Serial (Fase A)

### Protocolo de línea

Cada muestra se envía como una línea CSV terminada en `\n`:

```
<timestamp_ms>,<sensor_id>,<placement>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n
```

Ejemplo:
```
12345,1,ankle,0.1234,-0.0521,9.8134,0.0312,-0.0145,0.0223
12354,1,ankle,0.1198,-0.0534,9.8121,0.0298,-0.0152,0.0211
```

### Comandos del PC al Arduino

| Comando | Formato | Acción |
|---------|---------|--------|
| Iniciar captura | `START\n` | Comienza envío de datos |
| Detener captura | `STOP\n` | Pausa envío, mantiene conexión |
| Configurar trial | `SET_TRIAL:001\n` | El Arduino incluye trial_id en encabezado |
| Configurar sujeto | `SET_SUBJECT:s001\n` | Ídem para subject_id |
| Pulso de sync | `SYNC\n` | Arduino responde `SYNC_ACK:<millis>\n` |
| Estado | `STATUS\n` | Arduino responde con config actual |

### Respuestas especiales del Arduino

```
READY\n                   → Arduino iniciado, esperando START
SYNC_ACK:12345\n          → Respuesta a SYNC, con millis() actual
ERROR:IMU_NOT_FOUND\n     → Error de hardware
```

### Justificación del formato

- CSV plano es legible directamente, fácil de parsear con `str.split(',')`
- Sin encabezado en cada línea (eficiencia)
- El encabezado del CSV lo genera el script Python
- `timestamp_ms` en el Arduino permite calcular jitter y verificar ODR real

---

## 3. Esquema del CSV resultante

```
timestamp_pc_ms, timestamp_arduino_ms, sensor_id, placement,
ax, ay, az, gx, gy, gz, label, trial_id, subject_id
```

| Columna | Tipo | Descripción |
|---------|------|-------------|
| `timestamp_pc_ms` | int64 | Epoch milisegundos en el PC al recibir la muestra |
| `timestamp_arduino_ms` | int32 | `millis()` en el Arduino al tomar la muestra |
| `sensor_id` | int8 | 1=pelvis, 2=thigh, 3=ankle |
| `placement` | str | "pelvis", "thigh", "ankle" (o custom) |
| `ax` | float32 | Aceleración X en g |
| `ay` | float32 | Aceleración Y en g |
| `az` | float32 | Aceleración Z en g |
| `gx` | float32 | Velocidad angular X en °/s |
| `gy` | float32 | Velocidad angular Y en °/s |
| `gz` | float32 | Velocidad angular Z en °/s |
| `label` | str | "stance", "swing", "default" o vacío |
| `trial_id` | str | "t001", "t002", ... |
| `subject_id` | str | "s001", "s002", ... |

Ejemplo de filas:

```csv
timestamp_pc_ms,timestamp_arduino_ms,sensor_id,placement,ax,ay,az,gx,gy,gz,label,trial_id,subject_id
1716000000000,12345,3,ankle,0.1234,-0.0521,9.8134,0.0312,-0.0145,0.0223,,t001,s001
1716000000010,12354,3,ankle,0.1198,-0.0534,9.8121,0.0298,-0.0152,0.0211,,t001,s001
```

---

## 4. Pseudocódigo: data_logger_node (Arduino)

```cpp
// data_logger_node.ino
// Responsabilidad: leer IMU y enviar datos por Serial al PC

#include <Arduino_LSM9DS1.h>

// Estado del data logger
bool capturing = false;
String trial_id = "t000";
String subject_id = "s000";
String placement = "unknown";
int sensor_id = 0;

// Timing
unsigned long last_sample_us = 0;
const unsigned long SAMPLE_PERIOD_US = 10000;  // 100 Hz = 10,000 µs

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("ERROR:IMU_NOT_FOUND");
    while (true);
  }

  // Leer sensor_id y placement desde EEPROM o hardcoded
  sensor_id = SENSOR_ID;    // definido en config.h por cada nodo
  placement = PLACEMENT;

  Serial.println("READY");
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Procesar comandos entrantes
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    handle_command(cmd);
  }

  // Capturar y enviar muestra si está activo
  if (capturing) {
    unsigned long now = micros();
    if (now - last_sample_us >= SAMPLE_PERIOD_US) {
      last_sample_us = now;
      send_sample();
    }
  }

  // Parpadeo LED durante captura
  update_led();
}

void handle_command(String cmd) {
  if (cmd == "START") {
    capturing = true;
  } else if (cmd == "STOP") {
    capturing = false;
  } else if (cmd.startsWith("SET_TRIAL:")) {
    trial_id = cmd.substring(10);
  } else if (cmd.startsWith("SET_SUBJECT:")) {
    subject_id = cmd.substring(12);
  } else if (cmd == "SYNC") {
    Serial.print("SYNC_ACK:");
    Serial.println(millis());
  } else if (cmd == "STATUS") {
    Serial.print("STATUS:sensor_id="); Serial.print(sensor_id);
    Serial.print(",placement="); Serial.print(placement);
    Serial.print(",trial="); Serial.print(trial_id);
    Serial.print(",capturing="); Serial.println(capturing ? "1" : "0");
  }
}

void send_sample() {
  float ax, ay, az, gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Formato: ts_arduino,sensor_id,placement,ax,ay,az,gx,gy,gz
    Serial.print(millis());    Serial.print(',');
    Serial.print(sensor_id);  Serial.print(',');
    Serial.print(placement);  Serial.print(',');
    Serial.print(ax, 4);      Serial.print(',');
    Serial.print(ay, 4);      Serial.print(',');
    Serial.print(az, 4);      Serial.print(',');
    Serial.print(gx, 4);      Serial.print(',');
    Serial.print(gy, 4);      Serial.print(',');
    Serial.println(gz, 4);    // println añade \n
  }
}

void update_led() {
  if (capturing) {
    digitalWrite(LED_BUILTIN, (millis() / 200) % 2);  // parpadeo 5 Hz
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}
```

---

## 5. Pseudocódigo: serial_logger.py

```python
# acquisition/serial_logger.py
# Uso: python serial_logger.py --port /dev/ttyACM0 --sensor_id 1
#                               --placement ankle --subject_id s001
#                               --trial_id t001 --output data/raw/...csv

import serial
import csv
import time
import argparse
import threading
from pathlib import Path


COLUMNS = [
    "timestamp_pc_ms", "timestamp_arduino_ms",
    "sensor_id", "placement",
    "ax", "ay", "az", "gx", "gy", "gz",
    "label", "trial_id", "subject_id"
]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port",       required=True)
    p.add_argument("--baud",       default=115200, type=int)
    p.add_argument("--sensor_id",  required=True, type=int)
    p.add_argument("--placement",  required=True)
    p.add_argument("--subject_id", required=True)
    p.add_argument("--trial_id",   required=True)
    p.add_argument("--output",     required=True)
    p.add_argument("--label",      default="",
                   help="Label fijo para toda la sesión, o vacío para etiquetar después")
    return p.parse_args()


def send_command(ser, cmd):
    ser.write((cmd + "\n").encode())
    time.sleep(0.05)


def logger_loop(ser, writer, args, stats):
    """Loop de lectura en hilo separado."""
    while stats["running"]:
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
        except serial.SerialException:
            break

        if not line or line.startswith("ERROR") or line.startswith("READY"):
            print(f"[Arduino] {line}")
            continue

        if line.startswith("SYNC_ACK") or line.startswith("STATUS"):
            print(f"[Arduino] {line}")
            continue

        # Parsear línea de datos
        parts = line.split(",")
        if len(parts) != 9:
            stats["corrupt"] += 1
            continue

        ts_pc = int(time.monotonic_ns() // 1_000_000)
        ts_arduino = parts[0]
        # sensor_id y placement vienen del Arduino, pero también los tenemos
        # en los args para consistencia
        row = {
            "timestamp_pc_ms":      ts_pc,
            "timestamp_arduino_ms": ts_arduino,
            "sensor_id":            parts[1],
            "placement":            parts[2],
            "ax": parts[3], "ay": parts[4], "az": parts[5],
            "gx": parts[6], "gy": parts[7], "gz": parts[8],
            "label":      args.label,
            "trial_id":   args.trial_id,
            "subject_id": args.subject_id,
        }
        writer.writerow(row)
        stats["samples"] += 1

        if stats["samples"] % 500 == 0:
            print(f"  {stats['samples']} muestras | corrupt: {stats['corrupt']}")


def main():
    args = parse_args()
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    ser = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(2)  # esperar reset del Arduino

    # Esperar READY
    ready_line = ser.readline().decode().strip()
    print(f"[Arduino] {ready_line}")

    # Configurar trial y sujeto
    send_command(ser, f"SET_TRIAL:{args.trial_id}")
    send_command(ser, f"SET_SUBJECT:{args.subject_id}")

    # Sync inicial
    send_command(ser, "SYNC")
    sync_resp = ser.readline().decode().strip()
    print(f"[Sync] {sync_resp}")

    stats = {"running": True, "samples": 0, "corrupt": 0}

    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()

        # Iniciar captura
        send_command(ser, "START")
        print(f"[Logger] Capturando → {output_path}")
        print("[Logger] Presiona ENTER para detener...")

        # Hilo de lectura
        t = threading.Thread(target=logger_loop,
                             args=(ser, writer, args, stats))
        t.start()

        input()  # bloquea hasta ENTER
        stats["running"] = False
        send_command(ser, "STOP")
        t.join(timeout=2)

    ser.close()
    print(f"\n[Logger] Finalizado. {stats['samples']} muestras guardadas.")
    print(f"[Logger] Líneas corruptas: {stats['corrupt']}")

    # Verificación rápida
    import pandas as pd
    df = pd.read_csv(output_path)
    ts_diff = df["timestamp_pc_ms"].diff().dropna()
    mean_dt = ts_diff.mean()
    print(f"\n[QC] ODR estimado: {1000/mean_dt:.1f} Hz (esperado ~100 Hz)")
    print(f"[QC] Gaps > 20ms: {(ts_diff > 20).sum()}")
    print(f"[QC] Columnas: {list(df.columns)}")
    print(f"[QC] Filas: {len(df)}")


if __name__ == "__main__":
    main()
```

---

## 6. Pseudocódigo: ble_logger.py

```python
# acquisition/ble_logger.py
# Alternativa inalámbrica — requiere bleak (pip install bleak)
# Uso en Fase C/D para recibir predicciones, no datos crudos

import asyncio
import csv
import time
from bleak import BleakScanner, BleakClient
from pathlib import Path


# UUIDs GATT del sensor_node (definidos en firmware)
SERVICE_UUID    = "12345678-1234-5678-1234-56789abcdef0"
CHAR_UUID_DATA  = "12345678-1234-5678-1234-56789abcdef1"  # datos crudos
CHAR_UUID_PRED  = "12345678-1234-5678-1234-56789abcdef2"  # predicciones
CHAR_UUID_CMD   = "12345678-1234-5678-1234-56789abcdef3"  # comandos


def decode_imu_packet(data: bytes) -> dict:
    """
    Formato binario del paquete IMU (28 bytes):
    [ts_ms: uint32] [ax,ay,az,gx,gy,gz: float32 × 6]
    """
    import struct
    ts, ax, ay, az, gx, gy, gz = struct.unpack("<Iffffff", data)
    return {"ts": ts, "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz}


def decode_prediction_packet(data: bytes) -> dict:
    """
    Formato binario del paquete de predicción (16 bytes):
    [sensor_id: uint8] [window_idx: uint32] [class: uint8]
    [prob_stance, prob_swing, prob_default: float32 × 3] [pad: 2 bytes]
    """
    import struct
    sid, widx, cls, ps, psw, pd = struct.unpack("<BIBfff", data[:18])
    return {
        "sensor_id": sid, "window_idx": widx, "predicted_class": cls,
        "prob_stance": ps, "prob_swing": psw, "prob_default": pd
    }


async def scan_and_connect(device_name: str) -> BleakClient:
    print(f"[BLE] Escaneando dispositivo '{device_name}'...")
    devices = await BleakScanner.discover(timeout=5.0)
    target = next((d for d in devices if d.name == device_name), None)
    if target is None:
        raise RuntimeError(f"Dispositivo '{device_name}' no encontrado")
    client = BleakClient(target.address)
    await client.connect()
    print(f"[BLE] Conectado a {target.name} ({target.address})")
    return client


async def log_sensor(device_name: str, output_path: Path,
                     trial_id: str, subject_id: str):
    client = await scan_and_connect(device_name)
    samples = []

    def notification_handler(sender, data):
        ts_pc = int(time.monotonic_ns() // 1_000_000)
        packet = decode_imu_packet(data)
        packet["timestamp_pc_ms"] = ts_pc
        packet["trial_id"] = trial_id
        packet["subject_id"] = subject_id
        samples.append(packet)

    await client.start_notify(CHAR_UUID_DATA, notification_handler)
    print(f"[BLE] Capturando... CTRL+C para detener")

    try:
        while True:
            await asyncio.sleep(1)
            print(f"  {len(samples)} muestras")
    except KeyboardInterrupt:
        pass

    await client.stop_notify(CHAR_UUID_DATA)
    await client.disconnect()

    # Guardar CSV
    if samples:
        import pandas as pd
        df = pd.DataFrame(samples)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        df.to_csv(output_path, index=False)
        print(f"[BLE] Guardado: {output_path} ({len(df)} filas)")
```

---

## 7. Adquisición multisensor con tres Arduino (USB)

### Estrategia: un proceso Python, tres puertos Serial

```python
# acquisition/serial_logger.py  (modo multi-sensor)
# python serial_logger.py --multi --config config/acquisition_config.yaml

import serial, csv, time, threading
from pathlib import Path

# acquisition_config.yaml especifica:
# sensors:
#   - port: /dev/ttyACM0  sensor_id: 1  placement: pelvis
#   - port: /dev/ttyACM1  sensor_id: 2  placement: thigh
#   - port: /dev/ttyACM2  sensor_id: 3  placement: ankle

def open_sensor(cfg: dict) -> serial.Serial:
    s = serial.Serial(cfg["port"], 115200, timeout=2)
    time.sleep(2)
    return s

def multi_logger(sensor_cfgs, output_dir, trial_id, subject_id):
    writers = {}
    files   = {}
    serials = {}
    stats   = {c["sensor_id"]: {"n":0,"corrupt":0} for c in sensor_cfgs}

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    for cfg in sensor_cfgs:
        sid = cfg["sensor_id"]
        fname = output_dir / f"s{subject_id}_t{trial_id}_sensor{sid}.csv"
        f = open(fname, "w", newline="")
        writers[sid] = csv.DictWriter(f, fieldnames=COLUMNS)
        writers[sid].writeheader()
        files[sid] = f
        serials[sid] = open_sensor(cfg)

    # Enviar START a todos simultáneamente
    t0 = time.monotonic_ns()
    for s in serials.values():
        s.write(b"START\n")

    running = {"v": True}

    def read_sensor(sid, ser, cfg):
        while running["v"]:
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
            except:
                break
            if not line or not line[0].isdigit():
                continue
            parts = line.split(",")
            if len(parts) != 9:
                stats[sid]["corrupt"] += 1
                continue
            ts_pc = int(time.monotonic_ns() // 1_000_000)
            writers[sid].writerow({
                "timestamp_pc_ms": ts_pc,
                "timestamp_arduino_ms": parts[0],
                "sensor_id": sid,
                "placement": cfg["placement"],
                "ax": parts[3], "ay": parts[4], "az": parts[5],
                "gx": parts[6], "gy": parts[7], "gz": parts[8],
                "label": "", "trial_id": trial_id, "subject_id": subject_id
            })
            stats[sid]["n"] += 1

    threads = [threading.Thread(target=read_sensor,
                                args=(c["sensor_id"], serials[c["sensor_id"]], c))
               for c in sensor_cfgs]
    for t in threads: t.start()

    input("ENTER para detener...")
    running["v"] = False
    for s in serials.values(): s.write(b"STOP\n")
    for t in threads: t.join(timeout=2)
    for f in files.values(): f.close()
    for s in serials.values(): s.close()

    for sid, st in stats.items():
        print(f"  Sensor {sid}: {st['n']} muestras, {st['corrupt']} corrupt")
```

---

## 8. Mecanismo de label durante adquisición

### Opción 1: label vacío, etiquetar después (recomendado)

El CSV se guarda con la columna `label` vacía. El etiquetado se hace
posteriormente usando los archivos de `data/labels/`.

### Opción 2: label por teclado en tiempo real

```python
# Durante la captura, el operador puede escribir:
#   s  → label = "stance"
#   w  → label = "swing"
#   d  → label = "default"
#   [ENTER] → label vacío (transición)
# El script mantiene el "label activo" y lo asigna a cada muestra.
```

### Opción 3: botón físico en el Arduino

```cpp
// El Arduino detecta un botón y envía "EVENT:BUTTON_PRESS:<millis>\n"
// El script Python registra el timestamp y permite asignar labels
// a intervalos definidos por los presses del botón.
```

---

## 9. Verificaciones de calidad de señal (QC)

Después de cada sesión, ejecutar:

```python
# acquisition/qc_check.py
import pandas as pd
import numpy as np

def check_csv(path):
    df = pd.read_csv(path)

    # 1. ODR real
    dt = df["timestamp_pc_ms"].diff().dropna()
    odr = 1000 / dt.mean()
    print(f"ODR estimado: {odr:.1f} Hz")

    # 2. Gaps (muestras perdidas)
    gaps = dt[dt > 20]
    print(f"Gaps > 20ms: {len(gaps)} → {gaps.values[:5]}")

    # 3. Valores fuera de rango
    for col in ["ax","ay","az"]:
        out = df[df[col].abs() > 16]
        if len(out): print(f"  {col}: {len(out)} valores > 16g")
    for col in ["gx","gy","gz"]:
        out = df[df[col].abs() > 2000]
        if len(out): print(f"  {col}: {len(out)} valores > 2000 °/s")

    # 4. NaNs
    print(f"NaNs: {df.isna().sum().to_dict()}")

    # 5. Varianza (señal plana indica sensor defectuoso)
    for col in ["ax","ay","az","gx","gy","gz"]:
        var = df[col].var()
        if var < 1e-6:
            print(f"  ALERTA: {col} varianza muy baja ({var:.2e}) — sensor estático o muerto")

    # 6. acc_norm debe estar ~1g en reposo
    df["acc_norm"] = np.sqrt(df.ax**2 + df.ay**2 + df.az**2)
    print(f"acc_norm media (debe ~1g en reposo): {df.acc_norm.mean():.3f} g")

    return df
```

---

## 10. Estructura de archivos de datos

```
data/
├── raw/
│   ├── s001_t001_sensor1_pelvis.csv
│   ├── s001_t001_sensor2_thigh.csv
│   ├── s001_t001_sensor3_ankle.csv
│   ├── s001_t002_sensor1_pelvis.csv
│   └── ...
├── labels/
│   ├── s001_t001_labels.csv          ← intervalos stance/swing/default
│   ├── s001_t002_labels.csv
│   └── ...
└── processed/
    ├── windows_train.npz             ← X, y, metadata
    ├── windows_val.npz
    ├── windows_test.npz
    └── stats.json                    ← media y std por canal (para Arduino)
```

### Formato del archivo de labels

```csv
start_ms, end_ms, label, notes
0,       580,   default,  inicio de sesión, sujeto estático
580,     1240,  stance,   apoyo derecho
1240,    1680,  swing,    oscilación derecha
1680,    2310,  stance,   apoyo izquierdo
2310,    2730,  swing,    oscilación izquierda
...
```

`start_ms` y `end_ms` son en `timestamp_pc_ms` relativo al inicio del trial.
