# firmware/host_node

## Propósito

Firmware para el **Arduino coordinador** en Fase D. Actúa como Central BLE:
se conecta a los 3 sensor_nodes, recibe sus predicciones y aplica la lógica
de fusión para producir la clasificación global de la fase de marcha.

También puede ser reemplazado por un script Python (`acquisition/prediction_receiver.py`)
durante las primeras pruebas de Milestone 10/11/12.

## Archivos

| Archivo | Descripción |
|---------|-------------|
| `host_node.ino` | Sketch principal |
| `voting_fusion.h` | Algoritmos de votación y fusión |
| `../common/ble_protocol.h` | UUIDs y `unpack_prediction()` |

## Pseudocódigo: votación simple y fusión

```cpp
// ============================================================
// voting_fusion.h
// ============================================================

#pragma once
#include <Arduino.h>

#define NUM_SENSORS  3
#define NUM_CLASSES  3
#define CLASS_STANCE  0
#define CLASS_SWING   1
#define CLASS_DEFAULT 2

#define MIN_CONFIDENCE     0.50f   // mínimo para que un sensor tenga peso
#define GLOBAL_THRESHOLD   0.60f   // mínimo score global para no ir a default
#define MAX_STALE_WINDOWS  3       // ventanas sin actualizar → sensor ausente

struct SensorPrediction {
  uint8_t  sensor_id;
  uint32_t window_idx;
  uint8_t  predicted_class;
  float    probs[NUM_CLASSES];    // [stance, swing, default]
  uint32_t last_update_ms;
  bool     active;
};

SensorPrediction sensor_preds[NUM_SENSORS];

// -------------------------------------------------------
// Inicializar predicciones
// -------------------------------------------------------
void voting_init() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensor_preds[i].active         = false;
    sensor_preds[i].last_update_ms = 0;
    sensor_preds[i].predicted_class = CLASS_DEFAULT;
    for (int c = 0; c < NUM_CLASSES; c++)
      sensor_preds[i].probs[c] = 1.0f / NUM_CLASSES;
  }
}

// -------------------------------------------------------
// Actualizar predicción de un sensor
// -------------------------------------------------------
void update_prediction(uint8_t sensor_id, uint32_t window_idx,
                       uint8_t cls, float ps, float psw, float pd) {
  int idx = sensor_id - 1;  // sensor_id 1-based → index 0-based
  if (idx < 0 || idx >= NUM_SENSORS) return;

  sensor_preds[idx].sensor_id       = sensor_id;
  sensor_preds[idx].window_idx      = window_idx;
  sensor_preds[idx].predicted_class = cls;
  sensor_preds[idx].probs[0]        = ps;
  sensor_preds[idx].probs[1]        = psw;
  sensor_preds[idx].probs[2]        = pd;
  sensor_preds[idx].last_update_ms  = millis();
  sensor_preds[idx].active          = true;
}

// -------------------------------------------------------
// Marcar sensores estancados como inactivos
// -------------------------------------------------------
void check_stale_sensors(uint32_t stale_timeout_ms = 1500) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensor_preds[i].active &&
        (millis() - sensor_preds[i].last_update_ms) > stale_timeout_ms) {
      sensor_preds[i].active = false;
      Serial.print("WARN:SENSOR_STALE:"); Serial.println(i + 1);
    }
  }
}

// -------------------------------------------------------
// Votación ponderada (weighted voting)
// -------------------------------------------------------
uint8_t weighted_vote(float* global_score_out) {
  float class_scores[NUM_CLASSES] = {0.0f, 0.0f, 0.0f};
  int   active_count = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensor_preds[i].active) continue;
    active_count++;

    float confidence = sensor_preds[i].probs[sensor_preds[i].predicted_class];

    // Solo contribuir si la confianza es mínima
    if (confidence >= MIN_CONFIDENCE) {
      for (int c = 0; c < NUM_CLASSES; c++) {
        class_scores[c] += sensor_preds[i].probs[c] * confidence;
      }
    } else {
      // Confianza baja: contribuir con peso reducido (0.3)
      for (int c = 0; c < NUM_CLASSES; c++) {
        class_scores[c] += sensor_preds[i].probs[c] * 0.3f;
      }
    }
  }

  if (active_count == 0) {
    *global_score_out = 0.0f;
    return CLASS_DEFAULT;
  }

  // Normalizar por número de sensores activos
  for (int c = 0; c < NUM_CLASSES; c++)
    class_scores[c] /= active_count;

  // Encontrar clase ganadora
  uint8_t best_class = CLASS_DEFAULT;
  float   best_score = -1.0f;
  for (int c = 0; c < NUM_CLASSES; c++) {
    if (class_scores[c] > best_score) {
      best_score = class_scores[c];
      best_class = c;
    }
  }

  *global_score_out = best_score;

  // Fallback a default si el score global es bajo
  if (best_score < GLOBAL_THRESHOLD) {
    return CLASS_DEFAULT;
  }

  return best_class;
}

// -------------------------------------------------------
// Majority voting simple
// -------------------------------------------------------
uint8_t majority_vote() {
  int votes[NUM_CLASSES] = {0, 0, 0};
  int active_count = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensor_preds[i].active) continue;
    votes[sensor_preds[i].predicted_class]++;
    active_count++;
  }

  if (active_count == 0) return CLASS_DEFAULT;

  // Requiere mayoría simple (≥ ceil(active/2))
  int threshold = (active_count / 2) + 1;
  for (int c = 0; c < NUM_CLASSES; c++) {
    if (votes[c] >= threshold) return (uint8_t)c;
  }

  return CLASS_DEFAULT;  // no hay mayoría
}

// -------------------------------------------------------
// Clasificación global (usar weighted como primario)
// -------------------------------------------------------
uint8_t classify_global(int method = 1) {
  check_stale_sensors();
  float score;
  if (method == 1)
    return weighted_vote(&score);
  else
    return majority_vote();
}

const char* class_name(uint8_t cls) {
  switch (cls) {
    case CLASS_STANCE:  return "STANCE";
    case CLASS_SWING:   return "SWING";
    case CLASS_DEFAULT: return "DEFAULT";
    default:            return "UNKNOWN";
  }
}
```

```cpp
// ============================================================
// host_node.ino — pseudocódigo principal
// ============================================================

#include <ArduinoBLE.h>
#include "voting_fusion.h"
#include "ble_protocol.h"

// Periféricos BLE esperados (nombres de los sensor_nodes)
const char* SENSOR_NAMES[NUM_SENSORS] = {
  "GaitNode_1", "GaitNode_2", "GaitNode_3"
};

BLEDevice peripherals[NUM_SENSORS];
bool connected[NUM_SENSORS] = {false, false, false};

void setup() {
  Serial.begin(115200);
  voting_init();

  if (!BLE.begin()) {
    Serial.println("ERROR:BLE");
    while (true);
  }

  Serial.println("HOST_READY");
  // Iniciar escaneo y conexión a los 3 nodos
  connect_to_all_sensors();
}

void loop() {
  BLE.poll();

  // Leer notificaciones de cada sensor conectado
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!connected[i]) {
      try_reconnect(i);
      continue;
    }

    BLECharacteristic pred_char = peripherals[i]
      .service(SERVICE_UUID)
      .characteristic(CHAR_UUID_PRED);

    if (pred_char && pred_char.valueUpdated()) {
      uint8_t packet[18];
      pred_char.readValue(packet, sizeof(packet));

      // Desempaquetar predicción
      uint8_t  sid   = packet[0];
      uint32_t widx;  memcpy(&widx, packet+1, 4);
      uint8_t  cls   = packet[5];
      float ps, psw, pd;
      memcpy(&ps,  packet+6,  4);
      memcpy(&psw, packet+10, 4);
      memcpy(&pd,  packet+14, 4);

      update_prediction(sid, widx, cls, ps, psw, pd);
    }
  }

  // Producir clasificación global cada ~250 ms (4 Hz)
  static unsigned long last_classify_ms = 0;
  if (millis() - last_classify_ms >= 250) {
    last_classify_ms = millis();

    uint8_t global_class = classify_global();

    Serial.print("GLOBAL:");
    Serial.print(class_name(global_class));
    Serial.print(" | S1:");
    Serial.print(class_name(sensor_preds[0].predicted_class));
    Serial.print(" S2:");
    Serial.print(class_name(sensor_preds[1].predicted_class));
    Serial.print(" S3:");
    Serial.println(class_name(sensor_preds[2].predicted_class));

    // LED RGB para feedback visual
    show_class_led(global_class);
  }
}

void show_class_led(uint8_t cls) {
  switch (cls) {
    case CLASS_STANCE:  digitalWrite(LED_RED, LOW); /* verde */ break;
    case CLASS_SWING:   /* azul */ break;
    case CLASS_DEFAULT: /* amarillo */ break;
  }
}
```

## Alternativa: host en Python (para debugging)

```python
# acquisition/prediction_receiver.py — pseudocódigo

import asyncio
from bleak import BleakScanner, BleakClient
import struct
import numpy as np

CLASS_NAMES = {0: "stance", 1: "swing", 2: "default"}

def weighted_vote(predictions):
    """
    predictions = lista de dicts con keys: sensor_id, predicted_class, probs
    """
    scores = np.zeros(3)
    active = 0
    for p in predictions:
        if p is None: continue
        conf = max(p["probs"])
        weight = conf if conf >= 0.5 else 0.3
        scores += np.array(p["probs"]) * weight
        active += 1
    if active == 0:
        return 2  # default
    scores /= active
    if scores.max() < 0.6:
        return 2  # default
    return int(np.argmax(scores))

async def main():
    # Descubrir los 3 sensor_nodes
    print("Escaneando...")
    devices = await BleakScanner.discover(timeout=10)
    gait_nodes = [d for d in devices if d.name and "GaitNode" in d.name]
    print(f"Encontrados: {[d.name for d in gait_nodes]}")

    latest_preds = {}

    def make_handler(device_name):
        def handler(sender, data):
            sid, widx, cls = struct.unpack("<BIB", data[:6])
            ps, psw, pd    = struct.unpack("<fff", data[6:18])
            latest_preds[sid] = {
                "sensor_id": sid,
                "predicted_class": cls,
                "probs": [ps, psw, pd]
            }
            global_class = weighted_vote(list(latest_preds.values()))
            print(f"  S{sid}:{CLASS_NAMES[cls]} ({ps:.2f},{psw:.2f},{pd:.2f})"
                  f"  → GLOBAL: {CLASS_NAMES[global_class]}")
        return handler

    # Conectar a todos y subscribir notificaciones
    clients = []
    for node in gait_nodes:
        c = BleakClient(node.address)
        await c.connect()
        clients.append(c)
        await c.start_notify(CHAR_UUID_PRED, make_handler(node.name))

    print("Recibiendo predicciones... CTRL+C para detener")
    try:
        while True:
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        pass
    for c in clients:
        await c.disconnect()

asyncio.run(main())
```
