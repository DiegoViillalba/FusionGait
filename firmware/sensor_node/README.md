# firmware/sensor_node

## Propósito

Firmware para Arduino Nano 33 BLE Sense en **Fase C/D** (inferencia TinyML + comunicación).
Cada nodo lee la IMU, mantiene un buffer circular, ejecuta el modelo cuantizado INT8
y emite predicciones por BLE hacia el host.

## Archivos

| Archivo | Descripción |
|---------|-------------|
| `sensor_node.ino` | Sketch principal |
| `config.h` | SENSOR_ID, PLACEMENT, parámetros de ventana |
| `../common/model_data.h` | Modelo TFLite como array C (generado por export_tflite.py) |
| `../common/normalization.h` | Media y std por canal (de stats.json) |
| `../common/ble_protocol.h` | UUIDs GATT y funciones de empaquetado |

## Pseudocódigo completo

```cpp
// ============================================================
// sensor_node.ino
// ============================================================

#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>
#include <TensorFlowLite.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "config.h"
#include "model_data.h"       // g_model_data[], g_model_data_len
#include "normalization.h"    // CHANNEL_MEAN[], CHANNEL_STD[]
#include "ble_protocol.h"     // SERVICE_UUID, CHAR_UUID_PRED, pack_prediction()

// --- TFLite Micro ---
const int TENSOR_ARENA_SIZE = 10 * 1024;  // ajustar según modelo
uint8_t tensor_arena[TENSOR_ARENA_SIZE];

tflite::AllOpsResolver resolver;
const tflite::Model* tfl_model        = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input_tensor  = nullptr;
TfLiteTensor* output_tensor = nullptr;

// --- Buffer circular de ventana ---
float window_buf[WINDOW_SIZE][NUM_CHANNELS];
int   buf_write_idx = 0;
int   samples_since_last_inference = 0;

// --- BLE ---
BLEService        gait_service(SERVICE_UUID);
BLECharacteristic pred_char(CHAR_UUID_PRED, BLENotify, 18);

// --- Setup ---
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  // 1. Inicializar IMU
  if (!IMU.begin()) {
    Serial.println("ERROR:IMU");
    while (true);
  }

  // 2. Inicializar TFLite
  tfl_model = tflite::GetModel(g_model_data);
  if (tfl_model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("ERROR:TFLITE_VERSION");
    while (true);
  }
  interpreter = new tflite::MicroInterpreter(
    tfl_model, resolver, tensor_arena, TENSOR_ARENA_SIZE);
  interpreter->AllocateTensors();
  input_tensor  = interpreter->input(0);
  output_tensor = interpreter->output(0);

  // Log de memoria usada
  Serial.print("Arena usada: ");
  Serial.print(interpreter->arena_used_bytes());
  Serial.println(" bytes");

  // 3. Inicializar BLE
  if (!BLE.begin()) {
    Serial.println("ERROR:BLE");
    while (true);
  }
  BLE.setLocalName(BLE_DEVICE_NAME);  // e.g. "GaitNode_3"
  BLE.setAdvertisedService(gait_service);
  gait_service.addCharacteristic(pred_char);
  BLE.addService(gait_service);
  BLE.advertise();

  Serial.println("READY");
}

// --- Loop ---
void loop() {
  BLE.poll();  // mantener BLE activo

  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  // Escribir muestra en buffer circular
  window_buf[buf_write_idx][0] = ax;
  window_buf[buf_write_idx][1] = ay;
  window_buf[buf_write_idx][2] = az;
  window_buf[buf_write_idx][3] = gx;
  window_buf[buf_write_idx][4] = gy;
  window_buf[buf_write_idx][5] = gz;

  buf_write_idx = (buf_write_idx + 1) % WINDOW_SIZE;
  samples_since_last_inference++;

  // Ejecutar inferencia cada WINDOW_STEP muestras (overlap 50%)
  if (samples_since_last_inference >= WINDOW_STEP) {
    samples_since_last_inference = 0;
    run_inference();
  }
}

// --- Inferencia ---
void run_inference() {
  // 1. Normalizar ventana y copiar al input tensor (INT8)
  // El input tensor tiene shape [1, WINDOW_SIZE, NUM_CHANNELS]
  int8_t* inp_data = input_tensor->data.int8;
  float   in_scale = input_tensor->params.scale;
  int     in_zp    = input_tensor->params.zero_point;

  for (int t = 0; t < WINDOW_SIZE; t++) {
    int read_idx = (buf_write_idx + t) % WINDOW_SIZE;  // orden temporal
    for (int c = 0; c < NUM_CHANNELS; c++) {
      float val_normalized = (window_buf[read_idx][c] - CHANNEL_MEAN[c])
                             / CHANNEL_STD[c];
      // Cuantizar a INT8
      int quantized = (int)(val_normalized / in_scale) + in_zp;
      quantized = max(-128, min(127, quantized));  // clamp
      inp_data[t * NUM_CHANNELS + c] = (int8_t)quantized;
    }
  }

  // 2. Invocar modelo
  unsigned long t0 = micros();
  TfLiteStatus status = interpreter->Invoke();
  unsigned long dt = micros() - t0;

  if (status != kTfLiteOk) {
    Serial.println("ERROR:INFERENCE");
    return;
  }

  // 3. Leer salida y descuantizar
  int8_t* out_data = output_tensor->data.int8;
  float   out_scale = output_tensor->params.scale;
  int     out_zp    = output_tensor->params.zero_point;

  float probs[NUM_CLASSES];
  int   best_class = 0;
  float best_prob  = -1.0f;

  for (int i = 0; i < NUM_CLASSES; i++) {
    probs[i] = (out_data[i] - out_zp) * out_scale;
    if (probs[i] > best_prob) {
      best_prob  = probs[i];
      best_class = i;
    }
  }

  // 4. Emitir predicción por BLE y Serial
  static uint32_t window_counter = 0;
  window_counter++;

  uint8_t packet[18];
  pack_prediction(packet, SENSOR_ID, window_counter,
                  best_class, probs[0], probs[1], probs[2]);

  if (BLE.connected()) {
    pred_char.writeValue(packet, sizeof(packet));
  }

  // Debug por Serial
  Serial.print("PRED:");
  Serial.print(SENSOR_ID);    Serial.print(',');
  Serial.print(window_counter); Serial.print(',');
  Serial.print(best_class);   Serial.print(',');
  Serial.print(probs[0], 3);  Serial.print(',');
  Serial.print(probs[1], 3);  Serial.print(',');
  Serial.print(probs[2], 3);  Serial.print(',');
  Serial.print(dt / 1000);    Serial.println("ms");
}
```

```cpp
// ============================================================
// common/normalization.h — generado por export_tflite.py
// ============================================================
// Valores de stats.json convertidos a constantes C

#pragma once

const float CHANNEL_MEAN[6] = {
  0.0123f,  // ax_mean
  -0.0521f, // ay_mean
  9.8134f,  // az_mean
  0.0312f,  // gx_mean
 -0.0145f,  // gy_mean
  0.0223f   // gz_mean
};

const float CHANNEL_STD[6] = {
  1.2341f,  // ax_std
  1.1892f,  // ay_std
  0.8923f,  // az_std
  45.234f,  // gx_std
  38.192f,  // gy_std
  12.891f   // gz_std
};
// NOTA: estos valores son ejemplos; reemplazar con los reales de stats.json
```

```cpp
// ============================================================
// common/ble_protocol.h
// ============================================================

#pragma once

// UUIDs de servicio y características GATT
#define SERVICE_UUID     "12345678-1234-5678-1234-56789abcdef0"
#define CHAR_UUID_PRED   "12345678-1234-5678-1234-56789abcdef2"
#define CHAR_UUID_CMD    "12345678-1234-5678-1234-56789abcdef3"

// Nombre de dispositivo BLE (personalizar por nodo)
// #define BLE_DEVICE_NAME  "GaitNode_1"   // en config.h de cada nodo

/*
 * Formato del paquete de predicción (18 bytes):
 *  [0]     uint8_t  sensor_id
 *  [1..4]  uint32_t window_idx (little-endian)
 *  [5]     uint8_t  predicted_class (0=stance, 1=swing, 2=default)
 *  [6..9]  float32  prob_stance
 *  [10..13] float32 prob_swing
 *  [14..17] float32 prob_default
 */
inline void pack_prediction(uint8_t* buf, uint8_t sid, uint32_t widx,
                             uint8_t cls, float ps, float psw, float pd) {
  buf[0] = sid;
  memcpy(buf + 1, &widx, 4);
  buf[5] = cls;
  memcpy(buf + 6,  &ps,  4);
  memcpy(buf + 10, &psw, 4);
  memcpy(buf + 14, &pd,  4);
}
```

## Restricciones de memoria

| Recurso | Presupuesto máximo |
|---------|-------------------|
| tensor_arena | 50 KB |
| model_data.h (Flash) | 50 KB |
| window_buf (SRAM) | 50×6×4 = 1.2 KB |
| Stack + BLE stack | ~100 KB SRAM |
| **Total SRAM libre** | **> 50 KB** |
