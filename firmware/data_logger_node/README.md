# firmware/data_logger_node

## Propósito

Firmware para Arduino Nano 33 BLE Sense en **Fase A** (adquisición de datos).
El Arduino actúa como data logger puro: lee la IMU y envía datos crudos al PC
por Serial. No clasifica, no ejecuta ningún modelo.

## Archivos

| Archivo | Descripción |
|---------|-------------|
| `data_logger_node.ino` | Sketch principal |
| `config.h` | Constantes: SENSOR_ID, PLACEMENT, BAUD_RATE |

## Pseudocódigo completo

```cpp
// ============================================================
// data_logger_node.ino
// ============================================================

#include <Arduino_LSM9DS1.h>
#include "config.h"

// --- Estado ---
enum State { IDLE, CAPTURING };
State state = IDLE;
String trial_id   = "t000";
String subject_id = "s000";
unsigned long last_sample_us = 0;

// --- Setup ---
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000);  // timeout por si no hay PC

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  if (!IMU.begin()) {
    Serial.println("ERROR:IMU_NOT_FOUND");
    set_led(255, 0, 0);  // rojo = error
    while (true) delay(1000);
  }

  Serial.println("READY");
  set_led(0, 0, 255);  // azul = esperando START
}

// --- Loop ---
void loop() {
  // 1. Procesar comandos entrantes por Serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    process_command(cmd);
  }

  // 2. Captura periódica a ~SAMPLE_HZ Hz
  if (state == CAPTURING) {
    unsigned long now_us = micros();
    if (now_us - last_sample_us >= SAMPLE_PERIOD_US) {
      last_sample_us = now_us;
      read_and_send();
    }
  }
}

// --- Comandos ---
void process_command(const String& cmd) {
  if (cmd == "START") {
    state = CAPTURING;
    last_sample_us = micros();
    set_led(0, 255, 0);  // verde = capturando
    Serial.println("ACK:START");

  } else if (cmd == "STOP") {
    state = IDLE;
    set_led(0, 0, 255);  // azul = idle
    Serial.println("ACK:STOP");

  } else if (cmd.startsWith("SET_TRIAL:")) {
    trial_id = cmd.substring(10);
    Serial.print("ACK:SET_TRIAL:"); Serial.println(trial_id);

  } else if (cmd.startsWith("SET_SUBJECT:")) {
    subject_id = cmd.substring(12);
    Serial.print("ACK:SET_SUBJECT:"); Serial.println(subject_id);

  } else if (cmd == "SYNC") {
    Serial.print("SYNC_ACK:"); Serial.println(millis());

  } else if (cmd == "STATUS") {
    Serial.print("STATUS:id=");      Serial.print(SENSOR_ID);
    Serial.print(",placement=");     Serial.print(PLACEMENT);
    Serial.print(",trial=");         Serial.print(trial_id);
    Serial.print(",subject=");       Serial.print(subject_id);
    Serial.print(",capturing=");     Serial.println(state == CAPTURING ? "1" : "0");
  }
}

// --- Lectura y envío de muestra ---
void read_and_send() {
  float ax, ay, az, gx, gy, gz;

  // Verificar disponibilidad (evita bloqueo si IMU no está lista)
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  // Formato: ts_ms,sensor_id,placement,ax,ay,az,gx,gy,gz
  Serial.print(millis());
  Serial.print(','); Serial.print(SENSOR_ID);
  Serial.print(','); Serial.print(PLACEMENT);
  Serial.print(','); Serial.print(ax, 4);
  Serial.print(','); Serial.print(ay, 4);
  Serial.print(','); Serial.print(az, 4);
  Serial.print(','); Serial.print(gx, 4);
  Serial.print(','); Serial.print(gy, 4);
  Serial.print(','); Serial.println(gz, 4);
}

// --- LED RGB ---
void set_led(int r, int g, int b) {
  // Nano 33 BLE Sense: LED_RED, LED_GREEN, LED_BLUE son activos-bajos
  digitalWrite(LED_RED,   r > 0 ? LOW : HIGH);
  digitalWrite(LED_GREEN, g > 0 ? LOW : HIGH);
  digitalWrite(LED_BLUE,  b > 0 ? LOW : HIGH);
}
```

```cpp
// ============================================================
// config.h — personalizar por cada nodo antes de compilar
// ============================================================

#pragma once

#define SENSOR_ID      3          // 1=pelvis, 2=thigh, 3=ankle
#define PLACEMENT      "ankle"    // string descriptivo
#define BAUD_RATE      115200
#define SAMPLE_HZ      100
#define SAMPLE_PERIOD_US  (1000000 / SAMPLE_HZ)  // 10000 µs
```

## Dependencias

- `Arduino_LSM9DS1` library (instalar desde Arduino Library Manager)

## Cómo usar

1. Editar `config.h` con el `SENSOR_ID` y `PLACEMENT` correcto para cada Arduino.
2. Compilar y subir el sketch.
3. Abrir `acquisition/serial_logger.py` en el PC.
4. El Arduino envía `READY\n` al conectarse.
5. El script Python envía `START\n` para iniciar captura.

## Verificación rápida

Abrir el Serial Monitor (115200 baud) y verificar:
- Una línea por cada ~10 ms (100 Hz).
- Con el Arduino en reposo sobre la mesa: az ≈ 1.0, ax ≈ ay ≈ 0.
- Inclinando el Arduino: los valores de aceleración cambian coherentemente.
