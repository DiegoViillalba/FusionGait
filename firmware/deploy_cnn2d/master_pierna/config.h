#pragma once

// ── Sensor identity ──────────────────────────────────────────────────────
// MASTER = pierna (sensor_id=2 en datos de entrenamiento)
#define SENSOR_ID        2
#define PLACEMENT        "pierna"

// ── IMU ──────────────────────────────────────────────────────────────────
#define IMU_REV2         1   // 0=LSM9DS1, 1=BMI270 (Rev2)

// ── Número de esclavos BLE ───────────────────────────────────────────────
// 1 → Master(pierna, s2) + Slave(tobillo, s3)  → N_FEATURES=12
// Orden en el tensor: [pierna_ax..gz | tobillo_ax..gz]
// debe coincidir con SENSOR_IDS=[2,3] del notebook.
#define N_SLAVES         1

// Nombre BLE del esclavo (debe coincidir con inference_slave/config.h)
// Usar inference_slave/config_s3.h para el Arduino del tobillo
#define SLAVE2_NAME      "GaitSlave_3"   // tobillo

// UUIDs personalizados — iguales en master y slave
#define BLE_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"

// ── Modelo TinyML ─────────────────────────────────────────────────────────
// Frecuencia de muestreo real del BMI270 (medida en datos de entrenamiento: 16 ms/muestra)
#define SAMPLE_MS        16
#define SAMPLE_HZ        62             // 1000/16 = 62.5 → truncado a 62

#define WINDOW_SIZE      120            // 120 × 16 ms = 1.92 s de ventana
#define STEP             10             // inferencia cada 160 ms
#define N_FEATURES       (6 * (1 + N_SLAVES))   // 12 con 1 esclavo
#define N_CLASSES        4

// Tensor arena empírica (GaitCNN2D_Residual int8, N_FEATURES=12):
//   ~40 KB usado → 64 KB de arena es suficiente.
// Aumentar a 96 si el sketch imprime ERROR:ARENA_TOO_SMALL.
#define TENSOR_ARENA_KB  64

// ── Serial ───────────────────────────────────────────────────────────────
#define SERIAL_BAUD      115200
