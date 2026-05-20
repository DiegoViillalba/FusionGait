#pragma once

// ── Sensor identity ──────────────────────────────────────────────────────
#define SENSOR_ID        1
#define PLACEMENT        "tobillo"

// ── IMU ──────────────────────────────────────────────────────────────────
#define IMU_REV2         1   // 0=LSM9DS1, 1=BMI270 (Rev2)

// ── Número de esclavos BLE ───────────────────────────────────────────────
// 1 → Master(tobillo) + Slave2(pierna)           → N_FEATURES 12
// 2 → Master(tobillo) + Slave2(pierna) + Slave3  → N_FEATURES 18
#define N_SLAVES         2

// Nombres BLE de cada esclavo (deben coincidir con inference_slave/config.h)
#define SLAVE2_NAME      "GaitSlave_2"
#define SLAVE3_NAME      "GaitSlave_3"

// UUIDs personalizados — iguales en master y ambos slaves
#define BLE_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"

// ── Modelo TinyML ─────────────────────────────────────────────────────────
#define WINDOW_SIZE      50
#define STEP             25
#define N_FEATURES       (6 * (1 + N_SLAVES))   // 12 con 1 esclavo, 18 con 2
#define N_CLASSES        4

// Tensor arena.  Referencia empírica (Conv2D 16/32 + Dense 32):
//   N_FEATURES=12  → ~38 KB usado  →  64 KB arena
//   N_FEATURES=18  → ~52 KB usado  →  80 KB arena
// Aumentar si el sketch imprime ERROR:ARENA_TOO_SMALL.
#define TENSOR_ARENA_KB  80

// ── Serial ───────────────────────────────────────────────────────────────
#define SERIAL_BAUD      115200
