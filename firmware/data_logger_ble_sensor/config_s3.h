#pragma once
// ── Sensor 3 — Cadera ─────────────────────────────────────────────────────
// Copiar este archivo como config.h al compilar para el Sensor 3.

#define SENSOR_ID        3
#define PLACEMENT        "cadera"   // ajustar al segmento real

#define IMU_REV2         1   // 0=LSM9DS1, 1=BMI270 (Rev2)

// Nombre BLE anunciado — debe coincidir con SLAVE3_NAME en data_logger_hub/config.h
#define BLE_DEVICE_NAME  "GaitNode_3"

// UUIDs — deben ser iguales al hub (data_logger_hub/config.h)
#define BLE_SERVICE_UUID "19B20000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B20001-E8F2-537E-4F6C-D104768A1214"

#define SAMPLE_HZ        100
#define SAMPLE_MS        (1000 / SAMPLE_HZ)
