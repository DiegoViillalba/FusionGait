#pragma once
// ── Sensor 2 — Pierna ─────────────────────────────────────────────────────
// Para el Sensor 3 (cadera) usar config_s3.h copiado como config.h.

#define SENSOR_ID        2
#define PLACEMENT        "pierna"   // ajustar al segmento real

#define IMU_REV2         1   // 0=LSM9DS1, 1=BMI270 (Rev2)

// Nombre BLE anunciado — debe coincidir con SLAVE2_NAME en data_logger_hub/config.h
#define BLE_DEVICE_NAME  "GaitNode_2"

// UUIDs — deben ser iguales al hub (data_logger_hub/config.h)
#define BLE_SERVICE_UUID "19B20000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B20001-E8F2-537E-4F6C-D104768A1214"

#define SAMPLE_HZ        100
#define SAMPLE_MS        (1000 / SAMPLE_HZ)
