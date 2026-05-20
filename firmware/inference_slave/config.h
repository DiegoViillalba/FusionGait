#pragma once

// ── Sensor identity ──────────────────────────────────────────────────────
#define SENSOR_ID        2
#define PLACEMENT        "pierna"

// ── IMU model ────────────────────────────────────────────────────────────
// 0 = LSM9DS1  (Nano 33 BLE Sense original)
// 1 = BMI270   (Nano 33 BLE Sense Rev2)
#define IMU_REV2         1

// ── BLE identity ─────────────────────────────────────────────────────────
#define BLE_DEVICE_NAME  "GaitSlave_2"

// Custom 128-bit UUIDs — must match inference_master/config.h exactly
#define BLE_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"

// ── Sample rate ───────────────────────────────────────────────────────────
#define SAMPLE_HZ        100
#define SAMPLE_MS        (1000 / SAMPLE_HZ)   // 10 ms
