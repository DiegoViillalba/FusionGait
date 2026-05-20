#pragma once
// Copiar este archivo a config.h al compilar para el Esclavo 3.
// (o usar config.h para S2 y este para S3 si tienes dos sesiones de Arduino IDE abiertas)

#define SENSOR_ID        3
#define PLACEMENT        "cadera"      // ajustar al segmento real

#define IMU_REV2         1

#define BLE_DEVICE_NAME  "GaitSlave_3"

// UUIDs — deben ser iguales al master
#define BLE_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_IMU_UUID     "19B10001-E8F2-537E-4F6C-D104768A1214"

#define SAMPLE_HZ        100
#define SAMPLE_MS        (1000 / SAMPLE_HZ)
