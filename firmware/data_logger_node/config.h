#pragma once

// ─── EDITAR ANTES DE SUBIR A CADA ARDUINO ───────────────────────────────────
// Cambiar estos dos valores según el nodo que estás programando:
//   Nodo 1: SENSOR_ID 1, PLACEMENT "pelvis"
//   Nodo 2: SENSOR_ID 2, PLACEMENT "thigh"
//   Nodo 3: SENSOR_ID 3, PLACEMENT "ankle"
#define SENSOR_ID  3
#define PLACEMENT  "ankle"
// ─────────────────────────────────────────────────────────────────────────────

#define BAUD_RATE        115200
#define SAMPLE_HZ        100
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_HZ)   // 10 000 µs

// LED RGB del Nano 33 BLE Sense (activo en LOW)
#define LED_IDLE     LEDB    // azul  → esperando START
#define LED_CAPTURE  LEDG    // verde → capturando
#define LED_ERROR    LEDR    // rojo  → error hardware
