# Log de Experimentos

## Instrucciones de uso

Registrar cada sesión experimental aquí. Incluir suficiente detalle para
poder reproducir o descartar la sesión. Una sesión mal documentada no puede
usarse en el dataset final.

---

## Plantilla de sesión

```
## Sesión EXP-NNNN — YYYY-MM-DD

### Hardware
- Sensor 1: Arduino Nano 33 BLE Sense (serial: XXXX), placement: pelvis
- Sensor 2: Arduino Nano 33 BLE Sense (serial: XXXX), placement: thigh
- Sensor 3: Arduino Nano 33 BLE Sense (serial: XXXX), placement: ankle
- Firmware: git commit XXXXXXX
- Montaje: faja elástica / cinta adhesiva / velcro

### Sujeto
- subject_id: s001
- Talla / masa aproximada (si relevante): 1.72 m / 70 kg
- Calzado: zapatillas blandas
- Condiciones: sin patología aparente, marcha normal

### Protocolo
- Número de trials: 4
- Duración aproximada por trial: 60 s
- Distancia del pasillo: 10 m
- Condición: marcha a velocidad normal, giro al final
- Inicio de cada trial: heel tap triple (x3) antes de iniciar marcha

### Archivos generados
- data/raw/s001_t001_sensor1_pelvis.csv   (N filas)
- data/raw/s001_t001_sensor2_thigh.csv    (N filas)
- data/raw/s001_t001_sensor3_ankle.csv    (N filas)
- data/labels/s001_t001_labels.csv        (método: auto + revisión)

### QC
- ODR sensor 1: 99.8 Hz
- ODR sensor 2: 100.1 Hz
- ODR sensor 3: 99.7 Hz
- Gaps > 20 ms: sensor1=0, sensor2=2, sensor3=0
- Desincronización temporal estimada por xcorr: < 15 ms

### Observaciones
- Trial 2: artefacto entre ms 45000–47000 (cable jalado)
  → marcado como "default" en labels
- Sensor 2 mostró valores de gyro algo elevados en eje Z durante giros
  → normal para ese placement

### Decisión de uso
- [ ] Incluir en dataset (todos los trials)
- [ ] Incluir parcialmente (excluir trial XXXX)
- [ ] Excluir (razón: ___)
```

---

## Sesiones registradas

*(Vacío — registrar aquí cada sesión experimental)*
