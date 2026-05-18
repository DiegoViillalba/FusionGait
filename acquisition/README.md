# acquisition/ — Scripts de captura de datos

## Archivos

| Script | Milestone | Descripción |
|--------|-----------|-------------|
| `serial_logger.py` | M2, M4 | Logger USB Serial (1 o 3 Arduinos) |
| `ble_logger.py` | M10+ | Logger BLE alternativo (predicciones) |
| `live_plot.py` | M3 | Visualización en tiempo real |
| `sync_tools.py` | M4 | Alineación temporal entre señales |
| `prediction_receiver.py` | M10 | Recibe predicciones BLE y aplica votación |
| `qc_check.py` | M2+ | Verificación de calidad de CSVs |

## Uso rápido

```bash
# Un sensor (Milestone 2)
python serial_logger.py \
  --port /dev/ttyACM0 \
  --sensor_id 3 --placement ankle \
  --subject_id s001 --trial_id t001 \
  --output ../data/raw/s001_t001_sensor3_ankle.csv

# Tres sensores (Milestone 4)
python serial_logger.py \
  --multi \
  --config ../config/acquisition_config.yaml \
  --subject_id s001 --trial_id t001 \
  --output_dir ../data/raw/

# Visualización en vivo (Milestone 3)
python live_plot.py --port /dev/ttyACM0

# QC de un CSV (Milestone 2+)
python qc_check.py ../data/raw/s001_t001_sensor3_ankle.csv
```

## Dependencias

```
pip install pyserial pandas numpy matplotlib bleak
```

Ver pseudocódigo completo en [docs/acquisition_protocol.md](../docs/acquisition_protocol.md).
