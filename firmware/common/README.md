# firmware/common — Código compartido entre todos los nodos

## Archivos

| Archivo | Generado por | Descripción |
|---------|-------------|-------------|
| `model_data.h` | `export_tflite.py` + `xxd` | Modelo TFLite como array C `g_model_data[]` |
| `normalization.h` | `export_tflite.py` | Constantes `CHANNEL_MEAN[]` y `CHANNEL_STD[]` |
| `ble_protocol.h` | Manual | UUIDs GATT y función `pack_prediction()` |

## Notas

- `model_data.h` y `normalization.h` son archivos generados automáticamente.
  Nunca editarlos a mano.
- Para regenerarlos, correr `training/src/export_tflite.py` después de
  entrenar y seleccionar el mejor modelo.
- Tanto `sensor_node` como (opcionalmente) `host_node` incluyen estos headers.
