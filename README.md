# FusionGait

Sistema distribuido para clasificación de fases de la marcha humana usando tres
Arduino Nano 33 BLE Sense con IMU integrada y pipeline TinyML.

## Visión general

El sistema identifica fases de la marcha en tiempo real combinando señales de tres
sensores inerciales colocados en pelvis, muslo y tobillo. La arquitectura es
deliberadamente escalonada: primero adquisición robusta, luego comparación de
modelos offline, luego inferencia local TinyML, finalmente fusión multisensor.

## Clases objetivo

| Clase     | Descripción                                                   |
|-----------|---------------------------------------------------------------|
| `stance`  | Pie en contacto con el suelo                                  |
| `swing`   | Pie en el aire, avanzando                                     |
| `default` | Transición, ambigüedad, pausa o estado no reconocible         |

Extensión futura: heel strike, foot flat, mid stance, toe off, initial/mid/terminal swing.

## Fases del proyecto

| Fase | Descripción                            | Milestones |
|------|----------------------------------------|------------|
| A    | Adquisición de datos (data logger)     | M1 – M4    |
| B    | Entrenamiento y comparación offline    | M5 – M7    |
| C    | Despliegue TinyML en Arduino           | M8 – M9    |
| D    | Fusión multisensor y votación          | M10 – M13  |

## Hardware

- 3× Arduino Nano 33 BLE Sense (nRF52840 + LSM9DS1)
- IMU: acelerómetro + giroscopio de 6 ejes
- Comunicación Fase A: USB Serial (115200 baud)
- Comunicación Fase C/D: BLE (nRF52840 Bluetooth 5.0)
- Alimentación Fase A: USB; Fase C/D: LiPo 3.7V + regulador 3.3V

## Posicionamiento de sensores

```
sensor_id=1  placement=pelvis   → zona lumbar / sacro
sensor_id=2  placement=thigh    → cara lateral del muslo, encima de la rodilla
sensor_id=3  placement=ankle    → cara lateral del tobillo / espinilla distal
```

## Documentación técnica

- [Arquitectura del sistema](docs/architecture.md)
- [Protocolo de adquisición](docs/acquisition_protocol.md)
- [Guía de etiquetado](docs/labeling_guide.md)
- [Plan de milestones](docs/milestones.md)
- [Log de experimentos](docs/experiments.md)

## Estructura del repositorio

```
FusionGait/
├── README.md
├── docs/
│   ├── architecture.md          # Diseño técnico completo del sistema
│   ├── acquisition_protocol.md  # Protocolo de captura de datos
│   ├── labeling_guide.md        # Estrategias de etiquetado
│   ├── milestones.md            # Plan de implementación
│   └── experiments.md           # Log de sesiones experimentales
├── config/
│   ├── sensor_config.yaml       # IDs, placements, orientación
│   ├── acquisition_config.yaml  # Frecuencia, formato, puertos
│   └── model_config.yaml        # Arquitectura y entrenamiento
├── firmware/
│   ├── data_logger_node/        # Arduino en modo data logger (Fase A)
│   ├── sensor_node/             # Arduino con TinyML local (Fase C)
│   ├── host_node/               # Arduino coordinador BLE (Fase D)
│   └── common/                  # Librerías compartidas entre firmwares
├── acquisition/
│   ├── serial_logger.py         # Logger USB para 1 o 3 Arduinos
│   ├── ble_logger.py            # Logger BLE alternativo
│   ├── live_plot.py             # Visualización en tiempo real
│   └── sync_tools.py            # Alineación post-hoc de señales
├── training/
│   ├── data/                    # Symlink o copia local de data/processed/
│   ├── notebooks/               # Exploración, visualizaciones, análisis
│   └── src/
│       ├── load_data.py         # Carga y validación de CSVs crudos
│       ├── preprocess.py        # Filtrado, interpolación, limpieza
│       ├── windowing.py         # Segmentación en ventanas + asignación de labels
│       ├── features.py          # Features manuales para modelos clásicos
│       ├── train_baselines.py   # Reglas, LogReg, RF, SVM, MLP
│       ├── train_cnn1d.py       # CNN 1D con Keras/TF
│       ├── evaluate.py          # Métricas, matriz de confusión, por sujeto
│       └── export_tflite.py     # Cuantización y exportación a TFLite Micro
│   ├── models/                  # Artefactos: .h5, .tflite, .h (C array)
│   └── reports/                 # Tablas comparativas, figuras de evaluación
└── data/
    ├── raw/                     # CSVs directamente de serial_logger.py
    ├── labels/                  # Archivos de etiquetas por trial
    └── processed/               # Ventanas .npz para entrenamiento
```

## Inicio rápido (Milestone 1)

```bash
# 1. Instalar dependencias Python
pip install pyserial pandas numpy matplotlib

# 2. Subir firmware data_logger_node al Arduino
# (Arduino IDE o arduino-cli)

# 3. Capturar datos de un sensor
python acquisition/serial_logger.py \
  --port /dev/ttyACM0 \
  --sensor_id 1 \
  --placement ankle \
  --subject_id s001 \
  --trial_id t001 \
  --output data/raw/s001_t001_ankle.csv
```
