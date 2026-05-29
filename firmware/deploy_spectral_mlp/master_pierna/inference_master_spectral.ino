/**
 * inference_master_spectral.ino — FusionGait Master + MLP Espectral
 *
 * Hardware : Arduino Nano 33 BLE Sense Rev2 (nRF52840 + BMI270)
 * Role     : BLE Central — recibe 84 features espectrales del slave (tobillo),
 *            extrae sus propias 84 features (pierna) y corre el MLP int8.
 *
 * Modelo   : GaitSpectralMLP  input=(168,) output=(4,)  ~10 KB int8
 * Archivos requeridos (generados por el notebook + add_spectral_pipeline.py):
 *   gait_model_spectral.h   — modelo MLP int8
 *   spectral_scaler.h       — sp_scaler_mean[168], sp_scaler_scale[168]
 *                             + función normalize_spectral(float* x)
 *
 * Salida Serial 115200:
 *   INFER,<fase>,<p0>,<p1>,<p2>,<p3>
 *   STATUS:<msg>
 *   ARENA:<bytes>
 *   ERROR:<msg>
 */

#include <ArduinoBLE.h>
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/schema/schema_generated.h>

#include "config.h"
#include "gait_model_spectral.h"   // generado por add_spectral_pipeline.py
#include "spectral_scaler.h"       // sp_scaler_mean[], sp_scaler_scale[], normalize_spectral()

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

static const char* PHASE_NAMES[N_CLASSES] = {
    "loading", "midstance", "terminal", "swing"
};

// ── Ring buffer local (pierna) ────────────────────────────────────────────
static float ring[WINDOW_SIZE][N_AXES];
static int   buf_head  = 0;
static int   buf_count = 0;
static int   step_cnt  = 0;

// ── Features del slave (tobillo) recibidas por BLE ────────────────────────
static volatile float slave_features[SPECTRAL_DIM_PER_SENSOR];
static volatile bool  slave_ready = false;

void onSlaveNotify(BLEDevice, BLECharacteristic ch) {
    if (ch.valueLength() == SPECTRAL_DIM_PER_SENSOR * (int)sizeof(float)) {
        memcpy((void*)slave_features, ch.value(),
               SPECTRAL_DIM_PER_SENSOR * sizeof(float));
        slave_ready = true;
    }
}

// ── Tensor arena ─────────────────────────────────────────────────────────
static uint8_t tensor_arena[TENSOR_ARENA_KB * 1024];

// ── TFLite globals ───────────────────────────────────────────────────────
static const tflite::Model*      tfl_model   = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;
static TfLiteTensor*             input_tens  = nullptr;
static TfLiteTensor*             output_tens = nullptr;

// ── LED ──────────────────────────────────────────────────────────────────
inline void setLED(bool r, bool g, bool b) {
    digitalWrite(LEDR, r ? LOW : HIGH);
    digitalWrite(LEDG, g ? LOW : HIGH);
    digitalWrite(LEDB, b ? LOW : HIGH);
}
void haltError(const char* msg) {
    Serial.println(msg);
    while (true) { setLED(true,false,false); delay(300); setLED(false,false,false); delay(300); }
}

// ── Extractor espectral (mismo que inference_slave_spectral.ino) ──────────
static float bandEnergy(const float* sig, int n, float f_lo, float f_hi, float fs) {
    float energy = 0.0f;
    float df = fs / (float)n;
    int lo = (int)(f_lo / df + 0.5f);
    int hi = (int)(f_hi / df + 0.5f);
    if (hi > n / 2 + 1) hi = n / 2 + 1;
    if (lo >= hi) return 0.0f;
    for (int k = lo; k < hi; k++) {
        float re = 0.0f, im = 0.0f;
        float angle = 2.0f * 3.14159265f * (float)k / (float)n;
        for (int i = 0; i < n; i++) {
            re += sig[i] * cosf(angle * i);
            im -= sig[i] * sinf(angle * i);
        }
        energy += (re * re + im * im) / (float)(n * n);
    }
    return energy;
}

static float sig_buf[WINDOW_SIZE];   // buffer temporal para FFT

static void extractMasterFeatures(float features[SPECTRAL_DIM_PER_SENSOR]) {
    const float FS = (float)SAMPLE_HZ;
    const float BW = FS / 2.0f / (float)N_FFT_BINS;
    int feat_idx = 0;
    for (int ch = 0; ch < N_AXES; ch++) {
        float mu = 0.0f;
        for (int t = 0; t < WINDOW_SIZE; t++) {
            int row = (buf_head + t) % WINDOW_SIZE;
            sig_buf[t] = ring[row][ch];
            mu += sig_buf[t];
        }
        mu /= (float)WINDOW_SIZE;
        float var = 0.0f, rms_sum = 0.0f;
        float vmin = 1e9f, vmax = -1e9f;
        for (int t = 0; t < WINDOW_SIZE; t++) {
            sig_buf[t] -= mu;
            var     += sig_buf[t] * sig_buf[t];
            rms_sum += (sig_buf[t] + mu) * (sig_buf[t] + mu);
            if (sig_buf[t] + mu < vmin) vmin = sig_buf[t] + mu;
            if (sig_buf[t] + mu > vmax) vmax = sig_buf[t] + mu;
        }
        features[feat_idx++] = mu;
        features[feat_idx++] = sqrtf(var / (float)WINDOW_SIZE);
        features[feat_idx++] = sqrtf(rms_sum / (float)WINDOW_SIZE);
        features[feat_idx++] = vmax - vmin;
        for (int b = 0; b < N_FFT_BINS; b++) {
            features[feat_idx++] = bandEnergy(sig_buf, WINDOW_SIZE,
                                               b * BW, (b + 1) * BW, FS);
        }
    }
}

// ── Inferencia MLP int8 ───────────────────────────────────────────────────
static float master_feats[SPECTRAL_DIM_PER_SENSOR];
static float full_feats[SPECTRAL_DIM_TOTAL];   // [tobillo(s1,0..83) | pierna(s2,84..167)]

static int runInference(float out_probs[N_CLASSES]) {
    // 1. Extraer features del master (pierna)
    extractMasterFeatures(master_feats);

    // 2. Concatenar [tobillo(84) | pierna(84)]
    // IMPORTANTE: el modelo espectral fue entrenado con spectral_features()
    // que procesa sensores en orden [s1=tobillo, s2=pierna], igual que FEAT_COLS.
    //   full_feats[0..83]   = slave tobillo (sensor_id=1)
    //   full_feats[84..167] = master pierna (sensor_id=2)
    memcpy(full_feats,                          (const float*)slave_features,
           SPECTRAL_DIM_PER_SENSOR * sizeof(float));
    memcpy(full_feats + SPECTRAL_DIM_PER_SENSOR, master_feats,
           SPECTRAL_DIM_PER_SENSOR * sizeof(float));

    // 3. Normalizar con scaler espectral (in-place)
    normalize_spectral(full_feats);   // definida en spectral_scaler.h

    // 4. Cuantizar y llenar tensor de entrada int8
    int8_t*  inp   = input_tens->data.int8;
    float    scale = input_tens->params.scale;
    int32_t  zp    = input_tens->params.zero_point;
    for (int i = 0; i < SPECTRAL_DIM_TOTAL; i++) {
        float q = full_feats[i] / scale + (float)zp;
        inp[i]  = (int8_t)constrain((int)q, -128, 127);
    }

    if (interpreter->Invoke() != kTfLiteOk) return -1;

    int8_t*  out    = output_tens->data.int8;
    float    o_sc   = output_tens->params.scale;
    int32_t  o_zp   = output_tens->params.zero_point;
    int      best   = 0;
    float    best_v = -1e9f;
    for (int c = 0; c < N_CLASSES; c++) {
        float v      = (out[c] - o_zp) * o_sc;
        out_probs[c] = v;
        if (v > best_v) { best_v = v; best = c; }
    }
    return best;
}

// ── setup() ──────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 3000);
    pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
    setLED(false, false, false);

    if (!IMU.begin()) haltError("ERROR:IMU_INIT");
    Serial.print("IMU OK — "); Serial.print(IMU.accelerationSampleRate()); Serial.println(" Hz");

    // TFLite
    tfl_model = tflite::GetModel(gait_model_data);
    if (tfl_model->version() != TFLITE_SCHEMA_VERSION)
        haltError("ERROR:TFLITE_SCHEMA_MISMATCH");

    // Ops para GaitSpectralMLP (solo Dense + Softmax + BatchNorm fusionado)
    static tflite::MicroMutableOpResolver<4> resolver;
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddQuantize();
    resolver.AddDequantize();

    static tflite::MicroInterpreter static_interp(
        tfl_model, resolver, tensor_arena, sizeof(tensor_arena));
    interpreter = &static_interp;

    if (interpreter->AllocateTensors() != kTfLiteOk)
        haltError("ERROR:ARENA_TOO_SMALL — sube TENSOR_ARENA_KB en config.h");

    input_tens  = interpreter->input(0);
    output_tens = interpreter->output(0);

    Serial.print("ARENA:"); Serial.print(interpreter->arena_used_bytes()); Serial.println(" B");
    Serial.print("INPUT_SHAPE:(");
    for (int i = 0; i < input_tens->dims->size; i++) {
        if (i) Serial.print(",");
        Serial.print(input_tens->dims->data[i]);
    }
    Serial.println(")");   // debe ser (1,168)

    // BLE Central
    if (!BLE.begin()) haltError("ERROR:BLE_INIT");
    BLE.scanForName(SLAVE_NAME);
    Serial.print("STATUS:SCANNING "); Serial.println(SLAVE_NAME);
    setLED(true, false, false);
}

// ── Conexión BLE ──────────────────────────────────────────────────────────
static BLEDevice    slave_dev;
static BLECharacteristic slave_char;
static bool         ble_connected = false;

static bool connectSlave() {
    BLEDevice found = BLE.available();
    if (!found) return false;
    BLE.stopScan();
    slave_dev = found;
    if (!slave_dev.connect()) { BLE.scanForName(SLAVE_NAME); return false; }
    if (!slave_dev.discoverAttributes()) { slave_dev.disconnect(); BLE.scanForName(SLAVE_NAME); return false; }
    slave_char = slave_dev.characteristic(BLE_IMU_UUID);
    if (!slave_char || !slave_char.canSubscribe()) {
        slave_dev.disconnect(); BLE.scanForName(SLAVE_NAME); return false;
    }
    slave_char.setEventHandler(BLEUpdated, onSlaveNotify);
    slave_char.subscribe();
    ble_connected = true;
    Serial.print("STATUS:SLAVE_CONNECTED "); Serial.println(SLAVE_NAME);
    setLED(false, false, true);
    return true;
}

// ── loop() ───────────────────────────────────────────────────────────────
static unsigned long last_sample_ms = 0;

void loop() {
    // Conexión BLE
    if (!ble_connected) {
        static unsigned long blink_t = 0;
        static bool led_on = false;
        if (millis() - blink_t > 400) { blink_t = millis(); led_on=!led_on; setLED(led_on,false,false); }
        connectSlave();
        return;
    }
    if (!slave_dev.connected()) {
        ble_connected = false;
        slave_ready   = false;
        buf_count     = 0; buf_head = 0; step_cnt = 0;
        BLE.scanForName(SLAVE_NAME);
        Serial.println("STATUS:SLAVE_LOST");
        setLED(true, false, false);
        return;
    }

    // Leer IMU del master
    unsigned long now = millis();
    if (now - last_sample_ms < (unsigned long)SAMPLE_MS) return;
    last_sample_ms = now;
    if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    ring[buf_head][0]=ax; ring[buf_head][1]=ay; ring[buf_head][2]=az;
    ring[buf_head][3]=gx; ring[buf_head][4]=gy; ring[buf_head][5]=gz;
    buf_head = (buf_head + 1) % WINDOW_SIZE;
    if (buf_count < WINDOW_SIZE) buf_count++;
    step_cnt++;

    if (buf_count == WINDOW_SIZE && step_cnt == 1)
        setLED(false, true, false);   // verde: buffer listo

    // Inferir cuando buffer lleno + STEP nuevas muestras + slave listo
    if (buf_count < WINDOW_SIZE || step_cnt < STEP_SIZE || !slave_ready) return;
    step_cnt   = 0;
    slave_ready = false;

    float probs[N_CLASSES];
    int phase = runInference(probs);
    if (phase < 0) { Serial.println("ERROR:INFERENCE_FAILED"); return; }

    Serial.print("INFER,"); Serial.print(PHASE_NAMES[phase]);
    for (int c = 0; c < N_CLASSES; c++) {
        Serial.print(","); Serial.print(probs[c], 3);
    }
    Serial.println();
}
