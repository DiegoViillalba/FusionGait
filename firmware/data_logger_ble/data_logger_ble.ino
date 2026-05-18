#include "config.h"
#include <ArduinoBLE.h>

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// ─── Formato del paquete BLE (83 bytes) ──────────────────────────────────────
//
//  Byte 0        uint8   sensor_id
//  Bytes 1-2     uint16  packet_seq  (little-endian, wraps en 65535)
//  Por cada muestra i en [0, BATCH_SIZE):
//    Bytes 3+i*16 .. +3   uint32  timestamp_ms
//    Bytes +4 .. +5       int16   ax × 10000   (resolución 0.1 mg)
//    Bytes +6 .. +7       int16   ay × 10000
//    Bytes +8 .. +9       int16   az × 10000
//    Bytes +10 .. +11     int16   gx × 10      (resolución 0.1 °/s)
//    Bytes +12 .. +13     int16   gy × 10
//    Bytes +14 .. +15     int16   gz × 10
//  Total: 3 + BATCH_SIZE * 16 = 83 bytes

static const int PKT_SIZE  = 3 + BATCH_SIZE * 16;

// ─── GATT ─────────────────────────────────────────────────────────────────────
BLEService        gaitService(SERVICE_UUID);
BLECharacteristic imuChar(CHAR_IMU_UUID, BLENotify, PKT_SIZE);
BLECharacteristic cmdChar(CHAR_CMD_UUID, BLEWriteWithoutResponse, 32);
BLECharacteristic stsChar(CHAR_STS_UUID, BLERead, 64);

// ─── Estado ───────────────────────────────────────────────────────────────────
enum State { IDLE, CAPTURING };
static State    state        = IDLE;
static String   trial_id     = "t000";
static String   subject_id   = "s000";
static String   cmd_buf      = "";    // acumulador para comandos por Serial

// Timing IMU
static uint32_t last_sample_us = 0;
static uint32_t sample_count   = 0;

// Buffer de batch BLE
static uint8_t  pkt_buf[PKT_SIZE];
static uint16_t pkt_seq   = 0;
static int      batch_idx = 0;   // cuántas muestras hay en el buffer actual

// ─── Prototipos ───────────────────────────────────────────────────────────────
static void process_command(const String& cmd);
static void read_imu_to_batch();
static void flush_batch();
static void update_status_char();
static void set_led(bool r, bool g, bool b);

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(100);

  pinMode(LEDR, OUTPUT); digitalWrite(LEDR, HIGH);
  pinMode(LEDG, OUTPUT); digitalWrite(LEDG, HIGH);
  pinMode(LEDB, OUTPUT); digitalWrite(LEDB, HIGH);

  // IMU
  if (!IMU.begin()) {
    set_led(true, false, false);
    while (true) {
      if (Serial.available()) {
        String s = Serial.readStringUntil('\n');
        if (s.indexOf("STATUS") >= 0)
          Serial.println("ERROR:IMU_NOT_FOUND");
      }
      delay(300);
    }
  }

  // BLE
  if (!BLE.begin()) {
    set_led(true, false, false);
    Serial.println("ERROR:BLE_NOT_FOUND");
    while (true) delay(500);
  }

  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(gaitService);

  gaitService.addCharacteristic(imuChar);
  gaitService.addCharacteristic(cmdChar);
  gaitService.addCharacteristic(stsChar);
  BLE.addService(gaitService);

  update_status_char();
  BLE.advertise();

  // Inicializar header del paquete (sensor_id no cambia)
  pkt_buf[0] = (uint8_t)SENSOR_ID;

  set_led(false, false, true);   // azul = esperando conexión
  Serial.print("READY:BLE:");
  Serial.println(BLE_DEVICE_NAME);
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  BLE.poll();

  // ── Comandos por Serial (debug / wired fallback) ─────────────────────────
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmd_buf.trim();
      if (cmd_buf.length() > 0) process_command(cmd_buf);
      cmd_buf = "";
    } else if (c != '\r') {
      cmd_buf += c;
      if (cmd_buf.length() > 64) cmd_buf = "";
    }
  }

  // ── Comandos por BLE (Write Without Response) ────────────────────────────
  if (cmdChar.written()) {
    int len = cmdChar.valueLength();
    char buf[33] = {0};
    cmdChar.readValue(buf, min(len, 32));
    process_command(String(buf));
  }

  // ── Muestreo IMU ─────────────────────────────────────────────────────────
  if (state == CAPTURING) {
    uint32_t now = micros();
    if ((uint32_t)(now - last_sample_us) >= SAMPLE_PERIOD_US) {
      last_sample_us += SAMPLE_PERIOD_US;
      read_imu_to_batch();
    }
  }
}

// ─── Comandos ─────────────────────────────────────────────────────────────────
static void process_command(const String& cmd) {
  if (cmd == "START") {
    state          = CAPTURING;
    last_sample_us = micros();
    sample_count   = 0;
    batch_idx      = 0;
    set_led(false, true, false);   // verde
    Serial.println("ACK:START");

  } else if (cmd == "STOP") {
    if (batch_idx > 0) flush_batch();  // enviar muestras pendientes
    state = IDLE;
    set_led(false, false, true);
    Serial.print("ACK:STOP:"); Serial.println(sample_count);

  } else if (cmd.startsWith("SET_TRIAL:")) {
    trial_id = cmd.substring(10); trial_id.trim();
    update_status_char();
    Serial.print("ACK:SET_TRIAL:"); Serial.println(trial_id);

  } else if (cmd.startsWith("SET_SUBJECT:")) {
    subject_id = cmd.substring(12); subject_id.trim();
    update_status_char();
    Serial.print("ACK:SET_SUBJECT:"); Serial.println(subject_id);

  } else if (cmd == "SYNC") {
    Serial.print("SYNC_ACK:"); Serial.println(millis());

  } else if (cmd == "STATUS") {
    Serial.print("STATUS:sensor_id=");  Serial.print(SENSOR_ID);
    Serial.print(",ble=");              Serial.print(BLE_DEVICE_NAME);
    Serial.print(",placement=");        Serial.print(PLACEMENT);
    Serial.print(",trial=");            Serial.print(trial_id);
    Serial.print(",subject=");          Serial.print(subject_id);
    Serial.print(",capturing=");        Serial.print(state == CAPTURING ? 1 : 0);
    Serial.print(",samples=");          Serial.println(sample_count);
  }
}

// ─── Leer IMU y acumular en batch ─────────────────────────────────────────────
static void read_imu_to_batch() {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  uint32_t ts = millis();

  // Posición dentro del paquete
  int off = 3 + batch_idx * 16;

  // timestamp uint32 little-endian
  pkt_buf[off+0] = (uint8_t)(ts);
  pkt_buf[off+1] = (uint8_t)(ts >> 8);
  pkt_buf[off+2] = (uint8_t)(ts >> 16);
  pkt_buf[off+3] = (uint8_t)(ts >> 24);

  // ax,ay,az como int16 × 10000
  auto to_acc = [](float v) -> int16_t {
    return (int16_t)constrain((long)(v * 10000.0f), -32768L, 32767L);
  };
  auto to_gyro = [](float v) -> int16_t {
    return (int16_t)constrain((long)(v * 10.0f), -32768L, 32767L);
  };

  int16_t ax16 = to_acc(ax),  ay16 = to_acc(ay),  az16 = to_acc(az);
  int16_t gx16 = to_gyro(gx), gy16 = to_gyro(gy), gz16 = to_gyro(gz);

  pkt_buf[off+4]  = (uint8_t)(ax16);        pkt_buf[off+5]  = (uint8_t)(ax16 >> 8);
  pkt_buf[off+6]  = (uint8_t)(ay16);        pkt_buf[off+7]  = (uint8_t)(ay16 >> 8);
  pkt_buf[off+8]  = (uint8_t)(az16);        pkt_buf[off+9]  = (uint8_t)(az16 >> 8);
  pkt_buf[off+10] = (uint8_t)(gx16);        pkt_buf[off+11] = (uint8_t)(gx16 >> 8);
  pkt_buf[off+12] = (uint8_t)(gy16);        pkt_buf[off+13] = (uint8_t)(gy16 >> 8);
  pkt_buf[off+14] = (uint8_t)(gz16);        pkt_buf[off+15] = (uint8_t)(gz16 >> 8);

  batch_idx++;
  sample_count++;

  if (batch_idx >= BATCH_SIZE) flush_batch();
}

// ─── Enviar batch por BLE ─────────────────────────────────────────────────────
static void flush_batch() {
  // Escribir packet_seq en bytes 1-2
  pkt_buf[1] = (uint8_t)(pkt_seq);
  pkt_buf[2] = (uint8_t)(pkt_seq >> 8);
  pkt_seq++;

  if (BLE.connected()) {
    imuChar.writeValue(pkt_buf, PKT_SIZE);
  }
  batch_idx = 0;
}

// ─── Actualizar característica de status (Read) ───────────────────────────────
static void update_status_char() {
  char buf[64];
  snprintf(buf, sizeof(buf), "%d,%s,%s,%s",
           SENSOR_ID, PLACEMENT, trial_id.c_str(), subject_id.c_str());
  stsChar.writeValue(buf);
}

// ─── LEDs ─────────────────────────────────────────────────────────────────────
static void set_led(bool r, bool g, bool b) {
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}
