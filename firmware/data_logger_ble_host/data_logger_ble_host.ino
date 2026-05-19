#include "config.h"
#include <ArduinoBLE.h>

#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// ─── GATT: servicio hub (Hub → PC) ───────────────────────────────────────────
BLEService        hubService(HUB_SERVICE_UUID);
BLECharacteristic imuChar1(HUB_IMU1_UUID, BLENotify, PKT_SIZE);   // pelvis (propio)
BLECharacteristic imuChar2(HUB_IMU2_UUID, BLENotify, PKT_SIZE);   // thigh  (relay)
BLECharacteristic imuChar3(HUB_IMU3_UUID, BLENotify, PKT_SIZE);   // ankle  (relay)
BLECharacteristic cmdChar(HUB_CMD_UUID, BLEWriteWithoutResponse, 32);
BLECharacteristic stsChar(HUB_STS_UUID, BLERead, 128);

// ─── Conexiones centrales a los esclavos ─────────────────────────────────────
static BLEDevice       slave2, slave3;
static BLECharacteristic imuCh_s2, imuCh_s3;
static BLECharacteristic cmdCh_s2, cmdCh_s3;

// ─── Máquina de estados ───────────────────────────────────────────────────────
enum HubState { SCAN_S2, SCAN_S3, READY, CAPTURING };
static HubState hubState = SCAN_S2;

// Qué esclavos se conectaron al inicio (para no intentar reconectar los que nunca estuvieron)
static bool slave2_active = false;
static bool slave3_active = false;
static uint32_t scan_start_ms = 0;   // para el timeout de escaneo
static bool pc_was_connected = false;

// ─── Buffer de batch IMU propio ───────────────────────────────────────────────
static uint8_t  pkt_buf[PKT_SIZE];
static uint16_t pkt_seq        = 0;
static int      batch_idx      = 0;
static uint32_t last_sample_us = 0;
static uint32_t sample_count   = 0;

// ─── Sesión ───────────────────────────────────────────────────────────────────
static String trial_id   = "t000";
static String subject_id = "s000";
static String cmd_buf    = "";

// ─── LED ──────────────────────────────────────────────────────────────────────
static void set_led(bool r, bool g, bool b) {
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

// ─── Status BLE characteristic ───────────────────────────────────────────────
static void update_status_char() {
  char buf[128];
  snprintf(buf, sizeof(buf), "%d,%s,%s,%s,s2=%d,s3=%d",
           SENSOR_ID, PLACEMENT,
           trial_id.c_str(), subject_id.c_str(),
           slave2.connected() ? 1 : 0,
           slave3.connected() ? 1 : 0);
  stsChar.writeValue(buf);
}

// ─── Conectar a un esclavo y suscribir su característica IMU ─────────────────
static bool connect_slave(BLEDevice peripheral,
                           BLEDevice& out_dev,
                           BLECharacteristic& out_imu,
                           BLECharacteristic& out_cmd) {
  Serial.print("  Connecting: "); Serial.println(peripheral.localName());
  if (!peripheral.connect()) {
    Serial.println("  ERR: connect failed"); return false;
  }
  if (!peripheral.discoverAttributes()) {
    Serial.println("  ERR: discover failed");
    peripheral.disconnect(); return false;
  }
  BLECharacteristic imuCh = peripheral.characteristic(NODE_IMU_UUID);
  BLECharacteristic cmdCh = peripheral.characteristic(NODE_CMD_UUID);
  if (!imuCh || !cmdCh) {
    Serial.println("  ERR: chars not found");
    peripheral.disconnect(); return false;
  }
  if (!imuCh.canSubscribe() || !imuCh.subscribe()) {
    Serial.println("  ERR: subscribe failed");
    peripheral.disconnect(); return false;
  }
  out_dev = peripheral;
  out_imu = imuCh;
  out_cmd = cmdCh;
  Serial.print("  OK "); Serial.println(peripheral.address());
  return true;
}

// ─── Enviar comando a un esclavo ─────────────────────────────────────────────
static void send_cmd(BLECharacteristic& cmdCh, const char* cmd) {
  cmdCh.writeValue((const uint8_t*)cmd, strlen(cmd));
}

// ─── Procesar comando (desde PC por BLE o por Serial de debug) ───────────────
static void flush_batch();   // forward declaration

static void process_command(const String& cmd) {
  if (cmd == "START" && hubState == READY) {
    if (slave2_active) send_cmd(cmdCh_s2, "START");
    if (slave3_active) send_cmd(cmdCh_s3, "START");
    last_sample_us = micros();
    sample_count   = 0;
    batch_idx      = 0;
    pkt_buf[0]     = (uint8_t)SENSOR_ID;
    hubState       = CAPTURING;
    set_led(false, true, false);   // verde
    Serial.println("ACK:START");

  } else if (cmd == "STOP" && hubState == CAPTURING) {
    if (slave2_active) send_cmd(cmdCh_s2, "STOP");
    if (slave3_active) send_cmd(cmdCh_s3, "STOP");
    if (batch_idx > 0) flush_batch();
    hubState = READY;
    set_led(false, false, true);   // azul
    Serial.print("ACK:STOP:"); Serial.println(sample_count);

  } else if (cmd.startsWith("SET_TRIAL:")) {
    trial_id = cmd.substring(10); trial_id.trim();
    if (slave2_active) send_cmd(cmdCh_s2, cmd.c_str());
    if (slave3_active) send_cmd(cmdCh_s3, cmd.c_str());
    update_status_char();
    Serial.print("ACK:SET_TRIAL:"); Serial.println(trial_id);

  } else if (cmd.startsWith("SET_SUBJECT:")) {
    subject_id = cmd.substring(12); subject_id.trim();
    if (slave2_active) send_cmd(cmdCh_s2, cmd.c_str());
    if (slave3_active) send_cmd(cmdCh_s3, cmd.c_str());
    update_status_char();
    Serial.print("ACK:SET_SUBJECT:"); Serial.println(subject_id);

  } else if (cmd == "STATUS") {
    Serial.print("STATUS:hub,sensor_id="); Serial.print(SENSOR_ID);
    Serial.print(",placement=");  Serial.print(PLACEMENT);
    Serial.print(",s2=");  Serial.print(slave2.connected() ? 1 : 0);
    Serial.print(",s3=");  Serial.print(slave3.connected() ? 1 : 0);
    Serial.print(",capturing="); Serial.println(hubState == CAPTURING ? 1 : 0);
  }
}

// ─── Leer IMU propio y acumular en batch ─────────────────────────────────────
static void flush_batch() {
  pkt_buf[1] = (uint8_t)(pkt_seq);
  pkt_buf[2] = (uint8_t)(pkt_seq >> 8);
  pkt_seq++;
  if (BLE.connected()) {
    imuChar1.writeValue(pkt_buf, PKT_SIZE);
  }
  batch_idx = 0;
}

static void read_imu_to_batch() {
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;
  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);
  uint32_t ts = millis();

  int off = 3 + batch_idx * 16;
  pkt_buf[off+0] = (uint8_t)(ts);       pkt_buf[off+1] = (uint8_t)(ts >> 8);
  pkt_buf[off+2] = (uint8_t)(ts >> 16); pkt_buf[off+3] = (uint8_t)(ts >> 24);

  auto to_acc  = [](float v) -> int16_t {
    return (int16_t)constrain((long)(v * 10000.0f), -32768L, 32767L);
  };
  auto to_gyro = [](float v) -> int16_t {
    return (int16_t)constrain((long)(v * 10.0f), -32768L, 32767L);
  };

  int16_t ax16 = to_acc(ax),  ay16 = to_acc(ay),  az16 = to_acc(az);
  int16_t gx16 = to_gyro(gx), gy16 = to_gyro(gy), gz16 = to_gyro(gz);

  pkt_buf[off+4]  = (uint8_t)(ax16);       pkt_buf[off+5]  = (uint8_t)(ax16 >> 8);
  pkt_buf[off+6]  = (uint8_t)(ay16);       pkt_buf[off+7]  = (uint8_t)(ay16 >> 8);
  pkt_buf[off+8]  = (uint8_t)(az16);       pkt_buf[off+9]  = (uint8_t)(az16 >> 8);
  pkt_buf[off+10] = (uint8_t)(gx16);       pkt_buf[off+11] = (uint8_t)(gx16 >> 8);
  pkt_buf[off+12] = (uint8_t)(gy16);       pkt_buf[off+13] = (uint8_t)(gy16 >> 8);
  pkt_buf[off+14] = (uint8_t)(gz16);       pkt_buf[off+15] = (uint8_t)(gz16 >> 8);

  batch_idx++;
  sample_count++;
  if (batch_idx >= BATCH_SIZE) flush_batch();
}

// ─── Relay: reenviar notificación de esclavo al PC ───────────────────────────
static void relay_if_updated(BLEDevice& dev, BLECharacteristic& imuCh,
                              BLECharacteristic& outChar) {
  if (!dev.connected()) return;
  dev.poll();
  if (imuCh.valueUpdated()) {
    const uint8_t* buf = imuCh.value();
    int len = imuCh.valueLength();
    if (BLE.connected()) {
      outChar.writeValue(buf, len);
    }
  }
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(BAUD_RATE);
  pinMode(LEDR, OUTPUT); digitalWrite(LEDR, HIGH);
  pinMode(LEDG, OUTPUT); digitalWrite(LEDG, HIGH);
  pinMode(LEDB, OUTPUT); digitalWrite(LEDB, HIGH);
  set_led(true, false, false);   // rojo = inicializando

  if (!IMU.begin()) {
    set_led(true, false, false);
    while (true) delay(500);
  }

  if (!BLE.begin()) {
    Serial.println("ERROR:BLE_NOT_FOUND");
    while (true) delay(500);
  }

  // Configurar servicio hub (peripheral side)
  BLE.setLocalName(HUB_DEVICE_NAME);
  BLE.setAdvertisedService(hubService);
  hubService.addCharacteristic(imuChar1);
  hubService.addCharacteristic(imuChar2);
  hubService.addCharacteristic(imuChar3);
  hubService.addCharacteristic(cmdChar);
  hubService.addCharacteristic(stsChar);
  BLE.addService(hubService);
  update_status_char();

  pkt_buf[0] = (uint8_t)SENSOR_ID;

  // Iniciar escaneo de esclavos
  Serial.print("SCAN:"); Serial.println(SLAVE2_NAME);
  Serial.print("  (timeout="); Serial.print(SLAVE_SCAN_TIMEOUT_MS / 1000);
  Serial.println("s por esclavo)");
  BLE.scanForName(SLAVE2_NAME);
  scan_start_ms = millis();
  hubState = SCAN_S2;
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  BLE.poll();

  // Serial de debug
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

  // ── Escaneo y conexión a esclavos (con timeout) ──────────────────────────
  if (hubState == SCAN_S2) {
    BLEDevice d = BLE.available();
    bool timed_out = (millis() - scan_start_ms) >= SLAVE_SCAN_TIMEOUT_MS;
    if (d) {
      BLE.stopScan();
      if (connect_slave(d, slave2, imuCh_s2, cmdCh_s2)) {
        slave2_active = true;
        Serial.println("SLAVE2:OK");
        update_status_char();
      } else {
        Serial.println("SLAVE2:CONNECT_FAILED — skip");
      }
      // Si slave3 ya está conectado o no es activo, ir directo a READY
      if (!slave3_active || slave3.connected()) {
        delay(200);
        BLE.advertise();
        hubState = READY;
        set_led(false, false, true);
        Serial.print("READY:HUB:"); Serial.print(HUB_DEVICE_NAME);
        Serial.print(" s2="); Serial.print(slave2_active ? 1 : 0);
        Serial.print(" s3="); Serial.println(slave3_active ? 1 : 0);
      } else {
        Serial.print("SCAN:"); Serial.println(SLAVE3_NAME);
        BLE.scanForName(SLAVE3_NAME);
        scan_start_ms = millis();
        hubState = SCAN_S3;
      }
    } else if (timed_out) {
      BLE.stopScan();
      Serial.println("SLAVE2:TIMEOUT — skip");
      if (!slave3_active || slave3.connected()) {
        delay(200);
        BLE.advertise();
        hubState = READY;
        set_led(false, false, true);
        Serial.print("READY:HUB:"); Serial.print(HUB_DEVICE_NAME);
        Serial.print(" s2="); Serial.print(slave2_active ? 1 : 0);
        Serial.print(" s3="); Serial.println(slave3_active ? 1 : 0);
      } else {
        Serial.print("SCAN:"); Serial.println(SLAVE3_NAME);
        BLE.scanForName(SLAVE3_NAME);
        scan_start_ms = millis();
        hubState = SCAN_S3;
      }
    }
    return;
  }

  if (hubState == SCAN_S3) {
    BLEDevice d = BLE.available();
    bool timed_out = (millis() - scan_start_ms) >= SLAVE_SCAN_TIMEOUT_MS;
    if (d) {
      BLE.stopScan();
      if (connect_slave(d, slave3, imuCh_s3, cmdCh_s3)) {
        slave3_active = true;
        Serial.println("SLAVE3:OK");
        update_status_char();
      } else {
        Serial.println("SLAVE3:CONNECT_FAILED — skip");
      }
    } else if (!timed_out) {
      return;   // seguir esperando
    } else {
      BLE.stopScan();
      Serial.println("SLAVE3:TIMEOUT — skip");
    }
    // Llegar aquí = avanzar a READY (con los esclavos que haya)
    delay(200);   // pausa para que el stack BLE limpie el estado de escaneo
    BLE.advertise();
    hubState = READY;
    set_led(false, false, true);   // azul = esperando PC
    Serial.print("READY:HUB:"); Serial.print(HUB_DEVICE_NAME);
    Serial.print(" s2="); Serial.print(slave2_active ? 1 : 0);
    Serial.print(" s3="); Serial.println(slave3_active ? 1 : 0);
    return;
  }

  // ── Re-anunciar si PC se desconectó ─────────────────────────────────────
  bool pc_now = BLE.connected();
  if (pc_was_connected && !pc_now && (hubState == READY || hubState == CAPTURING)) {
    if (hubState == CAPTURING) process_command("STOP");
    hubState = READY;
    set_led(false, false, true);
    BLE.advertise();
    Serial.println("PC:DISCONNECTED — re-advertising");
  }
  pc_was_connected = pc_now;

  // ── READY / CAPTURING ────────────────────────────────────────────────────
  // Reconexión automática solo para los esclavos que estaban activos al inicio
  if (slave2_active && !slave2.connected()) {
    Serial.println("SLAVE2:LOST — reconnecting");
    if (hubState == CAPTURING) process_command("STOP");
    set_led(true, false, false);
    BLE.scanForName(SLAVE2_NAME);
    scan_start_ms = millis();
    hubState = SCAN_S2;
    return;
  }
  if (slave3_active && !slave3.connected()) {
    Serial.println("SLAVE3:LOST — reconnecting");
    if (hubState == CAPTURING) process_command("STOP");
    set_led(true, false, false);
    BLE.scanForName(SLAVE3_NAME);
    scan_start_ms = millis();
    hubState = SCAN_S3;
    return;
  }

  // Comandos BLE del PC
  if (cmdChar.written()) {
    char buf[33] = {0};
    cmdChar.readValue(buf, min((int)cmdChar.valueLength(), 32));
    process_command(String(buf));
  }

  if (hubState == CAPTURING) {
    // IMU propio
    uint32_t now = micros();
    if ((uint32_t)(now - last_sample_us) >= SAMPLE_PERIOD_US) {
      last_sample_us += SAMPLE_PERIOD_US;
      read_imu_to_batch();
    }
    // Relay solo de los esclavos que estaban activos al inicio
    if (slave2_active) relay_if_updated(slave2, imuCh_s2, imuChar2);
    if (slave3_active) relay_if_updated(slave3, imuCh_s3, imuChar3);
  }
}
