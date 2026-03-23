#include <Arduino.h>
#include <Wire.h>
extern "C" {
  #include <espnow.h>
}
#include <ESP8266WiFi.h>

// =========================
// CONFIG
// =========================
#define STICK_ID                1   // mude para 1 ou 2
#define I2C_SDA_PIN             4   // GPIO4 = D2 em muitas placas
#define I2C_SCL_PIN             5   // GPIO5 = D1 em muitas placas
#define MPU6050_ADDR            0x68

// MAC da CENTRAL ESP32
// descubra no Serial da central e copie aqui
uint8_t PEER_MAC[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};

// canal Wi-Fi fixo para ESP-NOW
#define ESPNOW_CHANNEL          1

// taxa de leitura
#define SAMPLE_PERIOD_MS        4     // ~250 Hz

// limiares
#define HIT_THRESHOLD_G         1.85f // aceleração dinâmica mínima
#define MIN_HIT_GAP_MS          70
#define VELOCITY_MIN            15
#define VELOCITY_MAX            127

// filtro
#define HP_ALPHA                0.92f

// =========================
// PACOTE
// =========================
#pragma pack(push, 1)
struct HitPacket {
  uint8_t  stickId;
  uint8_t  zone;       // 0..7
  uint8_t  velocity;   // 1..127
  uint8_t  flags;      // reservado
  uint16_t peakMilliG; // pico em mG
  uint16_t dtMs;       // intervalo desde ultimo hit
};
#pragma pack(pop)

// zones sugeridas
enum HitZone : uint8_t {
  ZONE_SNARE = 0,
  ZONE_HHC   = 1,
  ZONE_HHO   = 2,
  ZONE_TOM1  = 3,
  ZONE_TOM2  = 4,
  ZONE_CRASH = 5,
  ZONE_RIDE  = 6,
  ZONE_FX    = 7
};

volatile bool sendOk = false;
volatile bool sendDone = false;

// =========================
// MPU6050
// =========================
void mpuWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool mpuReadRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14);
  if (Wire.available() < 14) {
    return false;
  }

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

void initMPU6050() {
  // wake
  mpuWrite(0x6B, 0x00);

  // accel ±4g
  mpuWrite(0x1C, 0x08);

  // gyro ±500 dps
  mpuWrite(0x1B, 0x08);

  // DLPF ~44Hz
  mpuWrite(0x1A, 0x03);

  // sample divider
  mpuWrite(0x19, 0x04);
}

// =========================
// ESPNOW
// =========================
void onDataSent(uint8_t *mac_addr, uint8_t status) {
  sendOk = (status == 0);
  sendDone = true;
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(ESPNOW_CHANNEL);

  if (esp_now_init() != 0) {
    return false;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(onDataSent);

  if (esp_now_add_peer(PEER_MAC, ESP_NOW_ROLE_SLAVE, ESPNOW_CHANNEL, NULL, 0) != 0) {
    return false;
  }

  return true;
}

bool sendHit(uint8_t zone, uint8_t velocity, uint16_t peakMilliG, uint16_t dtMs) {
  HitPacket pkt;
  pkt.stickId = STICK_ID;
  pkt.zone = zone;
  pkt.velocity = velocity;
  pkt.flags = 0;
  pkt.peakMilliG = peakMilliG;
  pkt.dtMs = dtMs;

  sendDone = false;
  int rc = esp_now_send(PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (rc != 0) {
    return false;
  }

  uint32_t t0 = millis();
  while (!sendDone && millis() - t0 < 20) {
    delay(0);
  }
  return sendDone && sendOk;
}

// =========================
// DETECÇÃO
// =========================
float hpDyn = 0.0f;
float lastMag = 1.0f;
float lastAxG = 0.0f;
float lastAyG = 0.0f;
float lastAzG = 1.0f;
uint32_t lastHitMs = 0;

uint8_t clampVelocity(float dynG) {
  float x = (dynG - HIT_THRESHOLD_G) / 3.5f;
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;
  int v = VELOCITY_MIN + (int)(x * (VELOCITY_MAX - VELOCITY_MIN));
  if (v < 1) v = 1;
  if (v > 127) v = 127;
  return (uint8_t)v;
}

uint8_t classifyZone(float axg, float ayg, float azg) {
  // classificação simples por direção do pico
  // ajuste depois no teu uso real

  if (STICK_ID == 1) {
    // esquerda
    if (ayg < -1.2f) return ZONE_SNARE;
    if (axg < -1.1f) return ZONE_HHC;
    if (axg >  1.1f) return ZONE_HHO;
    if (azg < -1.2f) return ZONE_TOM1;
    return ZONE_SNARE;
  } else {
    // direita
    if (ayg < -1.2f) return ZONE_SNARE;
    if (axg >  1.2f) return ZONE_CRASH;
    if (axg < -1.2f) return ZONE_RIDE;
    if (azg < -1.2f) return ZONE_TOM2;
    return ZONE_SNARE;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  initMPU6050();

  if (!initEspNow()) {
    Serial.println("ERRO: falha no ESP-NOW");
    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("=== AIR DRUM STICK ESP8266 ===");
  Serial.printf("Stick ID: %d\n", STICK_ID);
  Serial.printf("WiFi MAC : %s\n", WiFi.macAddress().c_str());
}

void loop() {
  static uint32_t lastSample = 0;
  uint32_t nowMs = millis();

  if (nowMs - lastSample < SAMPLE_PERIOD_MS) {
    delay(0);
    return;
  }
  lastSample = nowMs;

  int16_t ax, ay, az, gx, gy, gz;
  if (!mpuReadRaw(ax, ay, az, gx, gy, gz)) {
    return;
  }

  // accel ±4g => 8192 LSB/g
  float axg = (float)ax / 8192.0f;
  float ayg = (float)ay / 8192.0f;
  float azg = (float)az / 8192.0f;

  float mag = sqrtf(axg * axg + ayg * ayg + azg * azg);

  // high-pass simples em torno de 1g
  float dyn = HP_ALPHA * (hpDyn + mag - lastMag);
  hpDyn = dyn;
  lastMag = mag;

  bool hitReady = false;
  float peakDyn = 0.0f;

  static bool armed = true;
  static float peakAx = 0.0f, peakAy = 0.0f, peakAz = 0.0f;

  if (armed && dyn > HIT_THRESHOLD_G && (nowMs - lastHitMs) > MIN_HIT_GAP_MS) {
    peakDyn = dyn;
    peakAx = axg;
    peakAy = ayg;
    peakAz = azg;
    armed = false;
    hitReady = true;
  }

  if (!armed && dyn < 0.35f) {
    armed = true;
  }

  lastAxG = axg;
  lastAyG = ayg;
  lastAzG = azg;

  if (hitReady) {
    uint8_t zone = classifyZone(peakAx, peakAy, peakAz);
    uint8_t velocity = clampVelocity(peakDyn);
    uint16_t peakMilliG = (uint16_t)(peakDyn * 1000.0f);
    uint16_t dt = (uint16_t)(nowMs - lastHitMs);
    lastHitMs = nowMs;

    bool ok = sendHit(zone, velocity, peakMilliG, dt);

    Serial.printf("hit z=%u vel=%u peak=%.2fg tx=%s\n",
                  zone, velocity, peakDyn, ok ? "ok" : "fail");
  }
}
