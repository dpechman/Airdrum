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
#define I2C_SDA_PIN             2   // GPIO2 = D4
#define I2C_SCL_PIN             0   // GPIO0 = D3
#define MPU6050_ADDR            0x68

// MAC da CENTRAL ESP32
// descubra no Serial da central e copie aqui
uint8_t PEER_MAC[6] = {0xA8, 0x46, 0x74, 0x92, 0x9A, 0x48};

// canal Wi-Fi fixo para ESP-NOW
#define ESPNOW_CHANNEL          1

// taxa de leitura
#define SAMPLE_PERIOD_MS        4     // ~250 Hz

// limiares
#define HIT_THRESHOLD_G         2.2f  // aceleração dinâmica mínima
#define MIN_HIT_GAP_MS          20    // gap mínimo — apenas evita duplo trigger no mesmo impacto
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
bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_channel(ESPNOW_CHANNEL);

  if (esp_now_init() != 0) {
    return false;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);

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

  int rc = esp_now_send(PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  return (rc == 0);
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

  // scanner I2C
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("I2C: 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) Serial.println("I2C: nenhum!");

  initMPU6050();

  if (!initEspNow()) {
    Serial.println("ESPNOW FAIL");
    while (true) delay(1000);
  }

  Serial.printf("STICK %d MAC:%s\n", STICK_ID, WiFi.macAddress().c_str());
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

  // re-arma por nível de sinal OU por tempo desde o último hit (para batidas rápidas)
  if (!armed && (dyn < 0.35f || (nowMs - lastHitMs) > MIN_HIT_GAP_MS)) {
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
    // z=zone v=velocity p=peak(mG) dt=intervalo tx=ok/fail
    Serial.printf("HIT z=%u v=%u p=%u dt=%u %s\n",
                  zone, velocity, peakMilliG, dt, ok ? "OK" : "FAIL");
  }
}
