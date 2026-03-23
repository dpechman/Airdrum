#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>
#include <math.h>

// =========================
// CONFIG GERAL
// =========================
#define ESPNOW_CHANNEL          1

// I2S -> PCM5102
#define I2S_PORT                I2S_NUM_0
#define I2S_BCLK_PIN            26
#define I2S_LRCLK_PIN           25
#define I2S_DOUT_PIN            22

#define SAMPLE_RATE             22050
#define AUDIO_BLOCK_SAMPLES     256

// vozes
#define MAX_VOICES              16

// =========================
// PACOTE
// =========================
#pragma pack(push, 1)
struct HitPacket {
  uint8_t  stickId;
  uint8_t  zone;
  uint8_t  velocity;
  uint8_t  flags;
  uint16_t peakMilliG;
  uint16_t dtMs;
};
#pragma pack(pop)

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

enum VoiceType : uint8_t {
  VOICE_NONE = 0,
  VOICE_KICK,
  VOICE_SNARE,
  VOICE_HAT_C,
  VOICE_HAT_O,
  VOICE_TOM1,
  VOICE_TOM2,
  VOICE_CRASH,
  VOICE_RIDE
};

struct Voice {
  bool active;
  VoiceType type;
  float phase;
  float phase2;
  float env;
  float decay;
  float freq;
  float freq2;
  float gain;
  uint32_t ageSamples;
};

static Voice voices[MAX_VOICES];

// fila simples para hits
struct PendingHit {
  uint8_t zone;
  uint8_t velocity;
  uint8_t stickId;
};

constexpr int HIT_QUEUE_SIZE = 64;
static PendingHit hitQueue[HIT_QUEUE_SIZE];
volatile uint16_t hitWrite = 0;
volatile uint16_t hitRead = 0;

portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;

// PRNG simples para noise
static uint32_t rngState = 0x12345678;

static inline float frandSigned() {
  rngState ^= rngState << 13;
  rngState ^= rngState >> 17;
  rngState ^= rngState << 5;
  uint32_t v = rngState & 0x7FFF;
  return ((float)v / 16384.0f) - 1.0f;
}

// =========================
// FILA
// =========================
bool enqueueHit(uint8_t zone, uint8_t velocity, uint8_t stickId) {
  portENTER_CRITICAL_ISR(&queueMux);
  uint16_t next = (hitWrite + 1) % HIT_QUEUE_SIZE;
  if (next == hitRead) {
    portEXIT_CRITICAL_ISR(&queueMux);
    return false;
  }
  hitQueue[hitWrite] = { zone, velocity, stickId };
  hitWrite = next;
  portEXIT_CRITICAL_ISR(&queueMux);
  return true;
}

bool dequeueHit(PendingHit &h) {
  portENTER_CRITICAL(&queueMux);
  if (hitRead == hitWrite) {
    portEXIT_CRITICAL(&queueMux);
    return false;
  }
  h = hitQueue[hitRead];
  hitRead = (hitRead + 1) % HIT_QUEUE_SIZE;
  portEXIT_CRITICAL(&queueMux);
  return true;
}

// =========================
// ÁUDIO
// =========================
void initI2S() {
  i2s_config_t i2s_config = {};
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  i2s_config.sample_rate = SAMPLE_RATE;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = AUDIO_BLOCK_SAMPLES;
  i2s_config.use_apll = false;
  i2s_config.tx_desc_auto_clear = true;
  i2s_config.fixed_mclk = 0;

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = I2S_BCLK_PIN;
  pin_config.ws_io_num = I2S_LRCLK_PIN;
  pin_config.data_out_num = I2S_DOUT_PIN;
  pin_config.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_set_clk(I2S_PORT, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

int allocVoice() {
  for (int i = 0; i < MAX_VOICES; i++) {
    if (!voices[i].active) return i;
  }

  // rouba a voz mais velha
  uint32_t oldestAge = 0;
  int oldest = 0;
  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].ageSamples > oldestAge) {
      oldestAge = voices[i].ageSamples;
      oldest = i;
    }
  }
  return oldest;
}

void triggerVoice(VoiceType type, uint8_t velocity) {
  int idx = allocVoice();
  Voice &v = voices[idx];

  float vel = (float)velocity / 127.0f;
  v.active = true;
  v.type = type;
  v.phase = 0.0f;
  v.phase2 = 0.0f;
  v.env = 1.0f;
  v.ageSamples = 0;
  v.gain = 0.22f + vel * 0.78f;

  switch (type) {
    case VOICE_KICK:
      v.freq = 130.0f + vel * 40.0f;
      v.freq2 = 40.0f;
      v.decay = 0.99955f;
      break;

    case VOICE_SNARE:
      v.freq = 190.0f;
      v.freq2 = 320.0f;
      v.decay = 0.99920f;
      break;

    case VOICE_HAT_C:
      v.freq = 7000.0f;
      v.freq2 = 9000.0f;
      v.decay = 0.99720f;
      break;

    case VOICE_HAT_O:
      v.freq = 6000.0f;
      v.freq2 = 8500.0f;
      v.decay = 0.99905f;
      break;

    case VOICE_TOM1:
      v.freq = 180.0f;
      v.freq2 = 110.0f;
      v.decay = 0.99945f;
      break;

    case VOICE_TOM2:
      v.freq = 120.0f;
      v.freq2 = 80.0f;
      v.decay = 0.99950f;
      break;

    case VOICE_CRASH:
      v.freq = 4500.0f;
      v.freq2 = 7600.0f;
      v.decay = 0.99955f;
      break;

    case VOICE_RIDE:
      v.freq = 3200.0f;
      v.freq2 = 6200.0f;
      v.decay = 0.99970f;
      break;

    default:
      v.active = false;
      break;
  }
}

void handleHit(const PendingHit &h) {
  switch (h.zone) {
    case ZONE_SNARE:
      triggerVoice(VOICE_SNARE, h.velocity);
      break;
    case ZONE_HHC:
      triggerVoice(VOICE_HAT_C, h.velocity);
      break;
    case ZONE_HHO:
      triggerVoice(VOICE_HAT_O, h.velocity);
      break;
    case ZONE_TOM1:
      triggerVoice(VOICE_TOM1, h.velocity);
      break;
    case ZONE_TOM2:
      triggerVoice(VOICE_TOM2, h.velocity);
      break;
    case ZONE_CRASH:
      triggerVoice(VOICE_CRASH, h.velocity);
      break;
    case ZONE_RIDE:
      triggerVoice(VOICE_RIDE, h.velocity);
      break;
    case ZONE_FX:
      triggerVoice(VOICE_KICK, h.velocity);
      break;
    default:
      triggerVoice(VOICE_SNARE, h.velocity);
      break;
  }
}

static inline float fastSin(float x) {
  return sinf(x);
}

float renderVoice(Voice &v) {
  if (!v.active) return 0.0f;

  float out = 0.0f;
  const float twoPi = 6.28318530718f;

  switch (v.type) {
    case VOICE_KICK: {
      float pitchEnv = expf(-0.00008f * (float)v.ageSamples);
      float f = v.freq2 + (v.freq - v.freq2) * pitchEnv;
      v.phase += twoPi * f / SAMPLE_RATE;
      out = fastSin(v.phase) * v.env * v.gain * 1.4f;
      break;
    }

    case VOICE_SNARE: {
      v.phase += twoPi * v.freq / SAMPLE_RATE;
      float tone = fastSin(v.phase) * 0.35f;
      float noise = frandSigned() * 0.95f;
      out = (tone + noise) * v.env * v.gain * 0.95f;
      break;
    }

    case VOICE_HAT_C: {
      float noise = frandSigned();
      float hp = noise - 0.85f * sinf(v.phase);
      v.phase += twoPi * v.freq / SAMPLE_RATE;
      out = hp * v.env * v.gain * 0.55f;
      break;
    }

    case VOICE_HAT_O: {
      float noise = frandSigned();
      float hp = noise - 0.75f * sinf(v.phase);
      v.phase += twoPi * v.freq / SAMPLE_RATE;
      out = hp * v.env * v.gain * 0.48f;
      break;
    }

    case VOICE_TOM1:
    case VOICE_TOM2: {
      float pitchEnv = expf(-0.00006f * (float)v.ageSamples);
      float f = v.freq2 + (v.freq - v.freq2) * pitchEnv;
      v.phase += twoPi * f / SAMPLE_RATE;
      out = fastSin(v.phase) * v.env * v.gain * 1.1f;
      break;
    }

    case VOICE_CRASH: {
      float n = frandSigned();
      v.phase += twoPi * v.freq / SAMPLE_RATE;
      v.phase2 += twoPi * v.freq2 / SAMPLE_RATE;
      float metal = (sinf(v.phase) + sinf(v.phase2) + n * 1.6f) * 0.33f;
      out = metal * v.env * v.gain * 0.85f;
      break;
    }

    case VOICE_RIDE: {
      float n = frandSigned() * 0.4f;
      v.phase += twoPi * v.freq / SAMPLE_RATE;
      v.phase2 += twoPi * v.freq2 / SAMPLE_RATE;
      float metal = (sinf(v.phase) * 0.55f + sinf(v.phase2) * 0.45f + n);
      out = metal * v.env * v.gain * 0.65f;
      break;
    }

    default:
      out = 0.0f;
      break;
  }

  v.env *= v.decay;
  v.ageSamples++;

  if (v.env < 0.0005f || fabsf(out) < 0.00001f) {
    v.active = false;
  }

  return out;
}

void audioTask(void *parameter) {
  static int16_t outBuf[AUDIO_BLOCK_SAMPLES * 2]; // estéreo intercalado

  while (true) {
    PendingHit h;
    while (dequeueHit(h)) {
      handleHit(h);
    }

    for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
      float mix = 0.0f;

      for (int v = 0; v < MAX_VOICES; v++) {
        if (voices[v].active) {
          mix += renderVoice(voices[v]);
        }
      }

      // limiter simples
      if (mix > 1.0f) mix = 1.0f;
      if (mix < -1.0f) mix = -1.0f;

      int16_t s = (int16_t)(mix * 30000.0f);

      outBuf[i * 2 + 0] = s; // L
      outBuf[i * 2 + 1] = s; // R
    }

    size_t written = 0;
    i2s_write(I2S_PORT, outBuf, sizeof(outBuf), &written, portMAX_DELAY);
  }
}

// =========================
// ESPNOW RX
// =========================
void onDataRecv(const esp_now_recv_info_t *recvInfo, const uint8_t *incomingData, int len) {
  if (len != sizeof(HitPacket)) {
    return;
  }

  HitPacket pkt;
  memcpy(&pkt, incomingData, sizeof(pkt));
  enqueueHit(pkt.zone, pkt.velocity, pkt.stickId);

  Serial.printf("RX stick=%u zone=%u vel=%u peak=%umG dt=%ums from %02X:%02X:%02X:%02X:%02X:%02X\n",
                pkt.stickId,
                pkt.zone,
                pkt.velocity,
                pkt.peakMilliG,
                pkt.dtMs,
                recvInfo->src_addr[0], recvInfo->src_addr[1], recvInfo->src_addr[2],
                recvInfo->src_addr[3], recvInfo->src_addr[4], recvInfo->src_addr[5]);
}

bool initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    return false;
  }

  esp_now_register_recv_cb(onDataRecv);
  return true;
}

void printLocalMac() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.printf("Central MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// =========================
// SETUP / LOOP
// =========================
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("=== AIR DRUM CENTRAL ESP32 + PCM5102 ===");

  printLocalMac();

  if (!initEspNow()) {
    Serial.println("ERRO: falha no ESP-NOW");
    while (true) delay(1000);
  }

  initI2S();

  for (int i = 0; i < MAX_VOICES; i++) {
    voices[i].active = false;
  }

  xTaskCreatePinnedToCore(
    audioTask,
    "audioTask",
    4096,
    nullptr,
    2,
    nullptr,
    1
  );

  // som de boot
  enqueueHit(ZONE_SNARE, 90, 0);
  enqueueHit(ZONE_HHC, 60, 0);

  Serial.println("Central pronta.");
}

void loop() {
  // Aqui depois você pluga:
  // - ws2812b
  // - sequencer 32x8
  // - encoders
  // - MIDI OUT
  delay(10);
}
