# Air Drum com ESP-07S, MPU6050, ESP32 e PCM5102

Projeto base para uma bateria eletrônica **air drum** com:

- **2 baquetas** usando **ESP-07S + MPU6050**
- **1 central** com **ESP32**
- comunicação por **ESP-NOW**
- geração de áudio direto no **PCM5102** via **I2S**

## Visão geral

Cada baqueta lê o MPU6050, detecta um golpe e envia um pacote curto para a central.  
A central recebe os eventos, converte em instrumentos de bateria e gera o som localmente.

Fluxo:

```text
MPU6050 -> ESP-07S -> ESP-NOW -> ESP32 -> I2S -> PCM5102 -> saída analógica
```

## Estrutura

### Baquetas
Cada baqueta faz:

- leitura do MPU6050 por I2C
- cálculo do pico de aceleração
- classificação simples da direção do golpe
- envio de pacote por ESP-NOW

### Central
A central faz:

- recepção dos pacotes via ESP-NOW
- enfileiramento dos eventos
- síntese simples de bateria
- saída de áudio estéreo por I2S para o PCM5102

## Arquivos

Sugestão de organização:

```text
/airdrum
  /stick
    airdrum_stick_esp8266.ino
  /central
    airdrum_central_esp32_pcm5102.ino
  README.md
```

## Hardware

### Baqueta x2
- 1x ESP-07S ou placa com ESP8266 equivalente
- 1x MPU6050
- alimentação 3.3 V
- bateria ou fonte local
- fios curtos no I2C

### Central
- 1x ESP32
- 1x módulo PCM5102
- alimentação estável
- saída de linha para caixa amplificada, mesa ou fone com amplificador

## Ligações

### MPU6050 -> ESP-07S

Os pinos podem variar conforme sua placa/adaptador. No código atual:

- `GPIO4` -> SDA
- `GPIO5` -> SCL

Além disso:
- `VCC` -> 3V3
- `GND` -> GND

Endereço padrão do MPU6050:
- `0x68`

### ESP32 -> PCM5102

No código atual:

- `GPIO26` -> `BCK`
- `GPIO25` -> `LRCK` / `LCK` / `WS`
- `GPIO22` -> `DIN`
- `3V3` -> `VCC`
- `GND` -> `GND`

Saída analógica:
- `LOUT`
- `ROUT`
- `GND`

## Dependências

### Baquetas
- core Arduino para ESP8266
- `Wire.h`
- `ESP8266WiFi.h`
- `espnow.h`

### Central
- core Arduino para ESP32
- `WiFi.h`
- `esp_now.h`
- `driver/i2s.h`

## Como gravar

### 1. Grave a central primeiro
Suba o arquivo da central no ESP32 e abra o monitor serial.

A central imprime o MAC local, algo assim:

```text
Central MAC: 24:6F:28:AA:BB:CC
```

Copie esse MAC.

### 2. Ajuste as baquetas
No arquivo de cada baqueta, altere:

```cpp
uint8_t PEER_MAC[6] = {0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC};
```

Também ajuste o ID:

- baqueta esquerda: `STICK_ID 1`
- baqueta direita: `STICK_ID 2`

### 3. Grave as duas baquetas
Suba o sketch em cada ESP-07S.

## Configuração importante

### Canal do ESP-NOW

Nos códigos, o canal está fixo em:

```cpp
#define ESPNOW_CHANNEL 1
```

Mantenha o mesmo canal na central e nas duas baquetas.

## Formato do pacote

Estrutura usada entre baquetas e central:

```cpp
struct HitPacket {
  uint8_t  stickId;
  uint8_t  zone;
  uint8_t  velocity;
  uint8_t  flags;
  uint16_t peakMilliG;
  uint16_t dtMs;
};
```

### Campos
- `stickId`: identifica a baqueta
- `zone`: peça detectada
- `velocity`: intensidade do golpe
- `flags`: reservado
- `peakMilliG`: pico detectado
- `dtMs`: tempo desde o último golpe

## Mapeamento de zonas

No código atual:

- `0` = snare
- `1` = hi-hat fechado
- `2` = hi-hat aberto
- `3` = tom 1
- `4` = tom 2
- `5` = crash
- `6` = ride
- `7` = fx

## Ajustes de sensibilidade

No código das baquetas, os parâmetros principais são:

```cpp
#define SAMPLE_PERIOD_MS 4
#define HIT_THRESHOLD_G  1.85f
#define MIN_HIT_GAP_MS   70
#define HP_ALPHA         0.92f
```

### O que cada um faz
- `SAMPLE_PERIOD_MS`: taxa de leitura do MPU6050
- `HIT_THRESHOLD_G`: sensibilidade do golpe
- `MIN_HIT_GAP_MS`: tempo mínimo entre hits
- `HP_ALPHA`: filtro high-pass simples

## Calibração

A função principal de classificação é:

```cpp
uint8_t classifyZone(float axg, float ayg, float azg)
```

Ela ainda é simples e deve ser ajustada conforme:

- posição física do sensor
- orientação da baqueta
- empunhadura
- tipo de movimento

O ideal é testar no serial e ir refinando.

## Como o som é gerado

A central usa síntese simples, sem samples reais, para validar o sistema.

Instrumentos já previstos:
- kick
- snare
- hi-hat fechado
- hi-hat aberto
- tom 1
- tom 2
- crash
- ride

Isso já serve para:
- validar latência
- testar detecção
- ajustar sensibilidade
- provar a arquitetura completa

## Limitações desta versão

Esta primeira versão ainda não inclui:

- matriz WS2812B 32x8
- sequenciador
- MIDI OUT
- pedal de hi-hat
- choke de crash
- samples reais de bateria
- calibração automática
- filtragem mais avançada

## Próximos passos sugeridos

1. validar comunicação ESP-NOW
2. validar áudio no PCM5102
3. calibrar a detecção das zonas
4. adicionar WS2812B
5. adicionar step sequencer
6. adicionar MIDI OUT
7. trocar síntese por samples reais

## Dicas de depuração

### Se a central não receber nada
Verifique:
- MAC correto na baqueta
- mesmo canal ESP-NOW
- alimentação estável
- distância entre módulos
- boot correto do ESP-07S

### Se o áudio sair distorcido
Verifique:
- alimentação do PCM5102
- GND comum
- saída em nível de linha
- entrada do amplificador ou caixa
- sample rate do I2S

### Se houver hits falsos
Ajuste:
- `HIT_THRESHOLD_G`
- `MIN_HIT_GAP_MS`
- lógica da `classifyZone()`

## Licença

Uso livre para estudo, testes e evolução do projeto.
