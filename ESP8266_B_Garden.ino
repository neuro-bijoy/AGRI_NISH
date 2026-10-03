#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SPI.h>
#include <LoRa.h>

#define SEP()  Serial.println(F("------------------------------------"))
#define SEP2() Serial.println(F("===================================="))

const uint8_t SEC_KEY[16] = {
  0x2A, 0x7F, 0x3C, 0x91, 0xB4, 0x05, 0xE8, 0x6D,
  0x1A, 0xC3, 0x77, 0x5E, 0x09, 0xF2, 0x4B, 0x88
};

uint8_t ESP_C_MAC[6] = { 0xD8, 0xBF, 0xC0, 0xF9, 0x23, 0x18 };

typedef struct { uint8_t cmd; uint8_t token[4]; } EspNowMsg;
const uint8_t ESPNOW_TOKEN[4] = { 0x3C, 0x7A, 0x11, 0xD5 };
#define ESPNOW_PUMP_ON  0x61
#define ESPNOW_PUMP_OFF 0x62
volatile bool espNowSendDone = false, espNowSendResult = false;
#define ESPNOW_TIMEOUT_MS 3000

#define LORA_NSS    15
#define LORA_RST    16
#define LORA_DIO0    4
#define LORA_FREQ   433E6
#define LORA_SF     10
#define LORA_BW     125E3
#define LORA_CR     5
#define LORA_SYNC   0xB4
#define PACKET_SIZE 12

#define CMD_CHECK     0x05
#define CMD_PUMP_ON   0x06
#define CMD_PUMP_OFF  0x07
#define RESP_MOISTURE 0xAA
#define ACK_PUMP_ON   0xB1
#define ACK_PUMP_OFF  0xB2
#define ACK_FAIL      0xBF

#define SENSOR_PIN  A0
#define DRY_VALUE   1024
#define WET_VALUE   350

bool     monitoring  = false;
uint32_t lastSend    = 0;
uint32_t monitorMs   = 2000;
uint32_t lastRxID    = 0;
uint32_t txCounter   = 0;
unsigned long monitorStart = 0;   // 15-min auto-stop timer

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

void xorCrypt(uint8_t* data, size_t len, const uint8_t* salt) {
  for (size_t i = 0; i < len; i++)
    data[i] ^= SEC_KEY[i % 16] ^ salt[i % 3];
}

bool sendLoRa(uint8_t cmd, float moisture = 0.0f) {
  uint8_t plain[5];
  plain[0] = cmd;
  if (cmd == RESP_MOISTURE) {
    uint16_t raw = (uint16_t)(moisture * 10.0f);
    plain[1] = 0; plain[2] = 0;
    plain[3] = (raw >> 8) & 0xFF;
    plain[4] =  raw       & 0xFF;
  } else {
    plain[1] = (txCounter >> 24) & 0xFF;
    plain[2] = (txCounter >> 16) & 0xFF;
    plain[3] = (txCounter >>  8) & 0xFF;
    plain[4] = (txCounter      ) & 0xFF;
  }
  txCounter++;

  uint32_t crc = crc32(plain, 5);
  uint8_t salt[3] = { (uint8_t)random(256), (uint8_t)random(256), (uint8_t)random(256) };
  uint8_t enc[5];
  memcpy(enc, plain, 5);
  xorCrypt(enc, 5, salt);

  uint8_t pkt[PACKET_SIZE];
  memcpy(pkt, enc, 5);
  pkt[5] = (crc >> 24) & 0xFF;
  pkt[6] = (crc >> 16) & 0xFF;
  pkt[7] = (crc >>  8) & 0xFF;
  pkt[8] =  crc        & 0xFF;
  pkt[9]  = salt[0];
  pkt[10] = salt[1];
  pkt[11] = salt[2];

  LoRa.beginPacket();
  LoRa.write(pkt, PACKET_SIZE);
  bool ok = LoRa.endPacket();
  Serial.printf("[LoRa] --> CMD=0x%02X %s\n", cmd, ok ? "OK" : "FAILED");
  return ok;
}

bool receiveLoRa(uint8_t* cmdOut, float* moistOut, uint32_t* rxIDOut) {
  int sz = LoRa.parsePacket();
  if (sz == 0) return false;
  if (sz != PACKET_SIZE) {
    Serial.printf("[LoRa] Wrong size %d (exp %d) — rejected\n", sz, PACKET_SIZE);
    while (LoRa.available()) LoRa.read();
    return false;
  }
  uint8_t pkt[PACKET_SIZE];
  for (int i = 0; i < PACKET_SIZE; i++) pkt[i] = LoRa.read();

  uint8_t salt[3] = { pkt[9], pkt[10], pkt[11] };
  uint8_t plain[5];
  memcpy(plain, pkt, 5);
  xorCrypt(plain, 5, salt);

  uint32_t rxCRC   = ((uint32_t)pkt[5]<<24)|((uint32_t)pkt[6]<<16)|((uint32_t)pkt[7]<<8)|pkt[8];
  uint32_t calcCRC = crc32(plain, 5);
  if (rxCRC != calcCRC) {
    Serial.println(F("[LoRa] CRC mismatch — rejected"));
    return false;
  }

  *cmdOut  = plain[0];
  *rxIDOut = ((uint32_t)plain[1]<<24)|((uint32_t)plain[2]<<16)|((uint32_t)plain[3]<<8)|plain[4];
  if (*cmdOut == RESP_MOISTURE) {
    uint16_t raw = ((uint16_t)plain[3] << 8) | plain[4];
    *moistOut = raw / 10.0f;
  }
  Serial.printf("[LoRa] <-- CMD=0x%02X RSSI=%d\n", *cmdOut, LoRa.packetRssi());
  return true;
}

float readMoisture() {
  int raw = analogRead(SENSOR_PIN);
  int pct = constrain(map(raw, DRY_VALUE, WET_VALUE, 0, 100), 0, 100);
  String lvl = pct < 30 ? "LOW" : pct < 70 ? "MEDIUM" : "HIGH";
  Serial.println("--------------------------------");
  Serial.print("Raw: "); Serial.println(raw);
  Serial.print("Moisture: "); Serial.print(pct); Serial.println("%");
  Serial.print("Status: "); Serial.println(lvl);
  Serial.println("--------------------------------");
  return (float)pct;
}

void espNowSendCb(uint8_t* mac, uint8_t status) {
  espNowSendDone   = true;
  espNowSendResult = (status == 0);
  Serial.printf("[ESP-NOW] %s\n", status == 0 ? "CONFIRMED ✓" : "FAILED ✗");
}

// ── Receive feedback from C (auto-OFF) and forward to A ──
void onEspNowReceive(uint8_t* mac, uint8_t* data, uint8_t len) {
  if (len != sizeof(EspNowMsg)) return;
  EspNowMsg* msg = (EspNowMsg*)data;
  if (memcmp(msg->token, ESPNOW_TOKEN, 4) != 0) return;
  if (msg->cmd == 0xF1) {
    Serial.println(F("[AUTO] C reports relay auto-OFF — sending ACK_PUMP_OFF to A"));
    sendLoRa(ACK_PUMP_OFF);
  }
}

bool sendEspNowConfirmed(uint8_t cmd) {
  EspNowMsg msg; msg.cmd = cmd;
  memcpy(msg.token, ESPNOW_TOKEN, 4);
  espNowSendDone = false; espNowSendResult = false;
  if (esp_now_send(ESP_C_MAC, (uint8_t*)&msg, sizeof(msg)) != 0) return false;
  unsigned long t0 = millis();
  while (!espNowSendDone && millis() - t0 < ESPNOW_TIMEOUT_MS) { delay(10); yield(); }
  return espNowSendDone && espNowSendResult;
}

void handlePumpCommand(uint8_t cmd) {
  bool wantOn = (cmd == CMD_PUMP_ON);
  Serial.printf("[CMD]  Pump %s → forwarding to C\n", wantOn ? "ON" : "OFF");
  bool ok = sendEspNowConfirmed(wantOn ? ESPNOW_PUMP_ON : ESPNOW_PUMP_OFF);
  delay(200);
  uint8_t ack = ok ? (wantOn ? ACK_PUMP_ON : ACK_PUMP_OFF) : ACK_FAIL;
  Serial.printf("[CMD]  ACK 0x%02X → A\n", ack);
  sendLoRa(ack);
}

void setup() {
  Serial.begin(115200); delay(800);
  SEP2(); Serial.println(F("  NODE B  GARDEN  BOOTING")); SEP2();

  WiFi.mode(WIFI_STA); WiFi.disconnect(); delay(100);
  Serial.print(F("[MAC] ")); Serial.println(WiFi.macAddress());

  if (esp_now_init() != 0) { Serial.println(F("ESP-NOW FAILED")); while(1); }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(espNowSendCb);
  esp_now_register_recv_cb(onEspNowReceive);   // ← receive auto-OFF feedback from C
  esp_now_add_peer(ESP_C_MAC, ESP_NOW_ROLE_COMBO, 1, nullptr, 0);
  Serial.println(F("[ESP-NOW] Ready"));

  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) { Serial.println(F("LoRa FAILED")); while(1); }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  Serial.println(F("[LoRa] Ready"));

  randomSeed(ESP.getCycleCount());
  SEP2(); Serial.println(F("  BOOT COMPLETE")); SEP2();
}

void loop() {
  // ── Auto-stop monitoring after 15 minutes ─────────
  if (monitoring && monitorStart == 0) monitorStart = millis();
  if (monitoring && millis() - monitorStart >= 900000UL) {
    monitoring   = false;
    monitorStart = 0;
    Serial.println(F("[AUTO] Monitoring auto-stopped after 15 min"));
  }
  if (!monitoring) monitorStart = 0;

  // ── Periodic moisture send ─────────────────────────
  if (monitoring && millis() - lastSend >= monitorMs) {
    sendLoRa(RESP_MOISTURE, readMoisture());
    lastSend = millis();
  }

  // ── LoRa receive ───────────────────────────────────
  uint8_t cmd; float moisture; uint32_t rxID;
  if (!receiveLoRa(&cmd, &moisture, &rxID)) return;

  switch (cmd) {
    case CMD_CHECK:
      Serial.println(F("[CMD] CHECK — monitoring ON"));
      monitoring   = true;
      monitorStart = millis();   // start 15-min timer
      sendLoRa(RESP_MOISTURE, readMoisture());
      lastSend = millis();
      break;
    case CMD_PUMP_ON:
    case CMD_PUMP_OFF:
      handlePumpCommand(cmd);
      break;
    default:
      Serial.printf("[CMD] Unknown 0x%02X\n", cmd);
  }
}