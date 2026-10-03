#include <ESP8266WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define SEP()  Serial.println(F("------------------------------------"))
#define SEP2() Serial.println(F("===================================="))

const uint8_t SEC_KEY[16] = {
  0x2A, 0x7F, 0x3C, 0x91, 0xB4, 0x05, 0xE8, 0x6D,
  0x1A, 0xC3, 0x77, 0x5E, 0x09, 0xF2, 0x4B, 0x88
};

#define HOME_SSID     "A06"
#define HOME_PASSWORD "12345678"
#define API_KEY       "AIzaSyAEo0sJ49Xk63LVWRcz3eb6JmBMQbM-QuE"
#define DATABASE_URL  "https://agrinis-c68e9-default-rtdb.firebaseio.com/"

#define FB_CMD_PATH      "/garden/command"
#define FB_MOISTURE_PATH "/garden/moisture"
#define FB_PUMP_PATH     "/garden/pump"
#define FB_PUMP_STATUS   "/garden/pump_status"
#define FB_STATUS_PATH   "/garden/status"

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

#define ACK_TIMEOUT_MS 5000
#define MAX_RETRIES    4

FirebaseData   fbdo, fbdo_write;
FirebaseAuth   fbAuth;
FirebaseConfig fbConfig;

bool     fbReady      = false;
float    lastMoisture = -1.0;
bool     pumpState    = false;
bool     monitoring   = false;
uint32_t txCounter    = 0;
uint32_t lastRxID     = 0;
int      fbFailCount  = 0;

unsigned long lastFbPoll  = 0;
unsigned long lastFbRetry = 0;
unsigned long monitorStart = 0;   // 15-min auto-stop timer

#define FB_POLL_MS   3000
#define FB_RETRY_MS  30000
#define FB_MAX_FAILS 5

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

void xorEncrypt(uint8_t* data, size_t len, const uint8_t* salt, size_t saltLen) {
  for (size_t i = 0; i < len; i++)
    data[i] ^= SEC_KEY[i % 16] ^ salt[i % saltLen];
}
void xorDecrypt(uint8_t* data, size_t len, const uint8_t* salt, size_t saltLen) {
  xorEncrypt(data, len, salt, saltLen);
}

bool sendLoRa(uint8_t cmd) {
  uint8_t plain[5];
  plain[0] = cmd;
  plain[1] = (txCounter >> 24) & 0xFF;
  plain[2] = (txCounter >> 16) & 0xFF;
  plain[3] = (txCounter >>  8) & 0xFF;
  plain[4] = (txCounter      ) & 0xFF;
  txCounter++;

  uint32_t crc = crc32(plain, 5);
  uint8_t salt[3];
  salt[0] = random(0, 256);
  salt[1] = random(0, 256);
  salt[2] = random(0, 256);

  uint8_t enc[5];
  memcpy(enc, plain, 5);
  xorEncrypt(enc, 5, salt, 3);

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
  Serial.printf("[LoRa] --> CMD=0x%02X TxID=%u %s\n", cmd, txCounter - 1, ok ? "OK" : "FAILED");
  return ok;
}

bool receiveLoRa(uint8_t* cmdOut, float* moistOut, uint32_t* rxIDOut) {
  int sz = LoRa.parsePacket();
  if (sz == 0) return false;

  if (sz != PACKET_SIZE) {
    Serial.printf("[LoRa] <-- Wrong size %d (exp %d) — rejected\n", sz, PACKET_SIZE);
    while (LoRa.available()) LoRa.read();
    return false;
  }

  uint8_t pkt[PACKET_SIZE];
  for (int i = 0; i < PACKET_SIZE; i++) pkt[i] = LoRa.read();

  uint8_t salt[3] = { pkt[9], pkt[10], pkt[11] };
  uint8_t plain[5];
  memcpy(plain, pkt, 5);
  xorDecrypt(plain, 5, salt, 3);

  uint32_t rxCRC  = ((uint32_t)pkt[5] << 24) | ((uint32_t)pkt[6] << 16)
                  | ((uint32_t)pkt[7] <<  8) | ((uint32_t)pkt[8]);
  uint32_t calcCRC = crc32(plain, 5);
  if (rxCRC != calcCRC) {
    Serial.println(F("[LoRa] <-- CRC mismatch — rejected"));
    return false;
  }

  *cmdOut  = plain[0];
  *rxIDOut = ((uint32_t)plain[1] << 24) | ((uint32_t)plain[2] << 16)
           | ((uint32_t)plain[3] <<  8) | ((uint32_t)plain[4]);

  if (*cmdOut == RESP_MOISTURE) {
    uint16_t raw = ((uint16_t)plain[3] << 8) | plain[4];
    *moistOut = raw / 10.0f;
  }

  Serial.printf("[LoRa] <-- CMD=0x%02X  RxID=%u  RSSI=%d\n",
                *cmdOut, *rxIDOut, LoRa.packetRssi());
  return true;
}

uint8_t waitForAck() {
  Serial.printf("[LoRa] Waiting for ACK (%d ms)...\n", ACK_TIMEOUT_MS);
  unsigned long start = millis();
  while (millis() - start < ACK_TIMEOUT_MS) {
    uint8_t cmd; float moisture; uint32_t rxID;
    if (receiveLoRa(&cmd, &moisture, &rxID)) {
      if (cmd == ACK_PUMP_ON || cmd == ACK_PUMP_OFF || cmd == ACK_FAIL) {
        Serial.printf("[LoRa] ACK received: 0x%02X\n", cmd);
        return cmd;
      }
      if (cmd == RESP_MOISTURE) {
        lastMoisture = moisture;
        Serial.printf("[LoRa] Moisture during wait: %.1f%%\n", moisture);
      }
    }
    delay(20);
  }
  Serial.println(F("[LoRa] !! ACK TIMEOUT"));
  return 0x00;
}

bool sendPumpConfirmed(uint8_t cmd) {
  bool wantOn = (cmd == CMD_PUMP_ON);
  uint8_t expectedAck = wantOn ? ACK_PUMP_ON : ACK_PUMP_OFF;

  SEP();
  Serial.printf("[CMD]  Pump %s — with confirmation\n", wantOn ? "ON" : "OFF");
  Firebase.RTDB.setString(&fbdo_write, FB_PUMP_STATUS, "PENDING");

  for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
    Serial.printf("[CMD]  Attempt %d/%d\n", attempt, MAX_RETRIES);
    if (!sendLoRa(cmd)) { delay(500); continue; }

    uint8_t ack = waitForAck();

    if (ack == expectedAck) {
      pumpState = wantOn;
      Serial.printf("[CMD]  ✓ Relay confirmed %s\n", wantOn ? "ON" : "OFF");
      Firebase.RTDB.setBool  (&fbdo_write, FB_PUMP_PATH,   pumpState);
      Firebase.RTDB.setString(&fbdo_write, FB_PUMP_STATUS, wantOn ? "ON" : "OFF");
      SEP(); return true;
    } else if (ack == ACK_FAIL) {
      Serial.println(F("[CMD]  B reports ESP-NOW to C FAILED — retrying"));
      delay(1000);
    } else {
      Serial.println(F("[CMD]  No ACK — retrying"));
      delay(500);
    }
  }

  Serial.println(F("[CMD]  !! All retries failed"));
  Firebase.RTDB.setBool  (&fbdo_write, FB_PUMP_PATH,   false);
  Firebase.RTDB.setString(&fbdo_write, FB_PUMP_STATUS, "ERROR");
  SEP(); return false;
}

void executeCommand(uint8_t cmd) {
  switch (cmd) {
    case CMD_PUMP_ON:
    case CMD_PUMP_OFF:
      sendPumpConfirmed(cmd);
      break;

    case CMD_CHECK:
      SEP();
      Serial.println(F("[CMD]  CHECK MOISTURE"));
      if (sendLoRa(CMD_CHECK)) {
        monitoring   = true;
        monitorStart = 0;   // reset so timer restarts fresh
        Serial.println(F("[CMD]  Monitoring STARTED — auto-stop in 15 min"));
      }
      SEP();
      break;
  }
}

void pollFirebase() {
  if (fbFailCount >= FB_MAX_FAILS) {
    if (millis() - lastFbRetry >= FB_RETRY_MS) {
      Serial.println(F("[FB]   Resuming after pause..."));
      fbFailCount = 0; lastFbRetry = millis();
    } else return;
  }
  if (!Firebase.ready()) { fbFailCount++; return; }

  if (!Firebase.RTDB.getString(&fbdo, FB_CMD_PATH)) {
    Serial.println("[FB]   Read FAILED: " + fbdo.errorReason());
    fbFailCount++; return;
  }
  fbFailCount = 0;

  String cmd = fbdo.stringData();
  cmd.trim();
  if (cmd == "NONE" || cmd == "") return;

  Serial.println("[FB]   Command: " + cmd);

  uint8_t loraCmd = 0;
  if      (cmd == "CHECK")    loraCmd = CMD_CHECK;
  else if (cmd == "PUMP_ON")  loraCmd = CMD_PUMP_ON;
  else if (cmd == "PUMP_OFF") loraCmd = CMD_PUMP_OFF;
  else { Firebase.RTDB.setString(&fbdo_write, FB_CMD_PATH, "NONE"); return; }

  Firebase.RTDB.setString(&fbdo_write, FB_CMD_PATH, "NONE");
  executeCommand(loraCmd);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  SEP2();
  Serial.println(F("  ESP8266-A  GARDEN GATEWAY  BOOTING"));
  SEP2();

  WiFi.mode(WIFI_STA);
  WiFi.begin(HOME_SSID, HOME_PASSWORD);
  Serial.print(F("[WiFi] Connecting"));
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println();
  Serial.print(F("[WiFi] Connected! IP: ")); Serial.println(WiFi.localIP());
  Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());

  fbConfig.api_key      = API_KEY;
  fbConfig.database_url = DATABASE_URL;
  fbConfig.token_status_callback = tokenStatusCallback;
  Firebase.signUp(&fbConfig, &fbAuth, "", "");
  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectWiFi(true);
  fbReady = true;
  Firebase.RTDB.setString(&fbdo_write, FB_STATUS_PATH,   "ONLINE");
  Firebase.RTDB.setString(&fbdo_write, FB_CMD_PATH,      "NONE");
  Firebase.RTDB.setFloat (&fbdo_write, FB_MOISTURE_PATH, -1);
  Firebase.RTDB.setBool  (&fbdo_write, FB_PUMP_PATH,     false);
  Firebase.RTDB.setString(&fbdo_write, FB_PUMP_STATUS,   "OFF");
  Serial.println(F("[FB]   Firebase ready ✓"));

  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("[ERROR] LoRa FAILED!"));
    while (true) { delay(2000); Serial.println(F("[ERROR] stuck")); }
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setCodingRate4(LORA_CR);
  LoRa.setSyncWord(LORA_SYNC);
  LoRa.enableCrc();
  Serial.printf("[LoRa] Ready — %.0fMHz SF%d BW%.0fk Sync=0x%02X\n",
                LORA_FREQ/1e6, LORA_SF, LORA_BW/1e3, LORA_SYNC);

  randomSeed(ESP.getCycleCount());
  SEP2();
  Serial.println(F("  BOOT COMPLETE — polling Firebase every 3s"));
  SEP2();
}

void loop() {
  // ── Auto-stop monitoring after 15 minutes ─────────
  if (monitoring && monitorStart == 0) monitorStart = millis();
  if (monitoring && millis() - monitorStart >= 900000UL) {
    monitoring   = false;
    monitorStart = 0;
    lastMoisture = -1.0;
    Firebase.RTDB.setFloat(&fbdo_write, FB_MOISTURE_PATH, -1);
    Serial.println(F("[AUTO] Monitoring auto-stopped after 15 min"));
  }
  if (!monitoring) monitorStart = 0;

  // ── Incoming LoRa from B ───────────────────────────
  uint8_t cmd; float moisture; uint32_t rxID;
  if (receiveLoRa(&cmd, &moisture, &rxID)) {
    if (cmd == RESP_MOISTURE) {
      lastMoisture = moisture;
      Serial.printf("[LoRa] <-- Moisture: %.1f%%\n", moisture);
      Firebase.RTDB.setFloat(&fbdo_write, FB_MOISTURE_PATH, moisture);
    } else if (cmd == ACK_PUMP_OFF) {
      pumpState = false;
      Serial.println(F("[AUTO] Pump auto-OFF feedback received — updating Firebase"));
      Firebase.RTDB.setBool  (&fbdo_write, FB_PUMP_PATH,   false);
      Firebase.RTDB.setString(&fbdo_write, FB_PUMP_STATUS, "AUTO_OFF");
    }
  }

  // ── Firebase poll ──────────────────────────────────
  if (millis() - lastFbPoll >= FB_POLL_MS) {
    lastFbPoll = millis();
    pollFirebase();
  }
}