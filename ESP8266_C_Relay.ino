#include <ESP8266WiFi.h>
#include <espnow.h>

#define SEP()  Serial.println(F("------------------------------------"))
#define SEP2() Serial.println(F("===================================="))

const uint8_t ESPNOW_TOKEN[4] = { 0x3C, 0x7A, 0x11, 0xD5 };
#define ESPNOW_PUMP_ON  0x61
#define ESPNOW_PUMP_OFF 0x62
#define ESPNOW_AUTO_OFF 0xF1   

typedef struct {
  uint8_t cmd;
  uint8_t token[4];
} EspNowMsg;

#define RELAY_PIN        5      
#define RELAY_ACTIVE_LOW true

bool    relayState  = false;
uint8_t senderMac[6] = {0};          
unsigned long relayOnTime = 0;        

void relayOn() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
  relayState  = true;
  relayOnTime = millis();   // start 20-min timer
  SEP();
  Serial.println(F("[Relay] *** RELAY ON  — PUMP RUNNING ***"));
  Serial.println(F("[Relay] Auto-OFF in 20 minutes if not stopped manually"));
  SEP();
}

void relayOff() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  relayState  = false;
  relayOnTime = 0;
  SEP();
  Serial.println(F("[Relay] *** RELAY OFF — PUMP STOPPED ***"));
  SEP();
}

void sendAutoOffFeedback() {
  // Only send if we have a valid sender MAC
  if (senderMac[0] == 0 && senderMac[1] == 0) return;
  EspNowMsg fb;
  fb.cmd = ESPNOW_AUTO_OFF;
  memcpy(fb.token, ESPNOW_TOKEN, 4);
  esp_now_send(senderMac, (uint8_t*)&fb, sizeof(fb));
  Serial.println(F("[AUTO] Feedback 0xF1 sent to B"));
}

void onDataReceived(uint8_t* mac, uint8_t* data, uint8_t len) {
  SEP();
  Serial.printf("[ESP-NOW] <-- Packet  len=%d\n", len);
  Serial.printf("[ESP-NOW] <-- From: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (len != sizeof(EspNowMsg)) {
    Serial.printf("[ESP-NOW] !! Wrong length — rejected\n");
    SEP(); return;
  }

  EspNowMsg* msg = (EspNowMsg*)data;
  if (memcmp(msg->token, ESPNOW_TOKEN, 4) != 0) {
    Serial.println(F("[ESP-NOW] !! Token MISMATCH — rejected"));
    SEP(); return;
  }

  // Remember B's MAC for sending feedback
  memcpy(senderMac, mac, 6);

  // Register B as peer if not already (needed to send back)
  esp_now_add_peer(senderMac, ESP_NOW_ROLE_COMBO, 1, nullptr, 0);

  Serial.printf("[ESP-NOW] Token OK  CMD=0x%02X\n", msg->cmd);

  switch (msg->cmd) {
    case ESPNOW_PUMP_ON:
      Serial.println(F("[ESP-NOW] Command: PUMP ON"));
      relayOn();
      break;
    case ESPNOW_PUMP_OFF:
      Serial.println(F("[ESP-NOW] Command: PUMP OFF"));
      relayOff();
      break;
    default:
      Serial.printf("[ESP-NOW] !! Unknown CMD=0x%02X — ignored\n", msg->cmd);
      SEP();
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);
  SEP2();
  Serial.println(F("  ESP8266-C  RELAY CONTROLLER  BOOTING"));
  SEP2();

  pinMode(RELAY_PIN, OUTPUT);
  relayOff();
  Serial.println(F("[BOOT] Relay pin initialised → OFF"));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.print(F("[WiFi] MAC: "));
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != 0) {
    Serial.println(F("[ERROR] ESP-NOW init FAILED!"));
    while (true) { delay(1000); Serial.println(F("[ERROR] stuck")); }
  }
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);   // COMBO so it can send back
  esp_now_register_recv_cb(onDataReceived);

  SEP2();
  Serial.println(F("  BOOT COMPLETE — waiting for commands from B"));
  Serial.printf ("  Relay: D1 (GPIO%d)  Active: %s\n",
                 RELAY_PIN, RELAY_ACTIVE_LOW ? "LOW" : "HIGH");
  SEP2();
}

void loop() {
  // ── Auto-OFF relay after 20 minutes ───────────────
  if (relayState && relayOnTime > 0 &&
      millis() - relayOnTime >= 1200000UL) {
    Serial.println(F("[AUTO] 20 min elapsed — turning relay OFF"));
    relayOff();
    sendAutoOffFeedback();   // notify B → A → Firebase
  }

  // ── Heartbeat ──────────────────────────────────────
  static unsigned long lastHB = 0;
  if (millis() - lastHB >= 10000) {
    lastHB = millis();
    Serial.printf("[Relay] Heartbeat — relay is %s\n",
                  relayState ? "ON" : "OFF");
  }
  delay(10);
}