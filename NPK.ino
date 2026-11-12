#include "BluetoothSerial.h"
#include <ModbusMaster.h>

BluetoothSerial SerialBT;
ModbusMaster node;

#define RXD2 16
#define TXD2 17
#define DE_RE 4

void preTransmission() {
  digitalWrite(DE_RE, HIGH);
}

void postTransmission() {
  digitalWrite(DE_RE, LOW);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("BK-NPK");

  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  node.begin(1, Serial2);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  Serial.println("NPK Bluetooth Started");
}

void loop() {
  uint8_t result = node.readHoldingRegisters(0x001E, 3);

  if (result == node.ku8MBSuccess) {
    uint16_t N = node.getResponseBuffer(0);
    uint16_t P = node.getResponseBuffer(1);
    uint16_t K = node.getResponseBuffer(2);

    String data = "Nitrogen (N): " + String(N) + "\n" +
                  "Phosphorus (P): " + String(P) + "\n" +
                  "Potassium (K): " + String(K);

    Serial.println(data);

    // Only send if app requested
    if (SerialBT.available()) {
      String command = SerialBT.readStringUntil('\n');
      command.trim();
      if (command == "GET") {
        SerialBT.print(data);
        SerialBT.print("#");
      }
    }

  } else {
    Serial.println("NPK Sensor Error");

    
    if (SerialBT.available()) {
      String command = SerialBT.readStringUntil('\n');
      command.trim();
      if (command == "GET") {
        SerialBT.print("SENSOR ERROR#");  
      }
    }
  }

  delay(2000);
}