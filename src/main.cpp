#include <Arduino.h>
#include <HardwareSerial.h>

// Pin definitions
#define LORA_B_TX_PIN 18
#define LORA_B_RX_PIN 19

#define SENSOR_FIELD_COUNT 16


// UART instance for LoRaB
HardwareSerial LoRaB(1);

#define SENSOR_FIELD_COUNT 16

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  LoRaB.begin(115200, SERIAL_8N1, LORA_B_RX_PIN, LORA_B_TX_PIN);
  Serial.println("Initializing LoRaB module...");

  LoRaB.print("AT\r\n");
  delay(300);
  LoRaB.print("AT+ADDRESS=1\r\n");
  delay(300);
  LoRaB.print("AT+NETWORKID=18\r\n");
  delay(300);
  LoRaB.print("AT+PARAMETER=9,7,1,12\r\n");
  delay(300);
  LoRaB.print("AT+BAND=915000000\r\n");
  delay(300);

  Serial.println("LoRaB initialization complete");
}

void loop() {
  if (LoRaB.available()) {
    String resp = LoRaB.readStringUntil('\n');
    resp.trim();

    if (resp.startsWith("+RCV=")) {
      String payload = resp.substring(5);  // Remove "+RCV="

      // Convert to C string
      char raw[payload.length() + 1];
      payload.toCharArray(raw, sizeof(raw));

      char *fields[32];
      int count = 0;

      char *token = strtok(raw, ",");
      while (token != NULL && count < 32) {
        fields[count++] = token;
        token = strtok(NULL, ",");
      }

      if (count >= 19) {
        // time 대신 왜곡된 필드를 받는 경우가 있으니 field[2]를 time으로 간주
        unsigned long time = strtoul(fields[2], NULL, 10);
        float data[16];

        for (int i = 0; i < 16; i++) {
          data[i] = atof(fields[i + 3]); // Skip addr, len, time
        }

        Serial.print(time);
        if (count >= 19) {
            // 센서값 16개 (fields[3]~fields[18]까지)
            for (int i = 3; i <= 17; i++) {
                Serial.print(",");
                Serial.print(fields[i]);
            }
            Serial.println();
        }

        Serial.println();
      } else {
        Serial.println("Invalid field count");
      }
    }
  }
}