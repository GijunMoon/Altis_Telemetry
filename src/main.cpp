#include <Arduino.h>
#include <HardwareSerial.h>

// Pin definitions
#define LORA_A_TX_PIN 18
#define LORA_A_RX_PIN 19

// UART instance
HardwareSerial LoRaA(1);

unsigned long lastSend = 0;
unsigned long lastPing = 0;
const unsigned long SEND_INTERVAL = 100; // 100ms for sensor data
const unsigned long PING_INTERVAL = 1000; // 1s for ping
unsigned long pingSentTime = 0;
bool awaitingPong = false;

// Sensor data structure
typedef struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
} SensorData_t;

// Generate dummy sensor data
SensorData_t generateSensorData() {
  SensorData_t data;
  data.accel_x = 1.0;
  data.accel_y = 2.0;
  data.accel_z = 3.0;
  data.gyro_x = 4.0;
  data.gyro_y = 5.0;
  data.gyro_z = 6.0;
  return data;
}

// Send AT command with improved debugging
void sendAT(HardwareSerial &serial, const char *cmd) {
  serial.print(cmd);
  serial.print("\r\n");
  Serial.print("Sending to ");
  Serial.print(&serial == &LoRaA ? "LoRaA" : "LoRaB");
  Serial.print(": ");
  Serial.println(cmd);
  serial.flush();
  delay(500); // Increased delay for module response
  String response = "";
  unsigned long start = millis();
  while (millis() - start < 1000) { // Increased timeout to 1s
    if (serial.available()) {
      char c = serial.read();
      response += c;
      Serial.print(c); // Print raw response for debugging
    }
  }
  response.trim();
  Serial.print("\nTrimmed Response: ");
  Serial.println(response.length() ? response : "No response");
}

// Send sensor data
void sendSensorData(HardwareSerial &serial, int address, SensorData_t data) {
  char cmd[128];
  char dataStr[100];
  snprintf(dataStr, sizeof(dataStr),
           "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
           data.accel_x, data.accel_y, data.accel_z,
           data.gyro_x, data.gyro_y, data.gyro_z);
  int len = strlen(dataStr);
  snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,%s", address, len, dataStr);
  sendAT(serial, cmd);
  Serial.print("Sent data to address ");
  Serial.println(address);
  Serial.print("dataStr: ");
  Serial.println(dataStr);
}

// Send ping
void sendPing(HardwareSerial &serial, int address) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+SEND=%d,4,PING", address);
  sendAT(serial, cmd);
  Serial.print("Sent PING to address ");
  Serial.println(address);
}

// Calculate distance
void calculateDistance(unsigned long pongReceivedTime) {
  unsigned long rtt = pongReceivedTime - pingSentTime;
  float distance = (rtt / 1000000.0) * (3.0e8 / 2.0); // Distance in meters
  Serial.print("RTT: ");
  Serial.print(rtt);
  Serial.println(" us");
  Serial.print("Estimated Distance: ");
  Serial.print(distance);
  Serial.println(" meters");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial port
  }

  LoRaA.begin(115200, SERIAL_8N1, LORA_A_RX_PIN, LORA_A_TX_PIN);
  Serial.println("Initializing LoRa modules...");

  // Initialize LoRaA
  Serial.println("Initializing LoRaA...");
  sendAT(LoRaA, "AT");              // Test communication
  sendAT(LoRaA, "AT+ADDRESS=0");    // Set address
  sendAT(LoRaA, "AT+NETWORKID=18"); // Set network ID
  sendAT(LoRaA, "AT+PARAMETER=9,7,1,12"); // Set parameters
  sendAT(LoRaA, "AT+BAND=915000000");     // Set frequency band
  
  Serial.println("Initialization complete");
}

void loop() {
  unsigned long now = millis();

  // Send sensor data every 100ms
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    SensorData_t data = generateSensorData();
    sendSensorData(LoRaA, 1, data); // LoRaA -> LoRaB
  }

  // Send ping every 1s
  if (now - lastPing >= PING_INTERVAL && !awaitingPong) {
    lastPing = now;
    sendPing(LoRaA, 1); // LoRaA -> LoRaB
    pingSentTime = micros();
    awaitingPong = true;
  }

  // Handle LoRaA responses
  if (LoRaA.available()) {
    String resp = LoRaA.readStringUntil('\n');
    resp.trim();
    if (resp.startsWith("+RCV=")) {
      Serial.print("A Rx: ");
      Serial.println(resp);
      if (awaitingPong && resp.indexOf("PONG") != -1) {
        unsigned long pongReceivedTime = micros();
        calculateDistance(pongReceivedTime);
        awaitingPong = false;
      }
    }
  }
}