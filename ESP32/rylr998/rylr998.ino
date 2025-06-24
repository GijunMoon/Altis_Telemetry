#include <HardwareSerial.h>

// 핀 정의
#define LORA_A_TX_PIN 18
#define LORA_A_RX_PIN 19
#define LORA_B_TX_PIN 23
#define LORA_B_RX_PIN 22

// UART 인스턴스
HardwareSerial LoRaA(1);
HardwareSerial LoRaB(2);

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 100; // 100ms

// 센서 데이터 구조체 (GCS와 호환, 타임스탬프 제외)
typedef struct {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
} SensorData_t;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  LoRaA.begin(115200, SERIAL_8N1, LORA_A_RX_PIN, LORA_A_TX_PIN);
  LoRaB.begin(9600, SERIAL_8N1, LORA_B_RX_PIN, LORA_B_TX_PIN); // LoRaB 보드레이트 9600

  delay(500);
  Serial.println("Initializing LoRa modules...");

  // LoRa-A 초기화
  Serial.println("Initializing LoRaA...");
  sendAT(LoRaA, "AT");
  sendAT(LoRaA, "AT+ADDRESS=0");
  sendAT(LoRaA, "AT+NETWORKID=18");
  sendAT(LoRaA, "AT+PARAMETER=9,7,1,12");
  sendAT(LoRaA, "AT+BAND=915000000");

  // LoRa-B 초기화
  Serial.println("Initializing LoRaB...");
  sendAT(LoRaB, "AT");
  sendAT(LoRaB, "AT+ADDRESS=1");
  sendAT(LoRaB, "AT+NETWORKID=18");
  sendAT(LoRaB, "AT+PARAMETER=9,7,1,12");
  sendAT(LoRaB, "AT+BAND=915000000");

  Serial.println("Initialization complete");
}

void loop() {
  unsigned long now = millis();

  // 100ms마다 데이터 전송
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    SensorData_t data = generateSensorData();
    sendSensorData(LoRaA, 1, data); // LoRaA -> LoRaB
  }

  // LoRa-B 수신 처리
  if (LoRaB.available()) {
    String resp = LoRaB.readStringUntil('\n');
    resp.trim();
    if (resp.startsWith("+RCV=")) {
      Serial.print("B Rx: ");
      Serial.println(resp);
      String dataStr = processReceivedData(resp);
      if (dataStr.length()) {
        Serial.print("Sending to GCS: "); Serial.println(dataStr); // 디버깅
        Serial.print(dataStr); // GCS로 전송 (\n 포함)
      }
    }
  }

  // LoRa-A 수신 처리 (필요 시)
  if (LoRaA.available()) {
    String resp = LoRaA.readStringUntil('\n');
    resp.trim();
    if (resp.startsWith("+RCV=")) {
      Serial.print("A Rx: ");
      Serial.println(resp);
    }
  }
}

// 센서 데이터 생성 (GCS와 호환, 타임스탬프 제외)
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

// 센서 데이터 전송 (GCS 형식: ASCII, 쉼표 구분, 타임스탬프 제외)
void sendSensorData(HardwareSerial &serial, int address, SensorData_t data) {
  char cmd[128];
  char dataStr[100];

  // GCS 형식: accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
  snprintf(dataStr, sizeof(dataStr),
           "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
           data.accel_x, data.accel_y, data.accel_z,
           data.gyro_x, data.gyro_y, data.gyro_z);

  int len = strlen(dataStr);
  snprintf(cmd, sizeof(cmd), "AT+SEND=%d,%d,%s", address, len, dataStr);

  sendAT(serial, cmd);
  Serial.print("Sent data to address ");
  Serial.println(address);
  Serial.print("Command: ");
  Serial.println(cmd);
}

// 수신 데이터 처리 (GCS 형식으로 변환)
String processReceivedData(String resp) {
  int firstComma = resp.indexOf(',');
  int secondComma = resp.indexOf(',', firstComma + 1);
  int thirdComma = resp.indexOf(',', secondComma + 1);

  String senderStr = resp.substring(5, firstComma);
  String lenStr = resp.substring(firstComma + 1, secondComma);
  String dataStr = resp.substring(secondComma + 1, thirdComma);

  int sender = senderStr.toInt();
  int len = lenStr.toInt();

  // 디버깅: 추출된 dataStr 확인
  Serial.print("Extracted dataStr: ");
  Serial.println(dataStr);

  SensorData_t receivedData;
  // String을 쉼표로 분리하여 파싱
  int startIndex = 0;
  int commaIndex;
  int fieldIndex = 0;
  float values[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 기본값 0.0

  for (int i = 0; i < 6 && startIndex < dataStr.length(); i++) {
    commaIndex = dataStr.indexOf(',', startIndex);
    if (commaIndex == -1) {
      commaIndex = dataStr.length(); // 마지막 필드
    }
    String field = dataStr.substring(startIndex, commaIndex);
    field.trim();
    Serial.print("Field "); Serial.print(i); Serial.print(": "); Serial.println(field); // 디버깅
    values[i] = field.toFloat();
    startIndex = commaIndex + 1;
    fieldIndex++;
  }

  // 값 할당
  receivedData.accel_x = values[0];
  receivedData.accel_y = values[1];
  receivedData.accel_z = values[2];
  receivedData.gyro_x = values[3];
  receivedData.gyro_y = values[4];
  receivedData.gyro_z = values[5];

  // GCS 형식으로 ASCII 문자열 생성 (줄 끝 \n)
  char outputStr[100];
  snprintf(outputStr, sizeof(outputStr),
           "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
           receivedData.accel_x, receivedData.accel_y, receivedData.accel_z,
           receivedData.gyro_x, receivedData.gyro_y, receivedData.gyro_z);

  // 디버깅 출력
  Serial.println("Received Sensor Data:");
  Serial.print("Accel X: "); Serial.println(receivedData.accel_x, 1);
  Serial.print("Accel Y: "); Serial.println(receivedData.accel_y, 1);
  Serial.print("Accel Z: "); Serial.println(receivedData.accel_z, 1);
  Serial.print("Gyro X: "); Serial.println(receivedData.gyro_x, 1);
  Serial.print("Gyro Y: "); Serial.println(receivedData.gyro_y, 1);
  Serial.print("Gyro Z: "); Serial.println(receivedData.gyro_z, 1);

  return String(outputStr);
}

// AT 명령 전송
void sendAT(HardwareSerial &serial, const char *cmd) {
  serial.print(cmd);
  serial.print("\r\n");
  Serial.print("Sending to ");
  Serial.print(&serial == &LoRaA ? "LoRaA" : "LoRaB");
  Serial.print(": ");
  Serial.println(cmd);
  serial.flush();
  delay(1000);
  String response = "";
  unsigned long start = millis();
  while (millis() - start < 1000 && serial.available()) {
    response += serial.readStringUntil('\n');
    delay(10);
  }
  response.trim();
  Serial.print("  Response: ");
  Serial.println(response.length() ? response : "No response");
}