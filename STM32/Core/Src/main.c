#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>

#define MAX_BUF         256
#define SEND_INTERVAL   100       // ms
#define PING_INTERVAL   1000      // ms

UART_HandleTypeDef huart2;

static char loraA_rx[MAX_BUF];
static char loraA_tx[MAX_BUF];
static volatile uint8_t loraA_flag = 0;
static uint16_t loraA_idx = 0;

static uint32_t lastSend = 0;
static uint32_t lastPing = 0;
static uint32_t pingSentTime = 0;
static uint8_t awaitingPong = 0;

typedef struct {
    uint32_t time;
    float alt;
    float vel;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float quat_w, quat_x, quat_y, quat_z;
    float ftv_ej_x, ftv_ej_y, ftv_ej_z;
} SensorData_t;

// Function Prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void LoRaA_Init(void);
static void SendSensorData(int address, SensorData_t data);
static void SendPing(int address);
static void CalculateDistance(uint32_t pongReceivedTime);
static SensorData_t GenerateSensorData(void);
static void LogMessage(const char* msg);
static void LogBuffer(const char* prefix, const char* buffer, uint16_t len);

// UART 수신 인터럽트
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        if (loraA_rx[loraA_idx] == '\n' || loraA_idx >= MAX_BUF - 2) {
            loraA_rx[loraA_idx] = '\0';
            loraA_flag = 1;
            loraA_idx = 0;
        } else {
            loraA_idx++;
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&loraA_rx[loraA_idx], 1);
    }
}

// Main Function
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_USART2_UART_Init();

    HAL_UART_Receive_IT(&huart2, (uint8_t*)loraA_rx, 1);  // UART 수신 시작
    LoRaA_Init();
    LogMessage("LoRa A Initialized\r\n");

    while (1)
    {
        uint32_t tick = HAL_GetTick();

        // 센서 데이터 전송
        if (tick - lastSend >= SEND_INTERVAL) {
            lastSend = tick;
            SensorData_t data = GenerateSensorData();
            SendSensorData(1, data);  // LoRa로 데이터 전송
        }

        // Ping 전송
        if (tick - lastPing >= PING_INTERVAL && !awaitingPong) {
            lastPing = tick;
            SendPing(1);  // 주소 1로 Ping
            pingSentTime = tick * 1000;
            awaitingPong = 1;
        }

        // LoRa 수신 처리
        if (loraA_flag) {
            loraA_flag = 0;
            LogBuffer("Recv: ", loraA_rx, strlen(loraA_rx));

            if (awaitingPong && strstr(loraA_rx, "PONG") != NULL) {
                uint32_t pongTime = HAL_GetTick() * 1000;
                CalculateDistance(pongTime);
                awaitingPong = 0;
            }

            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);  // 응답 수신 시 LED 토글
        }

        HAL_Delay(10);
    }
}

// LoRa 초기화
static void LoRaA_Init(void)
{
    HAL_Delay(500);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT\r\n", 4, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+ADDRESS=0\r\n", 14, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+NETWORKID=18\r\n", 17, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+PARAMETER=9,7,1,12\r\n", 23, 1000);
    HAL_Delay(100);
    HAL_UART_Transmit(&huart2, (uint8_t*)"AT+BAND=915000000\r\n", 19, 1000);
}

// 센서 데이터 전송
static void SendSensorData(int address, SensorData_t data)
{
    char dataStr[200];
    int len = snprintf(dataStr, sizeof(dataStr),
        "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
        data.time, data.alt, data.vel,
        data.accel_x, data.accel_y, data.accel_z,
        data.gyro_x, data.gyro_y, data.gyro_z,
        data.quat_w, data.quat_x, data.quat_y, data.quat_z,
        data.ftv_ej_x, data.ftv_ej_y, data.ftv_ej_z);

    if (len < 0 || len >= sizeof(dataStr)) {
        LogMessage("Sensor format error!\r\n");
        return;
    }

    snprintf(loraA_tx, MAX_BUF, "AT+SEND=%d,%d,%s\r\n", address, (int)strlen(dataStr), dataStr);
    HAL_UART_Transmit(&huart2, (uint8_t*)loraA_tx, strlen(loraA_tx), 1000);
    LogBuffer("Sent data: ", dataStr, strlen(dataStr));
}

// Ping 전송
static void SendPing(int address)
{
    snprintf(loraA_tx, MAX_BUF, "AT+SEND=%d,4,PING\r\n", address);
    HAL_UART_Transmit(&huart2, (uint8_t*)loraA_tx, strlen(loraA_tx), 1000);
    LogBuffer("Sent: ", loraA_tx, strlen(loraA_tx));
}

// 거리 계산
static void CalculateDistance(uint32_t pongReceivedTime)
{
    uint32_t rtt = pongReceivedTime - pingSentTime; // us 단위
    float distance = (rtt / 1000000.0f) * (3.0e8f / 2.0f);
    char msg[100];
    snprintf(msg, sizeof(msg), "RTT: %lu us, Distance: %.2f m\r\n", rtt, distance);
    LogMessage(msg);
}

// 가상 센서 데이터 생성
static SensorData_t GenerateSensorData(void)
{
    static uint32_t time = 1;

    return (SensorData_t){
        .time = time++,
        .alt = 1.23f,
        .vel = 4.56f,
        .accel_x = 0.1f, .accel_y = 0.2f, .accel_z = 0.3f,
        .gyro_x = 1.1f, .gyro_y = 1.2f, .gyro_z = 1.3f,
        .quat_w = 0.7071f, .quat_x = 0.0f, .quat_y = 0.7071f, .quat_z = 0.0f,
        .ftv_ej_x = 0.0f, .ftv_ej_y = 0.0f, .ftv_ej_z = 0.0f
    };
}

// USB CDC 로그 출력
static void LogMessage(const char* msg)
{
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

// USB CDC 버퍼 출력
static void LogBuffer(const char* prefix, const char* buffer, uint16_t len)
{
    CDC_Transmit_FS((uint8_t*)prefix, strlen(prefix));
    CDC_Transmit_FS((uint8_t*)buffer, len);
    CDC_Transmit_FS((uint8_t*)"\r\n", 2);
}

// 시스템 클럭
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_Osc = {0};
    RCC_ClkInitTypeDef RCC_Clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_Osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_Osc.HSEState = RCC_HSE_ON;
    RCC_Osc.PLL.PLLState = RCC_PLL_ON;
    RCC_Osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_Osc.PLL.PLLM = 8;
    RCC_Osc.PLL.PLLN = 336;
    RCC_Osc.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_Osc.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_Osc) != HAL_OK) Error_Handler();
    RCC_Clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_Clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_Clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_Clk.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_Clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_Clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

// USART2 사용 (PA2 TX, PA3 RX)
static void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

// GPIO 설정
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Pin = GPIO_PIN_13;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_Init);

    GPIO_Init.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_Init.Mode = GPIO_MODE_AF_PP;
    GPIO_Init.Pull = GPIO_NOPULL;
    GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_Init.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_Init);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1);
}
