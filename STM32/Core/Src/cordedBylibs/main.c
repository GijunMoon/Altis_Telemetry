/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LoRa A/B AT communication test with sensor data and ping/pong (USART2 + USB CDC)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include "rocket_telemetry.h"

/* Private define ------------------------------------------------------------*/
#define MAX_BUF 256
#define SEND_INTERVAL 100  // 100ms for sensor data
#define PING_INTERVAL 1000 // 1s for ping

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;  // LoRa-A (PA2=TX, PA3=RX)
static char loraA_rx[MAX_BUF], loraA_tx[MAX_BUF];
static volatile uint8_t loraA_flag = 0;
static uint16_t loraA_idx = 0;
static uint32_t lastSend = 0;
static uint32_t lastPing = 0;
static uint32_t pingSentTime = 0;
static volatile uint8_t awaitingPong = 0;

/* Sensor data structure */
typedef struct {
  uint32_t time;       // ms
  float alt;           // altitude
  float vel;           // velocity
  float accel_x, accel_y, accel_z; // accelerometer
  float gyro_x, gyro_y, gyro_z;   // gyroscope
  float quat_w, quat_x, quat_y, quat_z; // quaternion
  float ftv_ej_x, ftv_ej_y, ftv_ej_z;   // ftv_ej
} SensorData_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void LoRaA_Init(void);
static void LoRaA_Send(const char* msg);
static void SendSensorData(int address, SensorData_t data);
static void SendPing(int address);
static void CalculateDistance(uint32_t pongReceivedTime);
static SensorData_t GenerateSensorData(void);
static void LogMessage(const char* msg);
static void LogBuffer(const char* prefix, const char* buffer, uint16_t len);

/* UART Rx Complete Callback -------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    if (loraA_rx[loraA_idx] == '\n') {
      loraA_rx[loraA_idx] = '\0';
      loraA_flag = 1;
      loraA_idx = 0;
    } else {
      if (++loraA_idx >= MAX_BUF-1) loraA_idx = 0;
    }
    HAL_UART_Receive_IT(&huart2, (uint8_t*)&loraA_rx[loraA_idx], 1);
  }
}

/**
  * @brief  Main entry point.
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();

  /* Initialize UART receive interrupt */
  HAL_UART_Receive_IT(&huart2, (uint8_t*)loraA_rx, 1);

  /* Initialize LoRa module */
  LoRaA_Init();
  LogMessage("LoRa A Initialized\r\n");

  uint32_t last_tick = HAL_GetTick();

  Telemetry_Init(ROCKET_ADDR);

  /* Infinite loop */
  while (1)
  {
//    /* Send sensor data every 100ms */
//    if (HAL_GetTick() - lastSend >= SEND_INTERVAL) {
//      lastSend = HAL_GetTick();
//      SensorData_t data = GenerateSensorData();
//      SendSensorData(1, data); // Send to address 1 (LoRaB)
//    }
//
//    /* Send ping every 1s */
//    if (HAL_GetTick() - lastPing >= PING_INTERVAL && !awaitingPong) {
//      lastPing = HAL_GetTick();
//      SendPing(1); // Send to address 1 (LoRaB)
//      pingSentTime = HAL_GetTick() * 1000; // Convert to us
//      awaitingPong = 1;
//    }
//
//    /* Handle LoRaA responses */
//    if (loraA_flag) {
//      loraA_flag = 0;
//      LogBuffer("A Rx: ", loraA_rx, strlen(loraA_rx));
//      if (awaitingPong && strstr(loraA_rx, "PONG") != NULL) {
//        uint32_t pongReceivedTime = HAL_GetTick() * 1000; // Convert to us
//        CalculateDistance(pongReceivedTime);
//        awaitingPong = 0;
//      }
//      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13); // Toggle LED on data receive
//    }

	  Telemetry_SendData(&rocketData, GROUND_STATION_ADDR);

    HAL_Delay(10);
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief LoRa-A initialization (ADDRESS=0, NETWORKID=18, PARAMETER, BAND)
  */
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

/**
  * @brief LoRa-A send message
  */
static void LoRaA_Send(const char* msg)
{
  snprintf(loraA_tx, MAX_BUF, "AT+SEND=1,%d,%s\r\n", (int)strlen(msg), msg);
  HAL_UART_Transmit(&huart2, (uint8_t*)loraA_tx, strlen(loraA_tx), 1000);
  LogBuffer("Sent: ", loraA_tx, strlen(loraA_tx));
}

/**
  * @brief Send sensor data in specified format
  */
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
        LogMessage("Buffer overflow or format error!\r\n");
        return;
    }

    snprintf(loraA_tx, MAX_BUF, "AT+SEND=%d,%d,%s\r\n", address, (int)strlen(dataStr), dataStr);
    HAL_UART_Transmit(&huart2, (uint8_t*)loraA_tx, strlen(loraA_tx), 1000);

    // 데이터가 실제 맞는지 추가로 찍기
    LogMessage("Debug actual sensor string:\r\n");
    LogMessage(dataStr);
    LogMessage("\r\n");

    LogBuffer("Sent data: ", dataStr, strlen(dataStr));
}


/**
  * @brief Send ping message
  */
static void SendPing(int address)
{
  snprintf(loraA_tx, MAX_BUF, "AT+SEND=%d,4,PING\r\n", address);
  HAL_UART_Transmit(&huart2, (uint8_t*)loraA_tx, strlen(loraA_tx), 1000);
  LogBuffer("Sent: ", loraA_tx, strlen(loraA_tx));
}

/**
  * @brief Calculate distance based on RTT
  */
static void CalculateDistance(uint32_t pongReceivedTime)
{
  uint32_t rtt = pongReceivedTime - pingSentTime; // in us
  float distance = (rtt / 1000000.0f) * (3.0e8 / 2.0f); // Distance in meters
  char msg[100];
  snprintf(msg, sizeof(msg), "RTT: %lu us, Distance: %.2f meters\r\n", rtt, distance);
  LogMessage(msg);
}

/**
  * @brief Generate dummy sensor data based on provided format
  */
static SensorData_t GenerateSensorData(void)
{
  static uint32_t time = 1660;
  SensorData_t data = {
    .time = time++,
    .alt = 0.0467f,
    .vel = -2.4665f,
    .accel_x = 0.3199f,
    .accel_y = 0.6228f,
    .accel_z = 2.5463f,
    .gyro_x = 53.0914f,
    .gyro_y = -0.0499f,
    .gyro_z = 80.687f,
    .quat_w = 0.0081f,
    .quat_x = 0.0201f,
    .quat_y = -0.0359f,
    .quat_z = 0.0016f,
    .ftv_ej_x = 0.0f,
    .ftv_ej_y = 0.0f,
    .ftv_ej_z = 0.0f
  };
  return data;
}

/**
  * @brief Log message via USB CDC
  */
static void LogMessage(const char* msg)
{
  CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

/**
  * @brief Log buffer with prefix via USB CDC
  */
static void LogBuffer(const char* prefix, const char* buffer, uint16_t len)
{
  CDC_Transmit_FS((uint8_t*)prefix, strlen(prefix));
  CDC_Transmit_FS((uint8_t*)buffer, len);
  CDC_Transmit_FS((uint8_t*)"\r\n", 2);
}

/* USER CODE END 4 */

/* Error Handler -------------------------------------------------------------*/
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

/**
  * @brief System Clock Configuration
  */
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

/**
  * @brief USART2 init (PA2=TX, PA3=RX)
  */
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

/**
  * @brief GPIO init (PD13/PD14 LEDs)
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_Init = {0};
  GPIO_Init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_Init);

  GPIO_Init.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_Init.Mode = GPIO_MODE_AF_PP;
  GPIO_Init.Pull = GPIO_NOPULL;
  GPIO_Init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_Init.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_Init);
}
