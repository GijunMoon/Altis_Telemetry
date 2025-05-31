/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// 센서 데이터 구조체
typedef struct {
    uint32_t timestamp;
    int32_t gyro_x, gyro_y, gyro_z;        // 정수로 변환 (x100)
    int32_t accel_x, accel_y, accel_z;     // 정수로 변환 (x100)
    int32_t altitude;                       // 정수로 변환 (x10)
    int32_t temperature;                    // 정수로 변환 (x10)
    int32_t pressure;                       // 정수로 변환 (x100)
} SensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_BATCH_SIZE 3    // 한 번에 3개 센서 데이터 전송
#define MAX_TEXT_PACKET_SIZE 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// LoRa 통신 버퍼
char lora_rx_buffer[256];
char lora_tx_buffer[256];
char usb_tx_buffer[256];
volatile uint8_t lora_rx_complete = 0;
uint16_t lora_rx_index = 0;

// 센서 데이터 버퍼
SensorData_t sensor_buffer[SENSOR_BATCH_SIZE];
volatile uint8_t sensor_count = 0;
volatile uint8_t data_ready_flag = 0;
uint32_t packet_id = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  // 시스템 초기화 대기
  HAL_Delay(2000);

  // UART 인터럽트 수신 시작
  HAL_UART_Receive_IT(&huart1, (uint8_t*)lora_rx_buffer, 1);

  // RYLR998 초기화
  HAL_UART_Transmit(&huart1, (uint8_t*)"AT\r\n", 4, 1000);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart1, (uint8_t*)"AT+ADDRESS=0\r\n", 14, 1000);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart1, (uint8_t*)"AT+NETWORKID=18\r\n", 17, 1000);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart1, (uint8_t*)"AT+PARAMETER=9,7,1,12\r\n", 23, 1000);
  HAL_Delay(500);
  HAL_UART_Transmit(&huart1, (uint8_t*)"AT+BAND=915000000\r\n", 19, 1000);
  HAL_Delay(500);

  // 초기화 완료 메시지
  CDC_Transmit_FS((uint8_t*)"STM32 센서 시스템 초기화 완료\r\n", 34);

  // 초기화 완료 LED
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 센서 데이터 수집 (50Hz)
    static uint32_t last_sensor_read = 0;
    if(HAL_GetTick() - last_sensor_read >= 20)
    {
        last_sensor_read = HAL_GetTick();

        // 가상 센서 데이터 생성
        if(sensor_count < SENSOR_BATCH_SIZE)
        {
            static float time_factor = 0.0f;
            time_factor += 0.1f;

            sensor_buffer[sensor_count].timestamp = HAL_GetTick();

            // float를 정수로 변환 (소수점 2자리 유지)
            sensor_buffer[sensor_count].gyro_x = (int32_t)(1050 * sinf(time_factor));      // 10.50 * sin(t) * 100
            sensor_buffer[sensor_count].gyro_y = (int32_t)(1520 * cosf(time_factor));     // 15.20 * cos(t) * 100
            sensor_buffer[sensor_count].gyro_z = (int32_t)(870 * sinf(time_factor * 0.8f)); // 8.70 * sin(t) * 100

            sensor_buffer[sensor_count].accel_x = (int32_t)(102 + 15 * sinf(time_factor * 2.0f));  // 1.02 + 0.15*sin * 100
            sensor_buffer[sensor_count].accel_y = (int32_t)(25 * cosf(time_factor * 1.5f));        // 0.25*cos * 100
            sensor_buffer[sensor_count].accel_z = (int32_t)(981 + 30 * sinf(time_factor * 0.5f));  // 9.81 + 0.30*sin * 100

            sensor_buffer[sensor_count].altitude = (int32_t)(1205 + 452 * sinf(time_factor * 0.1f));    // 120.5 + 45.2*sin * 10
            sensor_buffer[sensor_count].temperature = (int32_t)(248 + 45 * cosf(time_factor * 0.05f));  // 24.8 + 4.5*cos * 10
            sensor_buffer[sensor_count].pressure = (int32_t)(101325 + 870 * sinf(time_factor * 0.03f)); // 1013.25 + 8.7*sin * 100

            sensor_count++;

            // 배치가 가득 차면 전송 준비
            if(sensor_count >= SENSOR_BATCH_SIZE)
            {
                data_ready_flag = 1;
            }
        }
    }

    // 수신 데이터 처리
    if(lora_rx_complete)
    {
        lora_rx_complete = 0;

        // USB CDC로 수신 데이터 출력
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "수신: %s\r\n", lora_rx_buffer);
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));

        // 수신 LED 토글
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);

        // 다음 수신 준비
        HAL_UART_Receive_IT(&huart1, (uint8_t*)lora_rx_buffer, 1);
        lora_rx_index = 0;
    }

    // 센서 데이터 전송
    if(data_ready_flag)
    {
        data_ready_flag = 0;

        // CSV 형태로 데이터 변환
        snprintf(lora_tx_buffer, sizeof(lora_tx_buffer),
            "AT+SEND=1,150,ID%lu:%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld;%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld;%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
            packet_id++,
            // 센서 1
            sensor_buffer[0].timestamp, sensor_buffer[0].gyro_x, sensor_buffer[0].gyro_y, sensor_buffer[0].gyro_z,
            sensor_buffer[0].accel_x, sensor_buffer[0].accel_y, sensor_buffer[0].accel_z,
            sensor_buffer[0].altitude, sensor_buffer[0].temperature, sensor_buffer[0].pressure,
            // 센서 2
            sensor_buffer[1].timestamp, sensor_buffer[1].gyro_x, sensor_buffer[1].gyro_y, sensor_buffer[1].gyro_z,
            sensor_buffer[1].accel_x, sensor_buffer[1].accel_y, sensor_buffer[1].accel_z,
            sensor_buffer[1].altitude, sensor_buffer[1].temperature, sensor_buffer[1].pressure,
            // 센서 3
            sensor_buffer[2].timestamp, sensor_buffer[2].gyro_x, sensor_buffer[2].gyro_y, sensor_buffer[2].gyro_z,
            sensor_buffer[2].accel_x, sensor_buffer[2].accel_y, sensor_buffer[2].accel_z,
            sensor_buffer[2].altitude, sensor_buffer[2].temperature, sensor_buffer[2].pressure
        );

        // LoRa로 전송
        HAL_UART_Transmit(&huart1, (uint8_t*)lora_tx_buffer, strlen(lora_tx_buffer), 2000);

        // USB CDC로 전송 상태 출력
        snprintf(usb_tx_buffer, sizeof(usb_tx_buffer), "센서 데이터 전송 완료 [ID:%lu]\r\n", packet_id - 1);
        CDC_Transmit_FS((uint8_t*)usb_tx_buffer, strlen(usb_tx_buffer));

        // 전송 LED 토글
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

        // 버퍼 초기화
        sensor_count = 0;
    }

    HAL_Delay(1);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// UART 수신 완료 콜백 (중복 정의 방지를 위해 약한 정의 확인)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        if(lora_rx_buffer[lora_rx_index] == '\n')
        {
            lora_rx_buffer[lora_rx_index] = '\0';
            lora_rx_complete = 1;
        }
        else
        {
            lora_rx_index++;
            if(lora_rx_index >= 255)
            {
                lora_rx_index = 0;
            }
            HAL_UART_Receive_IT(&huart1, (uint8_t*)&lora_rx_buffer[lora_rx_index], 1);
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
