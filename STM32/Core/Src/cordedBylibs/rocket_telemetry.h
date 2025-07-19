#ifndef ROCKET_TELEMETRY_H_
#define ROCKET_TELEMETRY_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#define TELEMETRY_UART_HANDLE   huart2
#define GROUND_STATION_ADDR     1
#define ROCKET_ADDR             2
#define LORA_NETWORK_ID         18

typedef enum
{
    ROCKET_STATE_PREP       = 0,
    ROCKET_STATE_ASCENT     = 1,
    ROCKET_STATE_APOGEE     = 2,
    ROCKET_STATE_DESCENT    = 3,
    ROCKET_STATE_LANDED     = 4,
    ROCKET_STATE_ERROR      = 5
} RocketState_e;

typedef struct __attribute__((packed))
{
	uint32_t packet_count;
    uint32_t timestamp;
    float altitude;
    float velocity;
    float accel[3];
    float gyro[3];
    float quat[4];
    uint8_t checksum;
} RocketData_t;


typedef enum
{
    TELEMETRY_OK = 0x00U,
    TELEMETRY_ERROR,
    TELEMETRY_BUSY,
    TELEMETRY_TIMEOUT,
    TELEMETRY_INVALID_PACKET,
    TELEMETRY_CHECKSUM_ERROR
} Telemetry_Status_t;

extern UART_HandleTypeDef TELEMETRY_UART_HANDLE;
extern RocketData_t rocketData;

Telemetry_Status_t Telemetry_Init(uint16_t myAddress);

Telemetry_Status_t Telemetry_SendData(RocketData_t* rocketData, uint16_t targetAddress);

Telemetry_Status_t Telemetry_ParseReceivedPacket(uint8_t* rxBuffer, uint16_t rxLen, RocketData_t* parsedData);

#endif
