/*
 * rocket_telemetry.c
 *
 *  Created on: Jul 15, 2025
 *      Author: potch
 */
#include "rocket_telemetry.h"

#define AT_PREFIX           "AT+"
#define AT_TERMINATOR       "\r\n"
#define AT_TIMEOUT_MS       200

#define CMD_ADDRESS         "ADDRESS"
#define CMD_NETWORKID       "NETWORKID"
#define CMD_BAND            "BAND"
#define CMD_PARAMETER       "PARAMETER"
#define CMD_SEND            "SEND"

static Telemetry_Status_t _Telemetry_SendCommand(const char* command, const char* value);
static uint8_t _Telemetry_CalculateChecksum(const RocketData_t* data);

RocketData_t rocketData = {0};


Telemetry_Status_t Telemetry_Init(uint16_t myAddress)
{
    char addrStr[6];
    char netIdStr[3];

    HAL_Delay(1000);

    sprintf(addrStr, "%u", myAddress);
    if (_Telemetry_SendCommand(CMD_ADDRESS, addrStr) != TELEMETRY_OK) {
        return TELEMETRY_ERROR;
    }
    HAL_Delay(100);

    sprintf(netIdStr, "%u", LORA_NETWORK_ID);
    if (_Telemetry_SendCommand(CMD_NETWORKID, netIdStr) != TELEMETRY_OK) {
        return TELEMETRY_ERROR;
    }
    HAL_Delay(100);

    if (_Telemetry_SendCommand(CMD_BAND, "915000000") != TELEMETRY_OK) {
        return TELEMETRY_ERROR;
    }
    HAL_Delay(100);

    if (_Telemetry_SendCommand(CMD_PARAMETER, "9,7,1,12") != TELEMETRY_OK) {
        return TELEMETRY_ERROR;
    }
    HAL_Delay(100);

    return TELEMETRY_OK;
}

Telemetry_Status_t Telemetry_SendData(RocketData_t* rocketData, uint16_t targetAddress)
{
    rocketData->packet_count++;

    rocketData->checksum = _Telemetry_CalculateChecksum(rocketData);

    uint8_t txBuffer[300];
    int cmd_len = sprintf((char*)txBuffer, "%s%s=%u,%u,",
                          AT_PREFIX,
                          CMD_SEND,
                          targetAddress,
                          sizeof(RocketData_t));

    memcpy(txBuffer + cmd_len, rocketData, sizeof(RocketData_t));

    uint16_t total_len = cmd_len + sizeof(RocketData_t);
    memcpy(txBuffer + total_len, AT_TERMINATOR, strlen(AT_TERMINATOR));
    total_len += strlen(AT_TERMINATOR);

    if (HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, txBuffer, total_len, AT_TIMEOUT_MS) != HAL_OK)
    {
        return TELEMETRY_ERROR;
    }

    return TELEMETRY_OK;
}

Telemetry_Status_t Telemetry_ParseReceivedPacket(uint8_t* rxBuffer, uint16_t rxLen, RocketData_t* parsedData)
{
    if (strncmp((char*)rxBuffer, "+RCV=", 5) != 0)
    {
        return TELEMETRY_INVALID_PACKET;
    }

    char* token_start = (char*)rxBuffer + 5;
    char* token_end = strchr(token_start, ',');
    if (token_end == NULL) return TELEMETRY_INVALID_PACKET;

    token_start = token_end + 1;
    token_end = strchr(token_start, ',');
    if (token_end == NULL) return TELEMETRY_INVALID_PACKET;

    int data_len = atoi(token_start);
    if (data_len != sizeof(RocketData_t))
    {
        return TELEMETRY_INVALID_PACKET;
    }

    uint8_t* data_ptr = (uint8_t*)token_end + 1;

    memcpy(parsedData, data_ptr, sizeof(RocketData_t));

    uint8_t received_checksum = parsedData->checksum;
    uint8_t calculated_checksum = _Telemetry_CalculateChecksum(parsedData);

    if (received_checksum != calculated_checksum)
    {
        return TELEMETRY_CHECKSUM_ERROR;
    }

    return TELEMETRY_OK;
}


static Telemetry_Status_t _Telemetry_SendCommand(const char* command, const char* value)
{
    char tx_buf[100];
    uint8_t rx_buf[20];
    HAL_StatusTypeDef status;

    int len = sprintf(tx_buf, "%s%s=%s%s", AT_PREFIX, command, value, AT_TERMINATOR);

    __HAL_UART_FLUSH_DRREGISTER(&TELEMETRY_UART_HANDLE);

    status = HAL_UART_Transmit(&TELEMETRY_UART_HANDLE, (uint8_t*)tx_buf, len, AT_TIMEOUT_MS);
    if (status != HAL_OK) return TELEMETRY_ERROR;

    status = HAL_UART_Receive(&TELEMETRY_UART_HANDLE, rx_buf, sizeof(rx_buf), AT_TIMEOUT_MS);
    if (status != HAL_OK) return TELEMETRY_TIMEOUT;

    if (strstr((char*)rx_buf, "+OK") != NULL)
    {
        return TELEMETRY_OK;
    }

    return TELEMETRY_ERROR;
}

static uint8_t _Telemetry_CalculateChecksum(const RocketData_t* data)
{
    uint8_t checksum = 0;
    const uint8_t* ptr = (const uint8_t*)data;

    for (size_t i = 0; i < sizeof(RocketData_t) - sizeof(data->checksum); ++i)
    {
        checksum += ptr[i];
    }
    return checksum;
}
