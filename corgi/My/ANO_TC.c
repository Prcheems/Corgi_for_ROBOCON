#include "ANO_TC.h"
#include "usart.h"
#include "string.h"

static uint8_t data[4 + LAN32 * 4 + 1];
void Usart_Send_To_Show32(UART_HandleTypeDef *huart, float *data_ts)
{
    uint8_t i, sum = 0;
    uint8_t data_head[4] = {0xAA, 0xAA, 0xF1, LAN32 * 4};
    memcpy(data, data_head, sizeof(uint8_t) * 4);
    for (i = 0; i < LAN32; i++)
    {
        uint8_t data_head[4] = {BYTE3(data_ts[i]), BYTE2(data_ts[i]), BYTE1(data_ts[i]), BYTE0(data_ts[i])};
        memcpy(data + 4 * (i + 1), data_head, sizeof(char) * 4);
        sum += BYTE3(data_ts[i]) + BYTE2(data_ts[i]) + BYTE1(data_ts[i]) + BYTE0(data_ts[i]);
    }
    sum += 0xAA + 0xAA + 0XF1 + LAN32 * 4;

    memcpy(data + 4 + LAN32 * 4, &sum, sizeof(char) * 1);
    HAL_UART_Transmit_DMA(huart, data, 4 + LAN32 * 4 + 1);
//		HAL_UART_Transmit(huart, data, 4 + LAN32 * 4 + 1,0xfff);
}
