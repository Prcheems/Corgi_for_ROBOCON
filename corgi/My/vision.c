#include "stm32h7xx_hal.h"
#include "vision.h"
#include "usart.h"
#include "speed_race.h"
#include "bluetooth.h"

Vision_Rx vision_rx;

uint8_t vision_rxbuff[16];

void Vision_Recv_Data_Process(uint8_t *rxbuff, Vision_Rx *Rx)
{
	// 定义标识符,表示数据的转化进行到了数组的第几个成员
	uint8_t Pos = 2;
	if (vision_rxbuff[0] == 0x0A && vision_rxbuff[1] == 0x0D && vision_rxbuff[14] == 0x0D && vision_rxbuff[15] == 0x0A)
	{
		for (int i = 0; i < 3; i++)
		{
			Rx->Float_data[i] = Char_to_Float(&rxbuff[Pos]);
			Pos += 4;
		}
	}
	Auto_spd.Sides_dis = Rx->Float_data[0];
	Auto_spd.Front_dis = Rx->Float_data[1];
	Auto_spd.Camera_dire_flag = Rx->Float_data[2];
}

void Vision_Recv_Start(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, &vision_rxbuff[0], 16);
	__HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
}

void Vision_Restart_IT(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, &vision_rxbuff[0], 16);
	__HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
	Vision_Recv_Data_Process(&vision_rxbuff[0], &vision_rx);
}
