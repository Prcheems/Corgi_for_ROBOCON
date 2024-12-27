#ifndef __DOG_VISION_H__
#define __DOG_VISION_H__

#include <stdbool.h>
#include "stdint.h"
#include "usart.h"

typedef struct
{
    char header[2];
    float Float_data[3];
    char end[2];
} Vision_Rx;

extern Vision_Rx vision_rx;
float Char_to_Float(unsigned char data[]);

void Vision_Restart_IT(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx);
void Vision_Recv_Start(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx);
void Vision_Recv_Data_Process(uint8_t *rxbuff, Vision_Rx *Rx);

#endif

