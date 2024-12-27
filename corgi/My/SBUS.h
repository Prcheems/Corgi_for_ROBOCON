#ifndef __SBUS_H__
#define __SBUS_H__

#include "usart.h"
#include "stdint.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

void SB_USART_Start(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx);

void SBUS_Rcv_Prcs(uint8_t *data);
//void SBUS_Handle(void);
extern uint8_t RxTemp_2[25];

extern int16_t g_sbus_channels[8];


#endif
