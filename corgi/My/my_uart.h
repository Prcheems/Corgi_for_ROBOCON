#ifndef _MY_UART_H_
#define _MY_UART_H_
#include "stm32h7xx.h"

#define RX_LEN1 78 // 78

typedef struct
{
	uint8_t RX_flag:1;
	uint16_t RX_Size;
	uint8_t RX_pData[RX_LEN1];
}USART_RECEIVETYPE;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
void usart_start_recv(void);
void UsartReceive_IDLE(UART_HandleTypeDef* huart);
void usart_reset(void);
extern int Gamepad_stop_flag;

#endif
