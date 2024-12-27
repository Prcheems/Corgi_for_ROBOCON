#ifndef __Bluetooth_H
#define __Bluetooth_H

#include <stdbool.h>
#include "stdint.h"
#include "usart.h"
// 指定发送到手机的数据包的结构----------在发送时会自动额外在前后加上包头,包尾和校验和数据,因此会多出3个字节
// 根据实际需要的变量,定义数据包中 byte short int float 4种类型的数目

#define TX_BYTE_NUM 1
#define TX_SHORT_NUM 1
#define TX_INT_NUM 2
#define TX_FLOAT_NUM 1
#define TX_BUFF_SIZE TX_BYTE_NUM + TX_SHORT_NUM * 2 + TX_INT_NUM * 4 + TX_FLOAT_NUM * 4 + 3

// 指定接收数据包的结构-----------------------------------------------------------------------------------
// 根据实际需要的变量,定义数据包中 byte short int float 4种类型的数目
// 板子->手机

/*
// Action
#define RX_BYTE_NUM 1
#define RX_SHORT_NUM 41
#define RX_INT_NUM 1
#define RX_FLOAT_NUM 1
#define RX_BUFF_SIZE RX_BYTE_NUM + RX_SHORT_NUM * 2 + RX_INT_NUM * 4 + RX_FLOAT_NUM * 4 + 3
*/

/*
// Walk
#define RX_BYTE_NUM 1
#define RX_SHORT_NUM 41
#define RX_INT_NUM 1
#define RX_FLOAT_NUM 6
#define RX_BUFF_SIZE RX_BYTE_NUM + RX_SHORT_NUM * 2 + RX_INT_NUM * 4 + RX_FLOAT_NUM * 4 + 3
*/


// Jump
#define RX_BYTE_NUM 1
#define RX_SHORT_NUM 41
#define RX_INT_NUM 1
#define RX_FLOAT_NUM 9
#define RX_BUFF_SIZE RX_BYTE_NUM + RX_SHORT_NUM * 2 + RX_INT_NUM * 4 + RX_FLOAT_NUM * 4 + 3



#pragma pack(1)
typedef struct
{
    char head;

#if (RX_BYTE_NUM != 0)
    char Byte_data[RX_BYTE_NUM];
#endif

#if (RX_SHORT_NUM != 0)
    short int Short_data[RX_SHORT_NUM];
#endif

#if (RX_INT_NUM != 0)
    int Int_data[RX_INT_NUM];
#endif

#if (RX_FLOAT_NUM != 0)
    float Float_data[RX_FLOAT_NUM];
#endif
    char last;
} Rx_Frame;
#pragma pack()

#pragma pack(1)
typedef struct
{
    char head;

#if (TX_BYTE_NUM != 0)
    char Byte_data[TX_BYTE_NUM];
#endif

#if (TX_SHORT_NUM != 0)
    short int Short_data[TX_SHORT_NUM];
#endif

#if (TX_INT_NUM != 0)
    int Int_data[TX_INT_NUM];
#endif

#if (TX_FLOAT_NUM != 0)
    float Float_data[TX_FLOAT_NUM];
#endif
    char last;
} Tx_Frame;
#pragma pack()

extern Tx_Frame Tx;
extern Rx_Frame Rx;

int Char_to_Int(unsigned char data[]);
void Int_to_Char(int num, unsigned char data[]);
void Float_to_Char(float num, unsigned char data[]);
float Char_to_Float(unsigned char data[]);
void Short_to_Char(short int num, unsigned char data[]);
short int Char_to_Short(unsigned char data[]);

void Make_Transmit_Data_Pack(Tx_Frame *Tx);
void Transmit_Data_Pack(UART_HandleTypeDef *huart);
void Bluetooth_Restart_IT(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx);
void Bluetooth_Recv_Start(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx);
void Bluetooth_Recv_Data_Process(uint8_t *rxbuff, Rx_Frame *Rx);
#endif
