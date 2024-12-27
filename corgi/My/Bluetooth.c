#include "Bluetooth.h"

Tx_Frame Tx;
Rx_Frame Rx;

int Char_to_Int(unsigned char data[])
{
	// 对拆分后的4个字节进行重组,模拟接收到hex后的数据还原过程
	int res;
	unsigned char *pp = (unsigned char *)&res;
	pp[0] = data[0];
	pp[1] = data[1];
	pp[2] = data[2];
	pp[3] = data[3];
	return res;
}

void Int_to_Char(int num, unsigned char data[])
{
	unsigned char *p = (unsigned char *)&num + 3; // 指针p先指向int的最高字节

	// 获取对应的4个字节,从低位到高位,这时就可以用于数据传输了
	data[0] = *(p - 3);
	data[1] = *(p - 2);
	data[2] = *(p - 1);
	data[3] = *p;
}

void Float_to_Char(float num, unsigned char data[])
{
	unsigned char *p = (unsigned char *)&num + 3; // 指针p先指向int的最高字节

	// 获取对应的4个字节,从低位到高位,这时就可以用于hex格式的数据传输了
	data[0] = *(p - 3);
	data[1] = *(p - 2);
	data[2] = *(p - 1);
	data[3] = *p;
}

float Char_to_Float(unsigned char data[])
{
	// 对拆分后的4个字节进行重组,模拟接收到hex后的数据还原过程
	float res;
	unsigned char *pp = (unsigned char *)&res;
	pp[0] = data[0];
	pp[1] = data[1];
	pp[2] = data[2];
	pp[3] = data[3];
	return res;
}

void Short_to_Char(short int num, unsigned char data[])
{
	unsigned char *p = (unsigned char *)&num + 1; // 指针p先指向int的最高字节

	// 获取对应的2个字节,从低位到高位,这时就可以用于hex格式的数据传输了
	data[0] = *(p - 1);
	data[1] = *p;
}

short int Char_to_Short(unsigned char data[])
{
	// 对拆分后的2个字节进行重组,模拟接收到hex后的数据还原过程
	short int res;
	unsigned char *pp = (unsigned char *)&res;
	pp[0] = data[0];
	pp[1] = data[1];
	return res;
}

// 定义缓冲数组
uint8_t txbuff[TX_BUFF_SIZE];

void Make_Transmit_Data_Pack(Tx_Frame *Tx)
{
	// 定义标识符,表示数据的转化进行到了数组的第几个成员
	uint8_t Pos = 1;

	// 包头包尾
	txbuff[0] = 0xA5;
	txbuff[TX_BUFF_SIZE - 1] = 0x5A;

// Byte量数据处理
#if (TX_BYTE_NUM != 0)
	for (int i = 0; i < TX_BYTE_NUM; i++)
	{
		txbuff[1 + i] = Tx->Byte_data[i];
		Pos++;
	}
#endif

#if (TX_SHORT_NUM != 0)
	for (int i = 0; i < TX_SHORT_NUM; i++)
	{
		Short_to_Char(Tx->Short_data[i], &txbuff[Pos]);
		Pos += 2;
	}
#endif

#if (TX_INT_NUM != 0)
	for (int i = 0; i < TX_INT_NUM; i++)
	{
		Int_to_Char(Tx->Int_data[i], &txbuff[Pos]);
		Pos += 4;
	}
#endif

#if (TX_SHORT_NUM != 0)
	for (int i = 0; i < TX_SHORT_NUM; i++)
	{
		Float_to_Char(Tx->Float_data[i], &txbuff[Pos]);
		Pos += 4;
	}
#endif
	// 计算校验和
	int sum = 0;
	for (int i = 0; i < (TX_BUFF_SIZE - 1); i++)
	{
		sum += txbuff[i];
	}
	txbuff[TX_BUFF_SIZE - 2] = (sum & 0xFF);
}

void Transmit_Data_Pack(UART_HandleTypeDef *huart)
{
	Make_Transmit_Data_Pack(&Tx);
	HAL_UART_Transmit_DMA(huart, txbuff, TX_BUFF_SIZE);
}

uint8_t rxbuff[RX_BUFF_SIZE];

void Bluetooth_Recv_Start(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, rxbuff, RX_BUFF_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
}

void Bluetooth_Restart_IT(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
	HAL_UARTEx_ReceiveToIdle_DMA(huart, rxbuff, RX_BUFF_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
	Bluetooth_Recv_Data_Process(rxbuff, &Rx);
}

void Bluetooth_Recv_Data_Process(uint8_t *rxbuff, Rx_Frame *Rx)
{
	// 定义标识符,表示数据的转化进行到了数组的第几个成员
	uint8_t Pos = 1;
	if (rxbuff[0] == 0xA5 && rxbuff[RX_BUFF_SIZE - 1] == 0x5A)
	{
		// 开始数据转换
		for (int i = 0; i < RX_BYTE_NUM; i++)
		{
			Rx->Byte_data[i] = rxbuff[Pos];
			Pos++;
		}

		for (int i = 0; i < RX_SHORT_NUM; i++)
		{
			Rx->Short_data[i] = Char_to_Short(&rxbuff[Pos]);
			Pos += 2;
		}

		for (int i = 0; i < RX_INT_NUM; i++)
		{
			Rx->Int_data[i] = Char_to_Int(&rxbuff[Pos]);
			Pos += 4;
		}
		for (int i = 0; i < RX_FLOAT_NUM; i++)
		{
			Rx->Float_data[i] = Char_to_Float(&rxbuff[Pos]);
			Pos += 4;
		}
	}
}
