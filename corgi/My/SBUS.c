/**
*@brief HotRC手柄的接收文件 SBUS.c
*@Usage main函数中写一句 SB_USART_Start(&huart2, &hdma_usart2_rx); 即可
                用不同的串口需要修改两个地方
                    1.main函数中的调用
                    2.本文件第43行
*@Channel 通道说明：总共8个通道
                    0，1通道对应右摇杆
                                                        0是x（横）方向，从左到右1800 -> 200
                                                        1是y（竖）方向，从下到上1800 -> 200
                    2，3通道对应左摇杆
                                                        2是y（竖）方向，从下到上1800 -> 200
                                                        3是x（横）方向，从左到右1800 -> 200
                    4，7通道对应SWA5和SWD8两个三状态开关，从下到上值分别为：1792，992，192
                    5，6通道对应SWB6和SWC7两个双状态开关，从下到上值分别为：1792，     192
                    中间四个按钮用于调整两个摇杆的初始位置对应的初始值
                    底下有四个开关可以控制通道0-3的正反，还有一个是混控开关，默认往上/在中间
                * 标志位failsafe_status：0表示连接正常，1表示连接断开
*/

#include "SBUS.h"
#include "string.h"

#define SBUS_START 0x0F // SBUS数据包头
#define SBUS_END 0x00   // SBUS数据包尾

uint8_t RxTemp_2[25] = {0};

// Initialize USART  初始化串口
void SB_USART_Start(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_usart_rx)
{
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)&RxTemp_2, 25);
    __HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
}

// 数据缓存和标志位
uint8_t failsafe_status = SBUS_SIGNAL_FAILSAFE;
int16_t g_sbus_channels[8] = {0};

// 手柄数据处理
void SBUS_Rcv_Prcs(uint8_t *data)
{
    if (data[0] == SBUS_START && data[24] == SBUS_END) // 数据包头尾正确则开始处理
    {
        g_sbus_channels[0] = ((RxTemp_2[1] | RxTemp_2[2] << 8) & 0x07FF);
        g_sbus_channels[1] = ((RxTemp_2[2] >> 3 | RxTemp_2[3] << 5) & 0x07FF);
        g_sbus_channels[2] = ((RxTemp_2[3] >> 6 | RxTemp_2[4] << 2 | RxTemp_2[5] << 10) & 0x07FF);
        g_sbus_channels[3] = ((RxTemp_2[5] >> 1 | RxTemp_2[6] << 7) & 0x07FF);
        g_sbus_channels[4] = ((RxTemp_2[6] >> 4 | RxTemp_2[7] << 4) & 0x07FF);
        g_sbus_channels[5] = ((RxTemp_2[7] >> 7 | RxTemp_2[8] << 1 | RxTemp_2[9] << 9) & 0x07FF);
        g_sbus_channels[6] = ((RxTemp_2[9] >> 2 | RxTemp_2[10] << 6) & 0x07FF);
        g_sbus_channels[7] = ((RxTemp_2[10] >> 5 | RxTemp_2[11] << 3) & 0x07FF);

        // 安全检测，检测是否失联或者数据错误
        // Security detection to check for lost connections or data errors
        failsafe_status = SBUS_SIGNAL_OK;
        if (RxTemp_2[23] & (1 << 2))
        {
            failsafe_status = SBUS_SIGNAL_LOST;

            // lost contact errors  遥控器失联错误
        }
        else if (RxTemp_2[23] & (1 << 3))
        {
            failsafe_status = SBUS_SIGNAL_FAILSAFE;

            // data loss error  数据丢失错误
        }
    }
}
