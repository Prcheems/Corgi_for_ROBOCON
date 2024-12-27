#include "A1_motor.h"
#include "stdint.h"
#include "stm32h7xx.h"
#include "usart.h"
#include "stdint.h"
#include "main.h"
#include "my_uart.h"
#include "delay_dwt.h"

#define reduction_ratio 9.1f // 减速比

MOTOR_send motor_s;
MOTOR_recv motor_recv[3];
A1_Motor_Struct motor[8];

float test_t = 0;
// 接收缓存区
uint8_t RS485_RX_BUF[78]; // 接收缓冲,最大64个字节.
// 接收到的数据长度
uint8_t RS485_RX_CNT = 0;

uint8_t res = 0;

enum motor_command_type
{
    TORQUE,
    POSITION,
    VELOCITY
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t bits = 0;
    uint32_t i = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;

    for (i = 0; i < len; i++)
    {
        xbit = (uint32_t)1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

int modify_data(MOTOR_send *motor)
{
    motor->motor_send_data.head.start[0] = 0xFE;
    motor->motor_send_data.head.start[1] = 0xEE;
    motor->motor_send_data.head.motorID = motor->id;

    motor->motor_send_data.Mdata.mode = motor->mode;

    motor->motor_send_data.Mdata.ModifyBit = 0xff;
    // motor->motor_send_data.Mdata.ReadBit = 0x01;

    motor->motor_send_data.Mdata.T = (int16_t)((motor->T) * 256);
    motor->motor_send_data.Mdata.W = (int16_t)((motor->W) * 128);
    motor->motor_send_data.Mdata.Pos = (motor->Pos) * 16384 / 6.2832f;
    motor->motor_send_data.Mdata.K_P = (int16_t)((motor->K_P) * 2048);
    motor->motor_send_data.Mdata.K_W = (int16_t)((motor->K_W) * 1024);

    motor->motor_send_data.CRCdata.u32 = crc32_core((uint32_t *)motor, 7);

    motor->hex_len = 34;

    return 0;
}

void A1_Motor_Send_Data(A1_Motor_Struct *p_send_data, UART_HandleTypeDef *huart)
{
    motor_s.id = p_send_data->id;                            // motor ID
    motor_s.mode = p_send_data->mode;                        // switch to servo mode
    motor_s.T = p_send_data->target.T / reduction_ratio;     // Nm, T<255.9
    motor_s.W = p_send_data->target.W * reduction_ratio;     // rad/s, W<511.9
    motor_s.Pos = p_send_data->target.Pos * reduction_ratio; // rad, Pos<131071.9
    motor_s.K_P = p_send_data->K_p;                          // K_P<31.9     value should around 0.1
    motor_s.K_W = p_send_data->K_w;                          // K_W<63.9     value should around 3

    modify_data(&motor_s);
    HAL_UART_Transmit(huart, (uint8_t *)&motor_s, 34, 0xFFF);
}

// 勿动,这里的id不要改
void MotorInit() // mode, T默认为0
{
    float kp, kw, id;
    for (uint8_t i = 0; i < 8; i++)
    {
        switch (i)
        {
        case 0: // motor[0]
        {
            id = 0;
            kp = 0;
            kw = 0;
            break;
        }
        case 1: // motor[1]
        {
            id = 1;
            kp = 0;
            kw = 0;
            break;
        }
        case 2: // motor[2]
        {
            id = 0;
            kp = 0;
            kw = 0;
            break;
        }
        case 3: // motor[3]
        {
            id = 1;
            kp = 0;
            kw = 0;
            break;
        }
        case 4: // motor[4]
        {
            id = 0;
            kp = 0;
            kw = 0;
            break;
        }
        case 5: // motor[5]
        {
            id = 1;
            kp = 0;
            kw = 0;
            break;
        }
        case 6: // motor[6]
        {
            id = 0;
            kp = 0;
            kw = 0;
            break;
        }
        case 7: // motor[7]
        {
            id = 1;
            kp = 0;
            kw = 0;
            break;
        }
        }
        motor[i].id = id;
        motor[i].K_p = kp;
        motor[i].K_w = kw;
    }
}
// 0 为停转, 5 为开环缓慢转动, 10 为闭环伺服控制
void SetAllMotorMode(uint8_t x)
{
    for (int i = 0; i < 8; i++)
    {
        motor[i].mode = x;
    }
}
// 设置所有电机的KpKw
void Set_All_Motor_Param(Pos_motor *param)
{
    for (int i = 0; i < 8; i++)
    {
//        motor[i].target.T = param->Tfb;
        motor[i].K_p = param->kp;
        motor[i].K_w = param->kw;
    }
}

// 设置单个腿的两个电机的kp与kw
void Set_Motor_Param(uint8_t legid, Pos_motor *param)
{
//    motor[legid * 2].target.T = param->Tfb;
    motor[legid * 2].K_p = param->kp;
    motor[legid * 2].K_w = param->kw;

//    motor[legid * 2 + 1].target.T = param->Tfb;
    motor[legid * 2 + 1].K_p = param->kp;
    motor[legid * 2 + 1].K_w = param->kw;
}

// 等待接收到所有电机回传数据
void motor_waiting()
{
    while (motor[0].measure.Pos == 0 || motor[1].measure.Pos == 0 || motor[2].measure.Pos == 0 || motor[3].measure.Pos == 0 || motor[4].measure.Pos == 0 || motor[5].measure.Pos == 0 || motor[6].measure.Pos == 0 || motor[7].measure.Pos == 0)
    {
        Delay_ms(2);
    }
}
