#include "imu.h"
#include "stdint.h"
#include "stm32h7xx.h"
#include "usart.h"
#include "string.h"
#include "dog.h"

/* 接收数据并使用0x91数据包结构定义来解释数据 */
__align(4) id0x91_t dat; /* struct must be 4 byte aligned */
float imu_angle[3];      /* eular angles:R/P/Y */

static float ABS(float a)
{
    return a > 0 ? a : -a;
}

// crc校验
static void crc16_update(uint16_t *currect_crc, const uint8_t *src, uint32_t len)
{
    uint32_t crc = *currect_crc;
    uint32_t j;
    for (j = 0; j < len; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    }
    *currect_crc = crc;
}

void imu_data_process(uint8_t *receive)
{
    uint16_t CRCReceived = 0;   /* CRC value received from a frame */
    uint16_t CRCCalculated = 0; /* CRC value caluated from a frame */
    uint8_t payload_len = 0;
    static float imu_Z_last = 0; // 上一时刻z角度,用于累计出z轴旋转的总角度
    float imu_Z_now = 0;
    float imu_Z_temp_1 = 0;
    float imu_Z_temp_2 = 0;
    float imu_Z_temp = 0;
    if (receive[0] == 0x5A && receive[1] == 0xA5) // 帧头
    {
        /* CRC */
        CRCReceived = receive[4] + (receive[5] << 8);
        payload_len = receive[2] + (receive[3] << 8);
        /* calculate CRC */
        crc16_update(&CRCCalculated, receive, 4);
        crc16_update(&CRCCalculated, receive + 6, payload_len);
        /* CRC match */
        if (CRCCalculated == CRCReceived)
        {
            memcpy(&dat, &receive[6], sizeof(id0x91_t));
            /* 计算Z轴累加旋转角度 */
            imu_Z_now = dat.eul[2];
            if (imu_Z_last <= imu_Z_now)
            {
                imu_Z_temp_1 = imu_Z_now - imu_Z_last;
                imu_Z_temp_2 = imu_Z_now - imu_Z_last - 360;
            }
            else
            {
                imu_Z_temp_1 = imu_Z_now - imu_Z_last;
                imu_Z_temp_2 = imu_Z_now - imu_Z_last + 360;
            }
            imu_Z_temp = (ABS(imu_Z_temp_1)) < (ABS(imu_Z_temp_2)) ? imu_Z_temp_1 : imu_Z_temp_2;
            imu_Z_last = imu_Z_now;

            imu_angle[0] = dat.eul[0];
            imu_angle[1] = dat.eul[1];
            imu_angle[2] = imu_angle[2] + imu_Z_temp;
        }
    }
}

float get_imu_angle(IMU x)
{
    switch (x)
    {
    case Pitch:
    {
        return imu_angle[1];
    }
    case Roll:
    {
        return imu_angle[0];
    }
    case Yaw:
    {
        return imu_angle[2];
    }
    default:
        return -1;
    }
}

void Euler_Update(Euler *euler)
{
    euler->Last_Yaw = euler->Yaw;
    euler->Yaw = get_imu_angle(Yaw) - euler->Update_Yaw; /*- euler->Update_Yaw*/

    euler->Roll = get_imu_angle(Pitch);

    float Pitch_Temp = get_imu_angle(Roll);
    if ((Pitch_Temp <= -170.0f) && (euler->Last_Pitch >= 170.0f))
        euler->cnt_Pitch++;
    if ((Pitch_Temp >= 170.0f) && (euler->Last_Pitch <= -170.0f))
        euler->cnt_Pitch--;
    euler->Last_Pitch = Pitch_Temp;
    euler->Pitch = Pitch_Temp + 360.0f * euler->cnt_Pitch;
}

void Yaw_Reset(Euler *euler)
{
    euler->Update_Yaw = get_imu_angle(Yaw);
}
