#ifndef _IMU_H
#define _IMU_H

#include "stdint.h"
#include "dog.h"

__packed typedef struct
{
    uint8_t tag; /* 0x91 */
    uint8_t id;
    uint8_t rev[6]; /* reserved */
    uint32_t ts;    /* timestamp */
    float acc[3];
    float gyr[3];
    float mag[3];
    float eul[3];  /* eular angles:R/P/Y */
    float quat[4]; /* quaternion */

} id0x91_t;

typedef enum
{
    Roll = 0,
    Pitch,
    Yaw
} IMU;

void imu_data_process(uint8_t *receive);
float get_imu_angle(IMU x);
void Euler_Update(Euler *euler);
void Yaw_Reset(Euler *euler);

#endif
