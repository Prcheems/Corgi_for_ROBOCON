#include "timer.h"
#include "tim.h"
#include "fsm.h"
#include "A1_motor.h"
#include "handkey.h"
#include "dog.h"
#include "delay_DWT.h"
#include "Bluetooth.h"
#include "my_uart.h"
#include "imu.h"

#define IS_USE_FREERTOS 0

#if IS_USE_FREERTOS
#include "freertos.h"
#include "task.h"
#endif

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern A1_Motor_Struct motor[8];

static float tick = 0;
static float tick_increment = 0;
float test_kp = 0;
float test_kw = 3.0f;

void TIM2_Start(void)
{
    TIM2->CNT = 0; // 确定是向上计数模式,清空计数器值
    HAL_TIM_Base_Start_IT(&htim2);
}

void TIM2_Stop(void)
{
    HAL_TIM_Base_Stop(&htim2);
    HAL_TIM_Base_Stop_IT(&htim2);
}

void TIM3_Start(void)
{
    TIM3->CNT = 0;
    HAL_TIM_Base_Start_IT(&htim3);
    tick_increment = (TIM3->ARR + 1) / 1000.0f; // ms
}

void TIM4_Start(void)
{
    TIM4->CNT = 0; // 确定是向上计数模式,清空计数器值
    HAL_TIM_Base_Start_IT(&htim4);
}

void TIM5_Start(void)
{
    TIM5->CNT = 0; // 确定是向上计数模式,清空计数器值
    HAL_TIM_Base_Start_IT(&htim5);
}

