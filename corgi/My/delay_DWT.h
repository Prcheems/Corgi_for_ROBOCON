#ifndef __DELAY_DWT_H_
#define __DELAY_DWT_H_
#include "stdint.h"
#include "stm32h7xx.h"

/* 获取内核时钟频率 */
#define GET_CPU_ClkFreq() HAL_RCC_GetSysClockFreq()
#define SysClockFreq (168000000)

// 函数声明
uint32_t CPU_TS_TmrRd(void);
void DWT_Init(void);

// 最大延时值为8秒
void Delay_us(uint32_t us);
float Generate_Time_Points(void);
#define Delay_ms(ms) Delay_us(ms * 1000)

typedef struct Time
{
  char sign;                   // 判断是否是第一次进入该函数
  uint32_t Cycle_Times;        // 循环次数,一次循环是10s
  float Individual_Cycle_time; // 单次循环内记录的时间(s)
  float Now_Tick;              // 当前时间(ms)=循环次数 * 1000 + 单次循环内记录的时间
  uint32_t DWT_tick;           // DWTCNT的值
} Time_Param;
extern Time_Param Time_Points_Param;

#endif
