#ifndef __DELAY_DWT_H_
#define __DELAY_DWT_H_
#include "stdint.h"
#include "stm32h7xx.h"

/* ��ȡ�ں�ʱ��Ƶ�� */
#define GET_CPU_ClkFreq() HAL_RCC_GetSysClockFreq()
#define SysClockFreq (168000000)

// ��������
uint32_t CPU_TS_TmrRd(void);
void DWT_Init(void);

// �����ʱֵΪ8��
void Delay_us(uint32_t us);
float Generate_Time_Points(void);
#define Delay_ms(ms) Delay_us(ms * 1000)

typedef struct Time
{
  char sign;                   // �ж��Ƿ��ǵ�һ�ν���ú���
  uint32_t Cycle_Times;        // ѭ������,һ��ѭ����10s
  float Individual_Cycle_time; // ����ѭ���ڼ�¼��ʱ��(s)
  float Now_Tick;              // ��ǰʱ��(ms)=ѭ������ * 1000 + ����ѭ���ڼ�¼��ʱ��
  uint32_t DWT_tick;           // DWTCNT��ֵ
} Time_Param;
extern Time_Param Time_Points_Param;

#endif
