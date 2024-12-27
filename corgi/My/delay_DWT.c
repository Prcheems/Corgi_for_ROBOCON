#include "delay_DWT.h"
#include "math.h"

/*
**********************************************************************
*         ʱ�����ؼĴ�������
**********************************************************************
*/
/*
 ��Cortex-M������һ�������DWT(Data Watchpoint and Trace),
 ��������һ��32λ�ļĴ�����CYCCNT,����һ�����ϵļ�����,
 ��¼�����ں�ʱ�����еĸ���,��ܼ�¼��ʱ��Ϊ��
 10.74s=2��32�η�/400000000
 (�����ں�Ƶ��Ϊ400M,�ں���һ�ε�ʱ����Ϊ1/400M=2.5ns)
 ��CYCCNT���֮��,����0���¿�ʼ���ϼ�����
 ʹ��CYCCNT�����Ĳ������裺
 1����ʹ��DWT����,����������ں˵��ԼĴ���DEMCR��λ24����,д1ʹ��
 2��ʹ��CYCCNT�Ĵ���֮ǰ,����0
 3��ʹ��CYCCNT�Ĵ���,�����DWT_CTRL(�����Ϻ궨��ΪDWT_CR)��λ0����,д1ʹ��
 */

#define DWT_CR *(__IO uint32_t *)0xE0001000
#define DWT_CYCCNT *(__IO uint32_t *)0xE0001004
#define DEM_CR *(__IO uint32_t *)0xE000EDFC

#define DEM_CR_TRCENA (1 << 24)
#define DWT_CR_CYCCNTENA (1 << 0)

/**
 * @brief  ��ʼ��ʱ���
 * @param  ��
 * @retval ��
 * @note   ʹ����ʱ����ǰ,������ñ�����
 */
void DWT_Init(void)
{
  /* ʹ��DWT���� */
  DEM_CR |= (uint32_t)DEM_CR_TRCENA;

  /* DWT CYCCNT�Ĵ���������0 */
  DWT_CYCCNT = (uint32_t)0u;

  /* ʹ��Cortex-M DWT CYCCNT�Ĵ��� */
  DWT_CR |= (uint32_t)DWT_CR_CYCCNTENA;
}

/**
 * @brief  ��ȡ��ǰʱ���
 * @param  ��
 * @retval ��ǰʱ���,��DWT_CYCCNT�Ĵ�����ֵ
 */
uint32_t CPU_TS_TmrRd(void)
{
  return ((uint32_t)DWT_CYCCNT);
}

/**
 * @brief  ��ȡ��ǰʱ���
 * @param  ��
 * @retval ��ǰʱ���,��DWT_CYCCNT�Ĵ�����ֵ
 */
uint32_t HAL_GetTick(void)
{
  return ((uint32_t)DWT_CYCCNT / SysClockFreq * 1000);
}

/**
 * @brief  ����CPU���ڲ�����ʵ�־�ȷ��ʱ,32λ������
 * @param  us : �ӳٳ���,��λ1 us
 * @retval ��
 * @note   �����ʱֵΪ32��
 */
void Delay_us(uint32_t us)
{
  us = us - 2;
  DWT_Init();
  uint32_t ticks;
  uint32_t told, tnow, tcnt = 0;

  ticks = (us) * (GET_CPU_ClkFreq() / 1000000); /* ��Ҫ�Ľ����� */
  tcnt = 0;
  told = (uint32_t)CPU_TS_TmrRd(); /* �ս���ʱ�ļ�����ֵ */

  while (1)
  {
    tnow = (uint32_t)CPU_TS_TmrRd();
    if (tnow != told)
    {
      /* 32λ�������ǵ��������� */
      if (tnow > told)
      {
        tcnt += tnow - told;
      }
      /* ����װ�� */
      else
      {
        tcnt += UINT32_MAX - told + tnow;
      }
      told = tnow;
      /*ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳� */
      if (tcnt >= ticks)
        break;
    }
  }
}

Time_Param Time_Points_Param;
/**
 * @brief DWTԭ��ͳ��ϵͳʱ��,ע��,ÿ10s���ٱ������һ�θú�����
 * @return ��ǰʱ��(s)
 */
float Generate_Time_Points(void)
{
  if (Time_Points_Param.sign == 0)
  {
    Time_Points_Param.sign = 1;
    DWT_Init();
  }
  else if (Time_Points_Param.sign == 1)
  {
    Time_Points_Param.DWT_tick = CPU_TS_TmrRd();
    if (Time_Points_Param.DWT_tick >= 4000000000)
    {
      Time_Points_Param.sign = 0;
      Time_Points_Param.Cycle_Times++;
      Time_Points_Param.DWT_tick -= 4000000000;
    }
    Time_Points_Param.Individual_Cycle_time = (((float)Time_Points_Param.DWT_tick) / ((float)GET_CPU_ClkFreq()));
  }
  Time_Points_Param.Now_Tick = Time_Points_Param.Cycle_Times * 10.0f + Time_Points_Param.Individual_Cycle_time;
  return Time_Points_Param.Now_Tick;
}
float GetTick(void)
{
  return Generate_Time_Points();
}
