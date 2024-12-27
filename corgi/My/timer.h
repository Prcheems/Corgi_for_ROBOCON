#ifndef TIMER_H_
#define TIMER_H_

#include "stdint.h"

void TIM2_Start(void);
void TIM2_Stop(void);
void TIM3_Start(void);
void TIM4_Start(void);
uint32_t GetTick(void);
void A1_All_Send(void);
void TIM5_Start(void);
#endif
