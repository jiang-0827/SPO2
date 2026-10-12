#ifndef _BSP_TIMER_H_
#define _BSP_TIMER_H_

#include "stm32f4xx.h"

void Timer2_Init(void);

extern volatile uint32_t g_tim2_5ms_flag;

#endif
