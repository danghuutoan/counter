#ifndef __TIMER_H__
#define __TIMER_H__
#include "stm32f10x.h"
#include <stdbool.h>
void counter_StartTimer(void);
void PL_Delay (u32 n100u);
void Delay_ms(uint16_t ms);
bool PL_DelayElapsed(u32 timestamp, u32 delay);
uint32_t timer_getick(void);
#endif /*__TIMER_H__ */

