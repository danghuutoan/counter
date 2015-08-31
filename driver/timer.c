#include "stm32f10x.h"
#include <stdio.h>
#include "std_type.h"
static uint32_t         Counter100uS;

#define _timestamp        Counter100uS

uint32_t timer_getick(void)
	{	uint32_t l_tick = _timestamp;
		return l_tick;
	}
/*******************************************************************************
* Function Name  : PL_DifferenceCounter
* Description    : Calculate the difference between two counter variables
* Input          : Start time, end time
* Output         : None
* Return         : Calculated difference
*******************************************************************************/
static u32 PL_DifferenceCounter(u32 counterX100us, u32 counterY100us)
{
 u32 difference;
 
 if (counterX100us > counterY100us) 
   difference = 600000 - counterX100us + counterY100us + 1;
 else	
   difference = counterY100us - counterX100us;
 
 return (difference);
}


/*******************************************************************************
* Function Name  : PL_DelayElapsed
* Description    : Verify if a specified delay is elapsed 
* Input          : Timestamp, set delay
* Output         : None
* Return         : TRUE (daly elapsed), FALSE (delay not elapsed)
*******************************************************************************/
bool PL_DelayElapsed(u32 timestamp, u32 delay)
{
  if (PL_DifferenceCounter(timestamp, Counter100uS) >= delay)
    return TRUE;
 
 return FALSE;
}


/*******************************************************************************
* Function Name  : PL_Delay
* Description    : Static Delay
* Input          : Request delay (1 unit = 100us)
* Output         : None
* Return         : None
*******************************************************************************/
void PL_Delay (u32 n100u)
{
  u32 tTime;
  
  tTime = _timestamp;
  while(PL_DelayElapsed(tTime, n100u) == FALSE);
}

void counter_StartTimer(void)
{
	 SysTick_Config(SystemCoreClock / 10000); // 100 us
   NVIC_SetPriority(SysTick_IRQn, 1); 
}


void Delay_ms(uint16_t ms)
{
  uint32_t del_us = ms * 10;
  PL_Delay(del_us);
}


void SysTick_Handler (void)
{
	 Counter100uS++;  
  if (Counter100uS > 600000)
    Counter100uS = 0;
}
