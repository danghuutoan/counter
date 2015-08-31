#include "led_7segs.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
//#define TOAN
#ifdef TOAN

#define LED7SEG_CLOCK_PIN			GPIO_Pin_1		// pin 11 of 595
#define LED7SEG_LATCH_PIN			GPIO_Pin_8		// pin 12
#define LED7SEG_DATA_PIN			GPIO_Pin_9		// pin 14


#define LED7SEG_PORT					GPIOB
#define LED7SEG_PORT_CLOCK		RCC_APB2Periph_GPIOB

#else

#define LED7SEG_CLOCK_PIN			GPIO_Pin_1		// pin 11 of 595
#define LED7SEG_LATCH_PIN			GPIO_Pin_2		// pin 12
#define LED7SEG_DATA_PIN			GPIO_Pin_3		// pin 14
#define LED7SEG_PORT					GPIOD
#define LED7SEG_PORT_CLOCK		RCC_APB2Periph_GPIOD

#endif

#define LED7SEG_DATA_HIGH					GPIO_SetBits(LED7SEG_PORT, LED7SEG_DATA_PIN);
#define LED7SEG_DATA_LOW					GPIO_ResetBits(LED7SEG_PORT, LED7SEG_DATA_PIN);
#define LED7SEG_CLOCK_HIGH				GPIO_SetBits(LED7SEG_PORT, LED7SEG_CLOCK_PIN);
#define LED7SEG_CLOCK_LOW					GPIO_ResetBits(LED7SEG_PORT, LED7SEG_CLOCK_PIN);
#define LED7SEG_LATCH							GPIO_SetBits(LED7SEG_PORT, LED7SEG_LATCH_PIN); 		\
																	delay_count(100);									\
																	GPIO_ResetBits(LED7SEG_PORT, LED7SEG_LATCH_PIN);

const uint8_t value_to_led7seg_table[10] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f};
static void delay_count(uint32_t count)
{
	volatile uint32_t i = count;
	while(i > 0)
	{
		i--;
	}
}
static void castLedValue(uint8_t value, bool blank)
{
	uint8_t i;
	uint8_t val_to_transmit;
	val_to_transmit = value_to_led7seg_table[value];
	if( blank == true)
	{
		for(i = 0; i < 8; i++)
		{
			LED7SEG_DATA_HIGH;
			delay_count(50);
			LED7SEG_CLOCK_HIGH;
			delay_count(50);
			LED7SEG_CLOCK_LOW;
		}			
	}
	else if( value <= 9 )
	{
		for(i = 0; i < 8; i++)
		{
			if((val_to_transmit<<i) & 0x80) 
			{
				LED7SEG_DATA_LOW;
			}
			else 
			{
				LED7SEG_DATA_HIGH;
			}
			delay_count(50);
			LED7SEG_CLOCK_HIGH;
			delay_count(50);
			LED7SEG_CLOCK_LOW;
		}		
	}
	else
	{
		return;
	}
}
void Init_Led7Seg(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(LED7SEG_PORT_CLOCK, ENABLE);
 
  
  GPIO_InitStructure.GPIO_Pin = LED7SEG_CLOCK_PIN | LED7SEG_DATA_PIN | LED7SEG_LATCH_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LED7SEG_PORT, &GPIO_InitStructure);
	
}

int Led7Seg_PrintNum(uint8_t value)
{
	uint8_t val1, val2;
	if( value > 99)
	{
		return -1;
	}
	else
	{
		/* calculate units digit */
		val1 = value % 10;
		/* calculate tens digit */
		val2 = value / 10;
		
		/* shift data out for units digit */
		castLedValue(val1, false);	
		/* shift data out for tens digit */
		if(val2 == 0)
		{
			castLedValue(0, true);
		}
		else
		{
			castLedValue(val2, false);
		}
		
		/* latch data for display */
		LED7SEG_LATCH;
		return 1;
	}
}
int main_TestLed7Seg(void)
{
	uint8_t i;
	Init_Led7Seg();
	while(1)
	{
		for( i = 0; i < 100; i++)
		{
			Led7Seg_PrintNum(i);
			delay_count(10000000);
		}
	}	
}
