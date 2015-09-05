#include "stm32f10x.h"
#include "hal_com.h"
#include "diskio.h"
#include "log.h"
#include <stdio.h>
#include "ff.h" 
#include "timer.h"
#include "counter.h"
#include "rtc.h"
#include <stdlib.h>
#include <math.h>
#include "led_7segs.h"
#include "hal_dio.h"
#include "switch.h"
#include "stm32f10x_flash.h"
#define  REALEASE_SRC
#define  PORTB_IN						&sw_process_data[0]
#define  PORTA_IN						&sw_process_data[1]
#define  PORTE_IN						&sw_process_data[2]
#define  PORTC_IN           					&sw_process_data[3]
#define  PORTD_IN           					&sw_process_data[4]
#define FLASH_PAGE_SIZE    ((uint16_t)0x400)
#define MAX_DEVICE					30
#define StartAddr  						0x08010400
#define EndAddr    ((uint32_t)0x0800C804)

#ifdef REALEASE_SRC
#define SW_NUM 			SWITCH_MAX

#define SW_DATABASE(SWITCH,port,pin)  \
{																			\
	.l_switch = &l_##SWITCH , 	  			\
	.process_data = port  , 				\
	.GPIO_Pin     = pin ,   				\
	.led          = &led_##SWITCH,				\
 }
 
#define __MACRO_NUM(x)								_##x

#define __SW_INIT(switch_id)					sw_init(&l_switch##switch_id,SWITCH##switch_id)
 /* this number must be an odd number to make sure that led status is on after blinking*/
#define LED_RESET_BLINK_TIME    		  13
void switch_callback(void)
{
	printf("a\r\n");
}

typedef struct _sw_database
{
  switch_t  *l_switch;
	uint16_t	*process_data;
	uint16_t   GPIO_Pin;
	hal_dio_t	*led;
} sw_database_t;
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

typedef enum {MODE_NORMAL,MODE_CONFIG} sys_mode_t;

 typedef struct
{
	sys_mode_t mode;
	uint8_t dev_num;
	uint32_t sw_status;
} dev_manager_t;

volatile dev_manager_t manager = {
	.mode = MODE_NORMAL,
	.dev_num = 30,
	.sw_status = 0
};

switch_t l_switch_1;
switch_t l_switch_2; 
switch_t l_switch_3; 
switch_t l_switch_4; 
switch_t l_switch_5;
switch_t l_switch_6;
switch_t l_switch_7;
switch_t l_switch_8;
switch_t l_switch_9;
switch_t l_switch_10;
switch_t l_switch_11;
switch_t l_switch_12; 
switch_t l_switch_13; 
switch_t l_switch_14; 
switch_t l_switch_15;
switch_t l_switch_16;
switch_t l_switch_17;
switch_t l_switch_18;
switch_t l_switch_19;
switch_t l_switch_20;
switch_t l_switch_21;
switch_t l_switch_22; 
switch_t l_switch_23; 
switch_t l_switch_24; 
switch_t l_switch_25;
switch_t l_switch_26;
switch_t l_switch_27;
switch_t l_switch_28;
switch_t l_switch_29;
switch_t l_switch_30;
switch_t l_switch_RESET;
switch_t l_switch_MODE;
switch_t l_switch_UP;
switch_t l_switch_DOWN;

hal_dio_t led_switch_1;
hal_dio_t led_switch_2;
hal_dio_t led_switch_3;
hal_dio_t led_switch_4;
hal_dio_t led_switch_5;
hal_dio_t led_switch_6;
hal_dio_t led_switch_7;
hal_dio_t led_switch_8;
hal_dio_t led_switch_9;
hal_dio_t led_switch_10;

hal_dio_t led_switch_11;
hal_dio_t led_switch_12;
hal_dio_t led_switch_13;
hal_dio_t led_switch_14;
hal_dio_t led_switch_15;
hal_dio_t led_switch_16;
hal_dio_t led_switch_17;
hal_dio_t led_switch_18;
hal_dio_t led_switch_19;
hal_dio_t led_switch_20;
hal_dio_t led_switch_21;
hal_dio_t led_switch_22;
hal_dio_t led_switch_23;
hal_dio_t led_switch_24;
hal_dio_t led_switch_25;
hal_dio_t led_switch_26;
hal_dio_t led_switch_27;
hal_dio_t led_switch_28;
hal_dio_t led_switch_29;
hal_dio_t led_switch_30;
hal_dio_t led_switch_RESET;
hal_dio_t led_switch_MODE;
hal_dio_t led_switch_UP;
hal_dio_t led_switch_DOWN;
hal_dio_t buzzer;
static uint16_t sw_process_data[5];
uint8_t l_sw_count_u8 = 0;
uint32_t bk_data;
uint32_t l_time_now, l_task_flash_tick;
static uint8_t l_led_blink = 0;
static uint32_t l_led_time_now;


static const sw_database_t data_table [] =
{
	/* switch_1 */
	SW_DATABASE(switch_1 ,PORTE_IN,GPIO_Pin_2),
		/* switch_2 */
	SW_DATABASE(switch_2 ,PORTE_IN,GPIO_Pin_3),
		/* switch_3 */
	SW_DATABASE(switch_3 ,PORTE_IN,GPIO_Pin_4),
		/* switch_4 */
	SW_DATABASE(switch_4 ,PORTE_IN,GPIO_Pin_5),
		/* switch_5 */
	SW_DATABASE(switch_5 ,PORTE_IN,GPIO_Pin_6),
		/* switch_6 */
	SW_DATABASE(switch_6 ,PORTC_IN,GPIO_Pin_13),
		/* switch_7 */
	SW_DATABASE(switch_7 ,PORTC_IN,GPIO_Pin_14),
		/* switch_8 */
	SW_DATABASE(switch_8 ,PORTC_IN,GPIO_Pin_15),
		/* switch_9 */
	SW_DATABASE(switch_9 ,PORTC_IN,GPIO_Pin_0),
		/* switch_10 */
	SW_DATABASE(switch_10 ,PORTC_IN,GPIO_Pin_1),
	/* switch_11 */
	SW_DATABASE(switch_11 ,PORTC_IN,GPIO_Pin_2),
		/* switch_12 */
	SW_DATABASE(switch_12 ,PORTC_IN,GPIO_Pin_3),
		/* switch_13 */
	SW_DATABASE(switch_13 ,PORTA_IN,GPIO_Pin_0),
		/* switch_14 */
	SW_DATABASE(switch_14 ,PORTA_IN,GPIO_Pin_1),
		/* switch_15 */
	SW_DATABASE(switch_15 ,PORTA_IN,GPIO_Pin_2),
		/* switch_16 */
	SW_DATABASE(switch_16 ,PORTA_IN,GPIO_Pin_3),
		/* switch_17 */
	SW_DATABASE(switch_17 ,PORTA_IN,GPIO_Pin_4),
		/* switch_18 */
	SW_DATABASE(switch_18 ,PORTA_IN,GPIO_Pin_5),
		/* switch_19 */
	SW_DATABASE(switch_19 ,PORTA_IN,GPIO_Pin_6),
		/* switch_20 */
	SW_DATABASE(switch_20 ,PORTA_IN,GPIO_Pin_7),	
	/* switch_21 */
	SW_DATABASE(switch_21 ,PORTC_IN,GPIO_Pin_4),
		/* switch_22 */
	SW_DATABASE(switch_22 ,PORTC_IN,GPIO_Pin_5),
		/* switch_23 */
	SW_DATABASE(switch_23 ,PORTB_IN,GPIO_Pin_0),
		/* switch_24 */
	SW_DATABASE(switch_24 ,PORTB_IN,GPIO_Pin_1),
		/* switch_25 */
	SW_DATABASE(switch_25 ,PORTE_IN,GPIO_Pin_7),
		/* switch_26 */
	SW_DATABASE(switch_26 ,PORTE_IN,GPIO_Pin_8),
		/* switch_27 */
	SW_DATABASE(switch_27 ,PORTE_IN,GPIO_Pin_9),
		/* switch_28 */
	SW_DATABASE(switch_28 ,PORTE_IN,GPIO_Pin_10),
		/* switch_29 */
	SW_DATABASE(switch_29 ,PORTE_IN,GPIO_Pin_11),
		/* switch_30 */
	SW_DATABASE(switch_30 ,PORTE_IN,GPIO_Pin_12),
		/* switch_30 */
	SW_DATABASE(switch_RESET ,PORTB_IN,GPIO_Pin_6),
		/* switch_MODE */
	SW_DATABASE(switch_MODE ,PORTD_IN,GPIO_Pin_5),
		/* switch_UP */
	SW_DATABASE(switch_UP ,PORTD_IN,GPIO_Pin_4),
		/* switch_DOWN */
	SW_DATABASE(switch_DOWN ,PORTD_IN,GPIO_Pin_6),
};

void FlashWrite(uint32_t address , uint16_t* data, uint8_t len);
void led_init(hal_dio_t* l_led, hal_dio_chanel_t l_id)
{
	l_led->chid  = l_id;
    	l_led->dir   = HAL_DIO_OUT;
	l_led->state = HAL_DIO_LOW;
    hal_dio_init(l_led);
}


void sw_init(switch_t* l_switch , switch_name_t l_sw_name)
{
	l_switch->name = l_sw_name;
	l_switch->trigger_type = EXTI_Trigger_Falling;
	l_switch->callback = switch_callback; 
	switch_init(l_switch);
	
	l_switch->last_data = switch_read(l_switch);
	l_switch->new_data  = l_switch->last_data;
	l_switch->pressed   = false;
		
	if(l_sw_name< SWITCH_RESET)
	{
		l_switch->led       = data_table[l_sw_name].led;
		led_init(l_switch->led,(hal_dio_chanel_t)l_switch->name);
	}					
}	


void sw_update_status (void)
{
	uint8_t l_sw_idx_u8 = 0;
	for( l_sw_idx_u8 = 0 ; l_sw_idx_u8 < SW_NUM ; l_sw_idx_u8 ++)
	{
			uint16_t l_data_u16 = *(data_table[l_sw_idx_u8].process_data )&(data_table[l_sw_idx_u8].GPIO_Pin);
		__disable_irq();
		if(l_data_u16)
		{
			data_table[l_sw_idx_u8].l_switch->new_data = true;
		}
		else
		{		
			data_table[l_sw_idx_u8].l_switch->new_data = false;
		}
		
		if((data_table[l_sw_idx_u8].l_switch->last_data == true ) && (data_table[l_sw_idx_u8].l_switch->new_data == false))
		{
			if(data_table[l_sw_idx_u8].l_switch->pressed == false)
			{				
				if(l_sw_idx_u8 < SWITCH_RESET)
				{
					if(l_sw_idx_u8 < manager.dev_num)
					{
						if(manager.mode == MODE_NORMAL)
						{
							data_table[l_sw_idx_u8].l_switch->pressed = true;
							l_sw_count_u8++;
							if(l_sw_count_u8 >= manager.dev_num)
							{
								l_sw_count_u8 = manager.dev_num;
							}
						}
						else
						{
							/* do nothing*/
						}
					}	
				}
				else
				{
					switch(l_sw_idx_u8)
						{
							case SWITCH_RESET:
								if(l_sw_count_u8 >= manager.dev_num)
							  	data_table[l_sw_idx_u8].l_switch->pressed = true;
							break;
							case SWITCH_MODE:
								data_table[l_sw_idx_u8].l_switch->pressed = true;
							break;
							case SWITCH_UP:
								data_table[l_sw_idx_u8].l_switch->pressed = true;
							break;
							case SWITCH_DOWN:
								data_table[l_sw_idx_u8].l_switch->pressed = true;
							break;
						}
				}
			}
		}
		else
		{
			/*do nothing*/
		}
		data_table[l_sw_idx_u8].l_switch->last_data = data_table[l_sw_idx_u8].l_switch->new_data;
		__enable_irq();
	}
}
void bkp_init (void)
{
	/* Enable PWR and BKP clocks */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
  	/* Allow access to BKP Domain */
  	PWR_BackupAccessCmd(ENABLE);
  	BKP_ClearFlag();
}





void FlashWrite(uint32_t address , uint16_t* data, uint8_t len)
{
	volatile FLASH_Status FLASHStatus = FLASH_BUSY ;
	/* Unlock the Flash Program Erase controller */
	FLASH_Unlock();
	
 
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

	/* Erase the FLASH pages */
	__disable_irq();
	FLASHStatus = FLASH_BUSY;
	while(FLASHStatus == FLASH_BUSY)
	FLASHStatus = FLASH_ErasePage(address);
	FLASHStatus = FLASH_COMPLETE;
	for(int i =0 ; (i< len)&&(FLASHStatus == FLASH_COMPLETE) ; i++)
	{
		FLASHStatus = FLASH_ProgramHalfWord(address, *data);
		data = data+1;
		address = address +2;
	}
		__enable_irq();
		FLASH_Lock();
}

int main (void)
{   

	__SW_INIT(_1);
	__SW_INIT(_2);
	__SW_INIT(_3);
	__SW_INIT(_4);
	__SW_INIT(_5);
	__SW_INIT(_6);
	__SW_INIT(_7);
	__SW_INIT(_8);
	__SW_INIT(_9);
	__SW_INIT(_10);
	__SW_INIT(_11);
	__SW_INIT(_12);
	__SW_INIT(_13);
	__SW_INIT(_14);
	__SW_INIT(_15);
	__SW_INIT(_16);
	__SW_INIT(_17);
	__SW_INIT(_18);
	__SW_INIT(_19);
	__SW_INIT(_20);
	__SW_INIT(_21);
	__SW_INIT(_22);
	__SW_INIT(_23);
	__SW_INIT(_24);
	__SW_INIT(_25);
	__SW_INIT(_26);
	__SW_INIT(_27);
	__SW_INIT(_28);
	__SW_INIT(_29);
	__SW_INIT(_30);
	__SW_INIT(_RESET);
	__SW_INIT(_MODE);
	__SW_INIT(_UP);
	__SW_INIT(_DOWN);
	led_init(&buzzer,HAL_DIO_CH32);
	bkp_init();
		
	counter_StartTimer();
	Init_Led7Seg();
		
	__disable_irq();

	
	l_sw_count_u8   =  BKP_ReadBackupRegister(BKP_DR1);
	manager.dev_num =  BKP_ReadBackupRegister(BKP_DR2);
	manager.sw_status =  (BKP_ReadBackupRegister(BKP_DR3)<<16) + BKP_ReadBackupRegister(BKP_DR4);
	
	if(manager.dev_num >=  MAX_DEVICE)
	{
		manager.dev_num = MAX_DEVICE;
	}
	if(l_sw_count_u8 >= manager.dev_num)
	{
		l_sw_count_u8 = manager.dev_num;
	}
	/* print out the last data to LED 7 segments*/
	if(manager.mode == MODE_NORMAL)
	{
		Led7Seg_PrintNum(l_sw_count_u8);
	}
	else
	{
		Led7Seg_PrintNum(manager.dev_num);
	}
	uint8_t l_sw_index_u8 = 0;
		for(l_sw_index_u8 = 0; l_sw_index_u8 < SWITCH_RESET; l_sw_index_u8++)
		{
			if((manager.sw_status &(1<<l_sw_index_u8))!=0)
			{
				/* these led gonna be reset to pressed false status when reset button is pressesed */
				hal_dio_set_high(data_table[l_sw_index_u8].led);
				data_table[l_sw_index_u8].l_switch->pressed = true;
			}
		}
				
	/* init timer , start to count */
	__enable_irq();
	timer_init();
	while(1)
	{
		/* update switch status*/
		sw_update_status();	
			
		/* store data from cache into flash every second */
		if(PL_DelayElapsed(l_task_flash_tick,1000))
		{
			/* only write when backup data is different from update data
			 * beacause sw_status changed equivalent to count data is changed
			 * so we don't have to check sw_status is change or not 
			 */

			  BKP_WriteBackupRegister(BKP_DR1,(uint16_t) l_sw_count_u8);
			  BKP_WriteBackupRegister(BKP_DR2,(uint16_t) manager.dev_num);
			  BKP_WriteBackupRegister(BKP_DR3,(uint16_t) (manager.sw_status>>16));
			  BKP_WriteBackupRegister(BKP_DR4,(uint16_t) manager.sw_status);
				l_task_flash_tick = timer_getick();
		}
		
		/* change buzzer freq here*/
		/*if count >= 30 turn on th buzzer*/
		if(l_sw_count_u8 >= manager.dev_num)
		{
			if(PL_DelayElapsed(l_time_now,7000))// change buzzer time 
			{
				hal_dio_toggle(&buzzer);
				l_time_now = timer_getick();
			}
			
			if(PL_DelayElapsed(l_led_time_now,8000))
			{
				if(l_led_blink < LED_RESET_BLINK_TIME)
				{
					uint8_t l_led_blink_id;
					for( l_led_blink_id = 0;l_led_blink_id <  manager.dev_num; l_led_blink_id++ )
					{
						if(l_led_blink %2)
						{
							hal_dio_set_low(data_table[l_led_blink_id].led);
						}
						else
						{
							hal_dio_set_high(data_table[l_led_blink_id].led);
						}
					}
					
					l_led_blink++;
					if(l_led_blink >= LED_RESET_BLINK_TIME)
					{
							l_led_blink = LED_RESET_BLINK_TIME;
					}
				}
				else
				{
				}
					
				l_led_time_now = timer_getick();
			}
		}
			
		/* scan switch status and turn on the according led */
		uint8_t l_sw_index_u8 = 0;
		for(l_sw_index_u8 = 0; l_sw_index_u8 < SW_NUM; l_sw_index_u8++)
		{
			if(l_sw_index_u8 < SWITCH_RESET)
			{
				if((manager.mode == MODE_NORMAL)&&(l_sw_count_u8< manager.dev_num ))
				{
					if(data_table[l_sw_index_u8].l_switch->pressed == true)
					{
						/* these led gonna be reset to pressed false status when reset button is pressesed */
						hal_dio_set_high(data_table[l_sw_index_u8].led);
						/* store data into cache */
						manager.sw_status |= (1<<l_sw_index_u8);
						Led7Seg_PrintNum(l_sw_count_u8);
					}
				}
			}
			else
			{
				switch(l_sw_index_u8)
				{
					case SWITCH_RESET:
					/* the program gonna jump into here if reset switch is on*/
					if(data_table[l_sw_index_u8].l_switch->pressed == true)
					{
						data_table[l_sw_index_u8].l_switch->pressed = false;
						l_sw_count_u8 = 0;
						l_led_blink = 0;
						Led7Seg_PrintNum(l_sw_count_u8);
						uint8_t l_switch_reset;
						for(l_switch_reset = 0;((l_switch_reset< SWITCH_RESET)&&(l_switch_reset < manager.dev_num));l_switch_reset++)
						{
							if(data_table[l_switch_reset].l_switch->pressed == true)
							{
								hal_dio_set_low(data_table[l_switch_reset].led);
								manager.sw_status = 0;
								data_table[l_switch_reset].l_switch->pressed = false;
							}
							else 
							{
								/*do nothing*/
							}
						}
						/* turn off the buzzer*/
						hal_dio_set_low(&buzzer);
					}
					break;
							
					case SWITCH_MODE:
						if(data_table[l_sw_index_u8].l_switch->pressed == true)
						{
							data_table[l_sw_index_u8].l_switch->pressed = false;
							if(manager.mode == MODE_CONFIG)
							{
								manager.mode = MODE_NORMAL;
								/* blinking led to notify */
								Led7Seg_PrintNum(88);
								Delay_ms(500);
								Led7Seg_PrintNum(100);
								Delay_ms(500);
								Led7Seg_PrintNum(88);
								Delay_ms(500);
								Led7Seg_PrintNum(100);
								Delay_ms(500);
								Led7Seg_PrintNum(l_sw_count_u8);
							}
							else
							{
								manager.mode = MODE_CONFIG;
								/* blinking led to notify */
								Led7Seg_PrintNum(88);
								Delay_ms(500);
								Led7Seg_PrintNum(100);
								Delay_ms(500);
								Led7Seg_PrintNum(88);
								Delay_ms(500);
								Led7Seg_PrintNum(100);
								Delay_ms(500);
								
								Led7Seg_PrintNum(manager.dev_num);
							}
						}
					break;
					case SWITCH_UP:
						if(data_table[l_sw_index_u8].l_switch->pressed == true)
						{
							data_table[l_sw_index_u8].l_switch->pressed = false;
							if(manager.mode == MODE_CONFIG)
							{
								manager.dev_num ++;
								
								if(manager.dev_num > MAX_DEVICE)
									manager.dev_num = 0;
									
								Led7Seg_PrintNum(manager.dev_num);
							}
								
						}
							break;
					case SWITCH_DOWN:
						if(data_table[l_sw_index_u8].l_switch->pressed == true)
						{
							data_table[l_sw_index_u8].l_switch->pressed = false;
							if(manager.mode == MODE_CONFIG)
							{
								manager.dev_num --;
								
								if(manager.dev_num > MAX_DEVICE)
									manager.dev_num = 0;
									
								Led7Seg_PrintNum(manager.dev_num);
							}
								
						}
							break;
					}
				}
			}			
		}
}
#endif


void TIM2_IRQHandler (void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET)
	{
		sw_process_data[0] = GPIOB->IDR;
		sw_process_data[1] = GPIOA->IDR;
		sw_process_data[2] = GPIOE->IDR;
		sw_process_data[3] = GPIOC->IDR;
		sw_process_data[4] = GPIOD->IDR;
		
		TIM_ClearFlag(TIM2, TIM_IT_Update);
	}
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
/* Infinite loop */
/* Use GDB to find out why we're here */
while (1);
}
#endif



