#include "switch.h"
#include  "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include <stdio.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED_PIN  								GPIO_Pin_13
#define LED_PORT 								GPIOC
#define LED_CLK  								RCC_APB2Periph_GPIOC
#define SW_EXTI_Line 						EXTI_Line4
#define SW_PIN    		  				GPIO_Pin_4
#define SW_PORT         				GPIOA
#define SW_GPIO_CLK     				RCC_APB2Periph_GPIOA
#define SW_GPIO_PortSource 			GPIO_PortSourceGPIOA
#define SW_GPIO_PinSource       GPIO_PinSource4
#define SW_NVIC_IRQChannel      EXTI4_IRQn


//#define LED_PIN  								GPIO_Pin_13
//#define LED_PORT 								GPIOE
//#define LED_CLK  								RCC_APB2Periph_GPIOE
//#define SW_IRQ    							EXTI2_IRQHandler
//#define SW_EXTI_Line 						EXTI_Line2
//#define SW_PIN    		  				GPIO_Pin_2
//#define SW_PORT         				GPIOE
//#define SW_GPIO_CLK     				RCC_APB2Periph_GPIOE
//#define SW_GPIO_PortSource 			GPIO_PortSourceGPIOE
//#define SW_GPIO_PinSource       GPIO_PinSource2
//#define SW_NVIC_IRQChannel      EXTI2_IRQn
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;

/* Private function prototypes -----------------------------------------------*/
void EXTI0_Config(void);

typedef struct _switch_config
{
	GPIO_TypeDef * GPIO_Port;
	uint32_t       GPIO_Clock;
	uint16_t       GPIO_Pin;
	uint8_t        GPIO_PortSource;
	uint8_t        GPIO_PinSource;
	uint32_t       EXTI_line;
	uint8_t        NVIC_IRQChannel;
} switch_config_t;
/* Private functions ---------------------------------------------------------*/
#define CREATE_SWITCH(port,pin,IRQChannel) 				 \
{																									 \
	.GPIO_Port  				= GPIO##port,								 \
	.GPIO_Clock 				= RCC_APB2Periph_GPIO##port, \
	.GPIO_Pin     			= GPIO_Pin_##pin,            \
	.GPIO_PortSource    = GPIO_PortSourceGPIO##port, \
	.GPIO_PinSource     = GPIO_PinSource##pin,       \
	.EXTI_line     			= EXTI_Line##pin,            \
	.NVIC_IRQChannel    = IRQChannel								 \
}
static const switch_config_t switch_list [] = 
{
	/* SWITCH_1*/
	CREATE_SWITCH(E,2,EXTI2_IRQn),
	/* SWITCH_2 */
	CREATE_SWITCH(E,3,EXTI3_IRQn),	
		/* SWITCH_3 */
	CREATE_SWITCH(E,4,EXTI4_IRQn),
		/* SWITCH_4 */
	CREATE_SWITCH(E,5,EXTI9_5_IRQn),
			/* SWITCH_5 */
	CREATE_SWITCH(E,6,EXTI9_5_IRQn),
		/* SWITCH_6 */
	CREATE_SWITCH(C,13,EXTI15_10_IRQn),
		/* SWITCH_7 */
	CREATE_SWITCH(C,14,EXTI15_10_IRQn),
		/* SWITCH_8 */
	CREATE_SWITCH(C,15,EXTI15_10_IRQn),
		/* SWITCH_9 */
	CREATE_SWITCH(C,0,EXTI0_IRQn),
		/* SWITCH_10 */
	CREATE_SWITCH(C,1,EXTI1_IRQn),
		/* SWITCH_11 */
	CREATE_SWITCH(C,2,EXTI3_IRQn),
		/* SWITCH_12 */
	CREATE_SWITCH(C,3,EXTI3_IRQn),
	 /* SWITCH_13 */
	CREATE_SWITCH(A,0,EXTI0_IRQn),
	/* SWITCH_14 */
	CREATE_SWITCH(A,1,EXTI1_IRQn),
	/* SWITCH_15 */
	CREATE_SWITCH(A,2,EXTI2_IRQn),
	/* SWITCH_16 */
	CREATE_SWITCH(A,3,EXTI3_IRQn),
	/* SWITCH_17 */
	CREATE_SWITCH(A,4,EXTI4_IRQn),
	/* SWITCH_18 */
	CREATE_SWITCH(A,5,EXTI9_5_IRQn),
	/* SWITCH_19 */
	CREATE_SWITCH(A,6,EXTI9_5_IRQn),
	/* SWITCH_20 */
	CREATE_SWITCH(A,7,EXTI9_5_IRQn),
	/* SWITCH_21 */
	CREATE_SWITCH(C,4,EXTI4_IRQn),
	/* SWITCH_22 */
	CREATE_SWITCH(C,5,EXTI9_5_IRQn),
	/* SWITCH_23 */
	CREATE_SWITCH(B,0,EXTI0_IRQn),
	/* SWITCH_24 */
	CREATE_SWITCH(B,1,EXTI1_IRQn),
	/* SWITCH_25 */
	CREATE_SWITCH(E,7,EXTI9_5_IRQn),
	/* SWITCH_26 */
	CREATE_SWITCH(E,8,EXTI9_5_IRQn),
	/* SWITCH_27 */
	CREATE_SWITCH(E,9,EXTI9_5_IRQn),
	/* SWITCH_28 */
	CREATE_SWITCH(E,10,EXTI15_10_IRQn),
	/* SWITCH_29 */
	CREATE_SWITCH(E,11,EXTI15_10_IRQn),
	/* SWITCH_30 */
	CREATE_SWITCH(E,12,EXTI15_10_IRQn),
	/* SWITCH_RESET */
	CREATE_SWITCH(B,6,EXTI9_5_IRQn),		
	/* SWITCH_MODE*/
	CREATE_SWITCH(B,5,EXTI2_IRQn),
		/* SWITCH_UP*/
	CREATE_SWITCH(B,2,EXTI2_IRQn),
};
static void (* callback_list[5]) (void);
void timer_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Enable the TIM2 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);
	
	 /* Compute the prescaler value */
  //PrescalerValue = (uint16_t) (SystemCoreClock / 12000000) - 1;
	
	  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 100 -1; /* from 10Khz down to 10 hz*/
  TIM_TimeBaseStructure.TIM_Prescaler = 7200 -1; /* from 36Mhz down to 10k*/
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	 /* TIM IT enable */
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);

	
}

void switch_init (switch_t* l_switch)
{
	/* Enable GPIOA clock */
	callback_list[l_switch->name] = l_switch->callback;
	//GPIO_AFIODeInit();
  //GPIO_DeInit(switch_list[l_switch->name].GPIO_Port);
	RCC_APB2PeriphClockCmd(switch_list[l_switch->name].GPIO_Clock, ENABLE);
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = switch_list[l_switch->name].GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	
  GPIO_Init(switch_list[l_switch->name].GPIO_Port, &GPIO_InitStructure);

#ifdef SW_IRQ
  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(switch_list[l_switch->name].GPIO_PortSource, switch_list[l_switch->name].GPIO_PinSource);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = switch_list[l_switch->name].EXTI_line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = switch_list[l_switch->name].NVIC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
}

switch_state_t  switch_read(switch_t* l_switch)
{
	switch_state_t l_data =(switch_state_t)( (switch_list[l_switch->name].GPIO_Port->IDR)&(switch_list[l_switch->name].GPIO_Pin) );
	if(l_data)
		return SWITCH_HIGH;
	else
		return SWITCH_LOW;
}
/**
  * @brief  Configure PA.00 in interrupt mode
  * @param  None
  * @retval None
  */
void EXTI0_Config(void)
{
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(SW_GPIO_CLK, ENABLE);
  
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = SW_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(SW_PORT, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(SW_GPIO_PortSource, SW_GPIO_PinSource);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = SW_EXTI_Line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = SW_NVIC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



void EXTI0_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line0);
  }
}


void EXTI1_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line1) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line1);
  }
}
void EXTI2_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line2);
  }
}
void EXTI3_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line3);
  }
}
void EXTI4_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line4);
  }
}
void EXTI5_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line5);
  }
}
void EXTI6_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line6);
  }
}
void EXTI9_5_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
  {
				callback_list[SWITCH_1]();
		EXTI_ClearITPendingBit(EXTI_Line7);
  }
}
void EXTI8_IRQHandler (void)
{
	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
  {
		EXTI_ClearITPendingBit(EXTI_Line8);
  }
}

