/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 * 
 * @file    hal_com.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    June-26-2015
 * @brief   This file contains expand of hal_com
 *
 ******************************************************************************/  
/* Includes ------------------------------------------------------------------*/
#include "hal_com.h"
/** #ASIC Hardware target */
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

/* Private typedef -----------------------------------------------------------*/

/* pin hardware configuration */
typedef struct _com_hw_pin_config{
    uint32_t               Clock;
    GPIO_TypeDef          *Port;
    uint16_t               PinNum;
    GPIOMode_TypeDef       Mode;
}com_hw_pin;

/* hardware COM configuration */
typedef struct _com_hw_config{
   USART_TypeDef*       COMx;
   uint32_t             ClockMask;
   uint8_t              IRQ;
   com_hw_pin           PIN_TX;
   com_hw_pin           PIN_RX;
}com_hw_config;

/* Private function_ ---------------------------------------------------------*/
static void hal_com_irq(uint8_t chid);

/* Private variables ---------------------------------------------------------*/

/* Interrupt Handler */
static void (*DataReceivedHandler[COM_MAX_IDX])(uint8_t);

static const com_hw_config hw_config[] = 
{
    /* USART1 */
    {
        .COMx         =  USART1,
        .ClockMask    =  RCC_APB2Periph_USART1,
        .IRQ          =  USART1_IRQn,
        /* tx pin */
        {
            .Clock = RCC_APB2Periph_GPIOA,
            .Port  = GPIOA,
            .PinNum= GPIO_Pin_9,
            .Mode  = GPIO_Mode_AF_PP,
        },
        /* rx pin */
        {
            .Clock = RCC_APB2Periph_GPIOA,
            .Port  = GPIOA,
            .PinNum= GPIO_Pin_10,
            .Mode  = GPIO_Mode_IN_FLOATING,
        }
    },
    /* USART2 */
    {
        .COMx         =  USART2,
        .ClockMask    =  RCC_APB1Periph_USART2,
        .IRQ          =  USART2_IRQn,
        /* tx pin */
        {
            .Clock = RCC_APB2Periph_GPIOA,
            .Port  = GPIOA,
            .PinNum= GPIO_Pin_2,
            .Mode  = GPIO_Mode_AF_PP,
        },
        /* rx pin */
        {
            .Clock = RCC_APB2Periph_GPIOA,
            .Port  = GPIOA,
            .PinNum= GPIO_Pin_3,
            .Mode  = GPIO_Mode_IN_FLOATING,
        }
    },
    /* USART3 */
    {
        .COMx         =  USART3,
        .ClockMask    =  RCC_APB1Periph_USART3,
        .IRQ          =  USART3_IRQn,
        /* tx pin */
        {
            .Clock = RCC_APB2Periph_GPIOC,
            .Port  = GPIOC,
            .PinNum= GPIO_Pin_10,
            .Mode  = GPIO_Mode_AF_PP,
        },
        /* rx pin */
        {
            .Clock = RCC_APB2Periph_GPIOC,
            .Port  = GPIOC,
            .PinNum= GPIO_Pin_11,
            .Mode  = GPIO_Mode_IN_FLOATING,
        }
    },
};

/* Exported functions ------------------------------------------------------- */

/*
 * hal_com_init
 * The function shall initialize independence com channel
 */
int hal_com_init( hal_com_t * com )
{
    int               ret;
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;

#if (HAL_COM_DEBUG == STD_ON)    
    /* check null pointer */
    if( (void*)0 == com )
    {
        ret = (int)COM_NULL_PTR;
    }
    /* INVALID PORT */
    else if( COM_MAX_IDX <= com->port_name || COM1 > com->port_name )
    {
        ret = (int)COM_INVALID_PORT;
    }
    else
#endif /* HAL_COM_DEBUG */        
    {
   
    
    /* Enable the USART1 and AFIO peripheral clocks */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		if(com->port_name == COM1)
		{
			RCC_APB2PeriphClockCmd(hw_config[com->port_name].ClockMask, ENABLE);
		}
    else
		{
			RCC_APB1PeriphClockCmd(hw_config[com->port_name].ClockMask, ENABLE);
		}
    RCC_APB2PeriphClockCmd(hw_config[com->port_name].PIN_TX.Clock, ENABLE);
	  RCC_APB2PeriphClockCmd(hw_config[com->port_name].PIN_RX.Clock, ENABLE);
 
  //////////////////////////////////////////////////////////////////
  // Configure the GPIO ports( USART Transmit and Receive Lines) 
  //////////////////////////////////////////////////////////////////

  /* Configure the USART_Tx as Alternate function Push-Pull */
  GPIO_InitStructure.GPIO_Mode = hw_config[com->port_name].PIN_TX.Mode;
  GPIO_InitStructure.GPIO_Pin =  hw_config[com->port_name].PIN_TX.PinNum;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(hw_config[com->port_name].PIN_TX.Port, &GPIO_InitStructure);
  
  /* Configure the USART_Rx as input floating */
  GPIO_InitStructure.GPIO_Mode = hw_config[com->port_name].PIN_RX.Mode;
  GPIO_InitStructure.GPIO_Pin = hw_config[com->port_name].PIN_RX.PinNum;
  GPIO_Init(hw_config[com->port_name].PIN_TX.Port, &GPIO_InitStructure);
  
  /*-------------------------------------------------------
  USART configured as:
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - BaudRate = USART_BAUDRATE bps
        - Receive and transmit enabled
  -------------------------------------------------------*/

  USART_InitStructure.USART_BaudRate = com->baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(hw_config[com->port_name].COMx, &USART_InitStructure);
  
  USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;
  USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
  USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable;  
  USART_ClockStructInit(&USART_ClockInitStruct);
 
  /* Enable the USART Receive register not empty interrupt */
  //USART_ITConfig(COMM_INTERFACE, USART_IT_RXNE, ENABLE);
  
  /* Enable the USART */
  USART_Cmd(hw_config[com->port_name].COMx, ENABLE);  
        
        ret = (int)COM_OK;
    }
    return ret;
}

/*
 * hal_com_deinit
 * The function stops operation on independence com channel
 */
int hal_com_deinit( hal_com_t * com )
{
    int ret;
#if (HAL_COM_DEBUG == STD_ON)    
    /* check null pointer */
    if( (void*)0 == com )
    {
        ret = (int)COM_NULL_PTR;
    }
    /* INVALID PORT */
    else if( COM_MAX_IDX <= com->port_name || COM1 > com->port_name )
    {
        ret = (int)COM_INVALID_PORT;
    }
    else
#endif /* HAL_COM_DEBUG */
    {
        USART_DeInit(hw_config[com->port_name].COMx);
        ret = (int)COM_OK;
    }
    return ret;   
}

/*
 * hal_com_sendbyte
 * The function sends one byte to hardware layer
 */
int hal_com_sendbyte( hal_com_t * com )
{
    int           ret;
    uint32_t      TimeOut = HAL_COM_TIMEOUT;
#if (HAL_COM_DEBUG == STD_ON)
    /* check null pointer */
    if( (void*)0 == com )
    {
        ret = (int)COM_NULL_PTR;
    }
    /* INVALID PORT */
    else if( COM_MAX_IDX <= com->port_name || COM1 > com->port_name )
    {
        ret = (int)COM_INVALID_PORT;
    }
    else
#endif /* HAL_COM_DEBUG */         
    {              
        /* wait transmit success */
        do{
            TimeOut--;
        }
        while( TimeOut && ((hw_config[com->port_name].COMx->SR & 0x0040) == (uint16_t)0) );
        /* Transmit Data */
        USART_SendData(hw_config[com->port_name].COMx, *(com->data));
        
        ret = (int)COM_OK;
    }
    return ret;    
}

/*
 * hal_com_readbyte
 * The function stops independence com channel
 */
int hal_com_readbyte( hal_com_t * com )
{
    int ret;
#if (HAL_COM_DEBUG == STD_ON)    
    /* check null pointer */
    if( (void*)0 == com )
    {
        ret = (int)COM_NULL_PTR;
    }
    /* INVALID PORT */
    else if( COM_MAX_IDX <= com->port_name || COM1 > com->port_name )
    {
        ret = (int)COM_INVALID_PORT;
    }
    else
#endif /* HAL_COM_DEBUG */        
    {
        /* Reaceive Data */
        *(com->data) = (uint8_t)(hw_config[com->port_name].COMx->DR & (uint16_t)0x01FF);
        ret = (int)COM_OK;
    }
    return ret;    
}

/*
 * hal_com_enable_irq
 * The function reads one byte from hardware layer
 */
int hal_com_enable_irq( hal_com_t * com )
{
    int              ret;
    NVIC_InitTypeDef NVIC_InitStructure;
#if (HAL_COM_DEBUG == STD_ON)       
    /* check null pointer */
    if( (void*)0 == com )
    {
        ret = (int)COM_NULL_PTR;
    }
    /* INVALID PORT */
    else if( COM_MAX_IDX <= com->port_name || COM1 > com->port_name )
    {
        ret = (int)COM_INVALID_PORT;
    }
    else if((void*)0 == com->irq_handler)
    {
        ret = (int)COM_NULL_IRQ;
    }
    else
#endif /* HAL_COM_DEBUG */        
    {
        /* Store point to irq handler function at upper layer */
        DataReceivedHandler[com->port_name] = com->irq_handler;
        /* Configure the Priority Group to 4 bit */
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
        hw_config[com->port_name].COMx->CR1 |= (1UL << 5);
        /* enable interrupt channel */
        NVIC_InitStructure.NVIC_IRQChannel            = hw_config[com->port_name].IRQ;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = com->priority;
        NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
        NVIC_Init( &NVIC_InitStructure );

				 /* Enable the USART Receive register not empty interrupt */
        USART_ITConfig( hw_config[com->port_name].COMx, USART_IT_RXNE, ENABLE);
        ret = (int)COM_OK;
    }
    return ret;    
}

/*
 * hal_com_disable_irq
 * The function disables hardware interrupt
 */
int hal_com_disable_irq( hal_com_t * com )
{
    int ret;
    NVIC_InitTypeDef NVIC_InitStructure;
#if (HAL_COM_DEBUG == STD_ON)       
    /* check null pointer */
    if( (void*)0 == com )
    {
        ret = (int)COM_NULL_PTR;
    }
    /* INVALID PORT */
    else if( COM_MAX_IDX <= com->port_name || COM1 > com->port_name )
    {
        ret = (int)COM_INVALID_PORT;
    }
    else
#endif /* HAL_COM_DEBUG */
    {
        /* disable hardware interrupt reception */
        /* usart interrupt disable */
        hw_config[com->port_name].COMx->CR1 &= ~(1UL << 5);
        /* disable arm interrupt channel */
        NVIC_InitStructure.NVIC_IRQChannel    = hw_config[com->port_name].IRQ;
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
        NVIC_Init( &NVIC_InitStructure );

        ret = (int)COM_OK;
    }
    return ret;    
}

/*
 * USARTx_IRQHandler
 * Interrupt function 
 */
static void hal_com_irq(uint8_t chid)
{
    uint8_t c = 0xFF;
#if (HAL_COM_DEBUG == STD_ON)      
    /* check Interrupt handler valid  */
    if( (void*)0 != DataReceivedHandler[chid] )
#endif /* HAL_COM_DEBUG */        
    {
      if(USART_GetITStatus(hw_config[chid].COMx, USART_IT_RXNE) != RESET)
			{  
			/* read data */
        c = (uint8_t)(hw_config[chid].COMx->DR & (uint16_t)0x01FF);
        /* call back function to transfer data to upper module */
        (*DataReceivedHandler[chid])(c);
				USART_ClearITPendingBit(hw_config[chid].COMx, USART_IT_RXNE);
			}
			
			if(USART_GetITStatus(hw_config[chid].COMx, USART_IT_TXE) != RESET)
			{
		
			}
    }
#if (HAL_COM_DEBUG == STD_ON)      
    else
    {
    /* do nothing */
    } 
#endif    
}

/*
 * USART1_IRQHandler
 * Interrupt function of #COM1
 */
void USART1_IRQHandler ( void )
{
    hal_com_irq(COM1);
}

/*
 * USART2_IRQHandler
 * Interrupt function of #COM2
 */
void USART2_IRQHandler ( void )
{
    hal_com_irq(COM2);
}

/*
 * USART3_IRQHandler
 * Interrupt function of #COM3
 */
void USART3_IRQHandler ( void )
{
    hal_com_irq(COM3);
}

/*
 * UART4_IRQHandler
 * Interrupt function of #COM4
 */
void UART4_IRQHandler ( void )
{
    hal_com_irq(COM4);
}

/*
 * UART5_IRQHandler
 * Interrupt function of #COM5
 */
void UART5_IRQHandler ( void )
{
    hal_com_irq(COM5);
}

/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE****/
