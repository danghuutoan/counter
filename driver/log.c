/*******************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 * 
 * @file    log.c
 * @author  The LightCo
 * @version V1.0.0
 * @date    June-16-2015
 * @brief   This file contains expand for log
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>  
#include <time.h>  
#include "log.h"
#include "hal_com.h"
#include "stm32f10x_usart.h"
#include <stdio.h>
#include <rt_misc.h>

#pragma import(__use_no_semihosting_swi)

/* Privated define --------------------------------------------------------- */
/* ASCII character */
#define CNTLQ                          0x11
#define CNTLS                          0x13
#define DEL                            0x7F
#define BACKSPACE                      0x08
#define CR                             0x0D
#define LF                             0x0A
#define ESC                            0x1B
#define CTRL_C                         0x03 
#define CTRL_D                         0x04
#define TAB                            0x09

#define BB_COUNT                       (sizeof(bb)/sizeof(bb[0]))

#define SLOG_INFO_HEADER               "\033[92mINFO\t: "
#define SLOG_FATAL_HEADER              "\033[91mFATAL\t: " 
#define SLOG_ERROR_HEADER              "\033[91mERROR\t: "
#define SLOG_WARN_HEADER               "\033[93mWARN\t: "
#define SLOG_DEBUG_HEADER              "\033[39mDEBUG\t: "  

/* Privated typedef --------------------------------------------------------- */ 


/* Privated function -------------------------------------------------------- */ 
static void     com_irq_handler(uint8_t c);

/* Privated variables ------------------------------------------------------- */ 
static hal_com_t com_dbg;
static uint8_t   c;
static volatile uint8_t   rc = 0;
static uint8_t log_com_idx = 0;
uint8_t log_com_buffer[100];
uint8_t data_len;
#define HEADER_LEN							5
#define MODE_TIME_CONFIG 				0x01

typedef enum {
	RCV_HEADER,
	RCV_LEN,
	RCV_DATA,
	RCV_FOOTER
} com_sm_t;
com_sm_t com_log_sm = RCV_HEADER;
bool header_received;
bool log_data_rx_completed;
/* local com driver configure */
static hal_com_t com_dbg = 
{
    .port_name   = COM1,
    .baudrate    = COM_BAUD_115200,
    .priority    = COM_PRIO_1,
    .irq_handler = com_irq_handler,
    .data        = &c
};

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
FILE __stderr;

static int SER_PutChar (int c) {


  *(com_dbg.data) = c;
	hal_com_sendbyte(&com_dbg);
  return (c);
}


/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
static int SER_GetChar (void) {
	return 0;

}

/* Exported functions ------------------------------------------------------- */



/*
 * _read
 */
int _read(int file, char *ptr, int len) 
{
  int CharCnt = 0x00;
  while(CharCnt < len) 
  {
      *ptr++ = hal_com_readbyte(&com_dbg);
      CharCnt++;
  }
  return CharCnt;  
}
/*
 * _write
 */
int _write(int file, char *ptr, int len) 
{
    int n;
    for (n = 0; n < len; n++) 
    {
        *(com_dbg.data) = *ptr++;
        hal_com_sendbyte(&com_dbg);
    }
    return n;
}

void log_com_start_rx(void)
{
	log_data_rx_completed = false;
	log_com_idx = 0;
	com_log_sm = RCV_HEADER;
}

bool log_data_is_rcved(void)
{
	return log_data_rx_completed;
}

uint8_t* log_get_buffer(void)
{
	return log_com_buffer;
}
/*
 * com_irq_handler
 */
static void com_irq_handler(uint8_t _c)
{
		//printf("%d \r\n",_c);
	  log_com_buffer[log_com_idx++]=_c;

	switch(com_log_sm)
	{
		case RCV_HEADER:
			 if((log_com_idx == HEADER_LEN)&&(log_data_rx_completed == false))
			 {
				 log_com_buffer[log_com_idx] = 0;
				 if(!strcmp((const char*)log_com_buffer,"START"))
				 {
					 com_log_sm = RCV_LEN;
				 }
				 else
				 {
					 log_com_start_rx();
				 }
				 
			 }
			break;
		case RCV_LEN:
			if(log_com_idx == (HEADER_LEN+1))
			{
				data_len = log_com_buffer[HEADER_LEN];
				com_log_sm = RCV_DATA;
			//	printf("len = %d",data_len);
			}
			break;
		case RCV_DATA:
			//printf("data0\r\n");
			if(log_com_idx == (HEADER_LEN + 1 + data_len))
			{
				//printf("data\r\n");
				com_log_sm = RCV_FOOTER;
			}
			
			break;
		case RCV_FOOTER:
			//printf("end0 \r\n");
			if(log_com_idx == (HEADER_LEN + 1+data_len +3))
			{ 
				log_com_buffer[HEADER_LEN +1 +data_len +3] = 0;
				if(!strcmp((const char *)&log_com_buffer[HEADER_LEN+1+data_len],"END"))
				{
				//	printf("end \r\n");
					log_data_rx_completed = true;
					log_com_idx = 0;
					com_log_sm = RCV_HEADER;
				}	
			}
			break;
	}
	
	if (log_com_idx > (HEADER_LEN + 1+data_len +3))
			{
				log_com_start_rx();
			}
		 
    rc = 1;
}

/*
 * log_init
 */
void log_init( void )
{
    hal_com_init(&com_dbg);
    /* enable IRQ */
    hal_com_enable_irq(&com_dbg);
}


int fputc(int c, FILE *f) {
  if (c == '\n')  {
    SER_PutChar('\r');
  }
  return (SER_PutChar(c));
}


int fgetc(FILE *f) {
  return (SER_PutChar(SER_GetChar()));
}


int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}


void _ttywrch(int c) {
  SER_PutChar(c);
}


void _sys_exit(int return_code) {
label:  goto label;  /* endless loop */
}

// /*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE****/
