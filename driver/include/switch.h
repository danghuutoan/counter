#ifndef __SWITCH_H__
#define __SWITCH_H__
#include "stm32f10x_exti.h"
#include "hal_dio.h"
#include <stdbool.h>
typedef enum _switch_name
{
	SWITCH_1 = 0,
	SWITCH_2,
	SWITCH_3,
	SWITCH_4,
	SWITCH_5,
	SWITCH_6,
	SWITCH_7,
	SWITCH_8,
	SWITCH_9,
	SWITCH_10,
	SWITCH_11,
	SWITCH_12,
	SWITCH_13,
	SWITCH_14,
	SWITCH_15,
	SWITCH_16,
	SWITCH_17,
	SWITCH_18,
	SWITCH_19,
	SWITCH_20,
	SWITCH_21,
	SWITCH_22,
	SWITCH_23,
	SWITCH_24,
	SWITCH_25,
	SWITCH_26,
	SWITCH_27,
	SWITCH_28,
	SWITCH_29,
	SWITCH_30,
	SWITCH_RESET,
	SWITCH_MODE,
	SWITCH_UP,
	SWITCH_DOWN,
	SWITCH_MAX,
} switch_name_t;

typedef enum _switch_state
{
	SWITCH_LOW,
	SWITCH_HIGH
} switch_state_t;
typedef struct _switch
{
	switch_name_t 				name;
	EXTITrigger_TypeDef trigger_type;
	uint8_t               last_data;
	uint8_t               new_data;
	bool                   pressed;
	hal_dio_t							 *led;
	void (* callback)(void);
} switch_t;
void EXTI0_Config(void);
void switch_init (switch_t* l_switch);
void timer_init(void);
switch_state_t  switch_read(switch_t* l_switch);
#endif /*__SWITCH_H__*/
