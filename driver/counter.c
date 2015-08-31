#include "counter.h"
#include "hal_com.h"
#include "std_type.h"


static uint8_t com_data;
static volatile bool data_received = FALSE;
static uint8_t com_buffer[FRAME_LENGTH+5];
static uint8_t com_index;
static void counter_com_irq_handler(uint8_t _c);
/* local com driver configure */
static hal_com_t com_counter = 
{
    .port_name   = COM2,
    .baudrate    = COM_BAUD_9600,
    .priority    = COM_PRIO_1,
    .irq_handler = counter_com_irq_handler,
    .data        = &com_data
};


static void counter_com_init(void);



void counter_init(void)
{
		counter_com_init();
}

bool counter_rx_data_is_rcved (void)
{
	return data_received;
}
void counter_start_rx(void)
{
	data_received = FALSE;
}
void counter_coppy_buffer(uint8_t* l_buffer)
{
	for(int i = 0 ; i< 9 ; i++)
	{
		l_buffer[i] = com_buffer[i];
	}
}


/*
 * com_irq_handler
 */
static void counter_com_irq_handler(uint8_t _c)
{
	  
		com_buffer[com_index++] = _c;
		if(com_index == 1)
	  {
			if(com_buffer[com_index-1] != 0xAA )
			{
				com_index = 0;
			}
	  }
		
		if(com_index >= FRAME_LENGTH )
		{
			com_index = 0;
			data_received = TRUE;
		}
    com_data = _c;
}


static void counter_com_init(void)
{
	hal_com_init(&com_counter);
	hal_com_enable_irq(&com_counter);
}
