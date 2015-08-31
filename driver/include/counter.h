#ifndef __COUNTER_H__
#define __COUNTER_H__
#include "std_type.h"

#define FRAME_LENGTH 										9

void counter_init(void);
bool counter_rx_data_is_rcved(void);
void counter_start_rx(void);
void counter_coppy_buffer(uint8_t* l_buffer);
#endif /* __COUNTER_H__ */
