#ifndef _LED7SEG_H_
#define _LED7SEG_H_

#include <stdint.h>
#include <stdbool.h>

void Init_Led7Seg(void);
int Led7Seg_PrintNum(uint8_t value);
int main_TestLed7Seg(void);


#endif
