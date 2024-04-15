#ifndef _Timer_control_H
#define _Timer_control_H
#include "main.h"



extern volatile uint32_t Timer6_counter;
extern volatile uint32_t Timer7_counter;

extern void Delay_timer6_1ms (uint32_t Timer6_counter_number);
extern void Delay_timer7_100ms (uint32_t Timer7_counter_number);


extern volatile uint32_t Timer16_counter;
extern void Delay_timer16_10us (uint32_t Timer16_counter_number);

#endif