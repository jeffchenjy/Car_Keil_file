#include "main.h"
#include "Timer_control.h"


void Delay_timer6_1ms (uint32_t Timer6_counter_number) {
	
	uint32_t currnt_counter;
	
	currnt_counter = Timer6_counter;
	while((Timer6_counter - currnt_counter) <  Timer6_counter_number) {
		__NOP();
	}
}
void Delay_timer7_100ms (uint32_t Timer7_counter_number) {
	
	uint32_t currnt_counter;
	
	currnt_counter = Timer7_counter;
	while((Timer7_counter - currnt_counter) <  Timer7_counter_number) {
		__NOP();
	}
}

void Delay_timer16_10us (uint32_t Timer16_counter_number) {
	
	uint32_t currnt_counter;
	
	currnt_counter = Timer16_counter;
	while((Timer16_counter - currnt_counter) <  Timer16_counter_number) {
		__NOP();
	}
}