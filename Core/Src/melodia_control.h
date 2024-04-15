#ifndef _melodia_control_H
#define _melodia_control_H

#include "main.h"

enum song_num{
	song_off,
	song_1,
	song_2,
	song_3,
	song_4,
	song_5,
	song_6,
	song_7,

};

extern void enable_buzzer_PWM(void);
extern void disable_buzzer_PWM(void);
extern void playMelody(uint32_t song_num);
extern void selectTone(uint32_t tone_num);
extern volatile uint32_t song_number;

extern void send_string(char * String_ptr);

#endif