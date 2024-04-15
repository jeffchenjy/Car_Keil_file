#include "stdio.h"
#include "melodia_def.h"
#include "melodia_control.h"
#include "Timer_control.h"


void enable_buzzer_PWM(void) {
	TIM14->CNT = 0;
	TIM14->CR1 |= 0x1ul;
	TIM14->CCER |= 0x1ul;
}

void disable_buzzer_PWM(void) {
	TIM14->CCER &= ~0x1ul;
	TIM14->CR1 &= ~0x1ul;
}

void selectTone(uint32_t tone_num) {
	uint32_t arrValue;
	//Do Re Mi Fa So La Si Do
	uint32_t ToneArr[]={0,764,681,607,573,510,454,405, 382,340,303,286,255,227,202, 191,170,152,143,128,114,101};
	arrValue = ToneArr[tone_num];
	TIM14->CNT = 0;
	TIM14->ARR = arrValue;
	TIM14->CCR1 = arrValue*0.8;
}

void playMelody(uint32_t song_num) {
	uint32_t stopValue = 110, toneValue, beatValue;
	enable_buzzer_PWM();
	switch(song_num) {
		case song_off :
			disable_buzzer_PWM();
			break;
		case song_1 :
			printf("Play Happy Song.\n\r");
			for(int i=0; i < sizeof(note_happy_song); i++) {
				toneValue = note_happy_song[i];
				if(toneValue == 0) {
					disable_buzzer_PWM();
					beatValue = beat_happy_song[i];
					Delay_timer6_1ms(beatValue);
					if(song_number != song_1) break;
					enable_buzzer_PWM();
				}
				else {
					toneValue += 7;
					selectTone(toneValue);
					beatValue = beat_happy_song[i];
					Delay_timer6_1ms(beatValue);
					disable_buzzer_PWM();
					Delay_timer6_1ms(stopValue);
					if(song_number != song_1) break;
					enable_buzzer_PWM();
				}
			}
			break;
		case song_2 :
			printf("Play Little Star Song.\n\r");
			for(int i=0; i < sizeof(note_little_star); i++) {
				toneValue = note_little_star[i];
				selectTone(toneValue+7);
				beatValue = beat_little_star[i];
				Delay_timer6_1ms(beatValue);
				disable_buzzer_PWM();
				Delay_timer6_1ms(stopValue);
				if(song_number != song_2) break;
				enable_buzzer_PWM();
			}
			break;
		case song_3 :
			printf("Play Little bee Song.\n\r");
			for(int i=0; i < sizeof(note_little_bee); i++) {
				toneValue = note_little_bee[i];
				selectTone(toneValue+7);
				beatValue = beat_little_bee[i];
				Delay_timer6_1ms(beatValue);
				disable_buzzer_PWM();
				Delay_timer6_1ms(stopValue);
				if(song_number != song_3) break;
				enable_buzzer_PWM();
			}
			break;
		case song_4 :
			printf("Play Epic_Sax_Guy.\n\r");
			for(int i=0; i < sizeof(Epic_Sax_Guy); i++) {
				toneValue = Epic_Sax_Guy[i];
				TIM14->CNT = 0;
				TIM14->ARR = toneValue;
				TIM14->CCR1 = toneValue*0.8;
				beatValue = beat_Epic_Sax_Guy[i];
				Delay_timer6_1ms(beatValue);
				disable_buzzer_PWM();
				Delay_timer6_1ms(stopValue);
				if(song_number != song_4) break;
				enable_buzzer_PWM();
			}
			break;
		case song_5 :
			printf("Play megalovania.\n\r");
			for(int i=0; i < sizeof(Melody_of_Megalovania); i++) {
				toneValue = Melody_of_Megalovania[i];
				TIM14->CNT = 0;
				TIM14->ARR = toneValue;
				TIM14->CCR1 = toneValue*0.8;
				beatValue = duration_of_Megalovania[i]*0.7;
				Delay_timer6_1ms(beatValue);
				disable_buzzer_PWM();
				Delay_timer6_1ms(stopValue);
				if(song_number != song_5) break;
				enable_buzzer_PWM();
			}
			break;
		case song_6 :
			printf("Play Bloody Stream.\n\r");
			for(int i=0; i < sizeof(Melody_of_Megalovania); i++) {
				toneValue = Melody_of_Bloody_Stream_JojoBA[i];
				TIM14->CNT = 0;
				TIM14->ARR = toneValue;
				TIM14->CCR1 = toneValue*0.8;
				beatValue = duration_of_Bloody_Stream_JojoBA[i]*0.75;
				Delay_timer6_1ms(beatValue);
				disable_buzzer_PWM();
				Delay_timer6_1ms(stopValue);
				if(song_number != song_6) break;
				enable_buzzer_PWM();
			}
			break;
		case song_7 :
			printf("Play Pirates.\n\r");
			for(int i=0; i < sizeof(Pirates_note); i++) {
				toneValue = Pirates_note[i];
				if(toneValue == 0) {
					disable_buzzer_PWM();
					beatValue = 1000/Pirates_duration[i];
					Delay_timer6_1ms(beatValue);
					if(song_number != song_7) break;
					enable_buzzer_PWM();
				}
				else {
					TIM14->CNT = 0;
					TIM14->ARR = toneValue;
					TIM14->CCR1 = toneValue*0.8;
					beatValue = 1000/Pirates_duration[i];
					Delay_timer6_1ms(beatValue);
					disable_buzzer_PWM();
					stopValue = beatValue*0.8;
					Delay_timer6_1ms(stopValue);
					if(song_number != song_7) break;
					enable_buzzer_PWM();
				}
			}
			break;
	}
	disable_buzzer_PWM();
}