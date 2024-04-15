#include "Car_control.h"
#include "Timer_control.h"
#include "DFPLAYER_MINI.h"
#include <stdio.h>
uint8_t Distance_array[2], motor_counter = 1;
void Car_direction(uint8_t dir_cmd){
	switch(dir_cmd) {
		case Car_stop : 
			GPIOC->ODR |= 0xful<<4;
			Car_state = Car_stop;
			break;
		case Car_forward : 
			PWM_Level_CH3(70);
			PWM_Level_CH4(70);
			GPIOC->ODR &= ~(0xful<<4);
			GPIOC->ODR |= 0x1ul<<7 | 0x1ul<<4;
			Car_state = Car_forward;
			break;
		case Car_right : 
			PWM_Level_CH3(70);
			GPIOC->ODR &= ~(0xful<<4);
			GPIOC->ODR |= 0x1ul<<4;
			Car_state = Car_right;
			break;
		case Car_left : 
			PWM_Level_CH4(70);
			GPIOC->ODR &= ~(0xful<<4);
			GPIOC->ODR |= 0x1ul<<7;
			Car_state = Car_left;
			break;
		case Car_back : 
			PWM_Level_CH3(60);
			PWM_Level_CH4(60);
			GPIOC->ODR &= ~(0xful<<4);
			GPIOC->ODR |= 0x1ul<<6 | 0x1ul<<5;
			Car_state = Car_back;
			break;
			case MH_Car_right : 
			PWM_Level_CH3(50);
			GPIOC->ODR &= ~(0xful<<4);
			GPIOC->ODR |= 0x1ul<<4;
			Car_state = Car_right;
			break;
		case MH_Car_left : 
			PWM_Level_CH4(50);
			GPIOC->ODR &= ~(0xful<<4);
			GPIOC->ODR |= 0x1ul<<7;
			Car_state = Car_left;
			break;
	}
}
void obstacle_avoidance_cmd(void) {
		switch(motor_counter) {
			case 1 : 
				PWM_Level_TIM15_CH1(35);
				HAL_Delay(1000);
				motor_counter++;
				break;
			case 2 : 
				Distance_array[0] = Distance;
				PWM_Level_TIM15_CH1(105);
				HAL_Delay(1000);
				motor_counter++;
				break;
			case 3 : 
				Distance_array[1] = Distance;
				PWM_Level_TIM15_CH1(75);
				HAL_Delay(1000);
				motor_counter = 1;
				servo_motor_flag = false;
				Distance = 100;
				if(Distance_array[0] > Distance_array[1]) {
					Car_direction(Car_back);
					HAL_Delay(500);
					Car_direction(Car_right);
					HAL_Delay(500);
					Car_direction(Car_forward);
				}
				else if (Distance_array[0] < Distance_array[1]) {
					Car_direction(Car_back);
					HAL_Delay(500);
					Car_direction(Car_left);
					HAL_Delay(500);
					Car_direction(Car_forward);
				}
				else {
					Car_direction(Car_back);
					HAL_Delay(700);
					Car_direction(Car_right);
					HAL_Delay(500);
					Car_direction(Car_forward);
				}
				break;
		}
		
}
void Car_Switch_CMD(uint8_t rxData) {
	switch(rxData) {
			/*Song Control Function*/
			case '1' :
				DF_design(song_1);
				break;
			case '2' :
				DF_design(song_2);
				break;
			case '3' :
				DF_design(song_3);
				break;
			case '4' :
				DF_design(song_4);
				break;
			case '5' :
				DF_design(song_5);
				break;
			case '6':
				DF_design(song_6);
				break;
			case '7':
				DF_design(song_7);
				break;
			case '8':
				DF_design(song_8);
				break;
			case '9':
				DF_design(song_9);
				break;
			case '0':
				DF_design(song_10);
				break;
			/*Music control*/
			case '+' :
				DF_volume_up();
				break;
			case '-' :
				DF_volume_down();
				break;
			case 'N' :
				DF_Next();
				break;
			case 'P' :
				DF_Previous();
				break;
			case 'S' :
				DF_Playback();
				break;
			case 'O' :
				DF_Pause();
				break;
			case 'Q' :
				DF_stop_play();
				break;
			/*Car Control Function*/
			case 'f' :
				if(Car_mod == OA_mode || Car_mod == TC_mode || Car_mod == Normal_mode) {
					Car_direction(Car_forward);
				}
				break;
			case 'l' :
				if(Car_mod == OA_mode || Car_mod == TC_mode || Car_mod == Normal_mode) {
					Car_direction(Car_left);
				}
				break;
			case 'r' :
				if(Car_mod == OA_mode || Car_mod == TC_mode || Car_mod == Normal_mode) {
					Car_direction(Car_right);
				}
				break;
			case 'b' :
				if(Car_mod == OA_mode || Car_mod == TC_mode || Car_mod == Normal_mode) {
					Car_direction(Car_back);
				}
				break;
			case 'p' :
				Car_direction(Car_stop);
				break;
			/*Car mod*/
			case 'a' :
				Car_mod = OA_mode;
				lcd_flag = true;
				Car_direction(Car_stop);
				break;
			case 'd' :
				Car_mod = Stby_mode;
				Car_direction(Car_stop);
				break;
			case 'x' :
				Car_mod = TC_mode;
				lcd_flag = true;
				Car_direction(Car_stop);
				break;
			case 'y' :
				Car_mod = Normal_mode;
				lcd_flag = true;
				Car_direction(Car_stop);
				break;
		}
}

void Parking_sensors_cmd(void) {
	if(Distance > 40) {
		PWM_Level_CH3(60);
		PWM_Level_CH4(60);
	}
	else if((Distance > 20) && (Distance <= 40)) {
		PWM_Level_CH3(50);
		PWM_Level_CH4(50);
	}
	else if((Distance > 10) && (Distance <= 20)) {
		PWM_Level_CH3(40);
		PWM_Level_CH4(40);
	}
	else if((Distance > 5) && (Distance <= 10)) {
		PWM_Level_CH3(30);
		PWM_Level_CH4(30);
	}
	else {
		Car_direction(Car_stop);
	}

}


/*MOTOR PWM*/
void Enable_stdby(void) {
	GPIOA->ODR &= ~0x1ul;
}

void Disable_stdby(void) {
	GPIOA->ODR |= 0x1ul;
}

//Timer 3(TIM3)
			/****CH3****/
void Enable_PWM_CH3(void) {
	TIM3->CNT = 0;
	TIM3->CR1 |= 0x1ul;
	TIM3->CCER |= 0x1ul<<8;
}
void Disable_PWM_CH3(void) {
	TIM3->CCER &= ~(0x1ul<<8);
	TIM3->CR1 &= ~0x1ul;
}
void PWM_Level_CH3(uint32_t level) {
	TIM3->CCR3 = level;

}
			/****CH4****/
void Enable_PWM_CH4(void) {
	TIM3->CNT = 0;
	TIM3->CR1 |= 0x1ul;
	TIM3->CCER |= 0x1ul<<12;
}
void Disable_PWM_CH4(void) {
	TIM3->CCER &= ~(0x1ul<<12);
	TIM3->CR1 &= ~0x1ul;
}
void PWM_Level_CH4(uint32_t level) {
	TIM3->CCR4 = level;
}


void Disable_PWM_TIM15_CH1(void) {
	TIM15->CCER &= ~(0x1ul);
	TIM15->CR1 &= ~(0x1ul);
}
void PWM_Level_TIM15_CH1(uint32_t level) {
	TIM15->CCR1 = level;

}