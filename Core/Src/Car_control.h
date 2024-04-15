#ifndef _Car_control_H
#define _Car_control_H
#include "main.h"
#include <stdbool.h>
enum Car_Direction {
	Car_stop = 0,
	Car_forward,
	Car_right,
	Car_left,
	Car_back,
	MH_Car_left,
	MH_Car_right
};
enum Car_MOD {
	OA_mode,
	TC_mode,
	Normal_mode,
	Stby_mode
};

extern void Car_direction(uint8_t dir_cmd);
extern void obstacle_avoidance_cmd(void);
extern void Parking_sensors_cmd(void);
extern void Car_Switch_CMD(uint8_t rxData);

extern volatile uint32_t Car_state;
extern uint8_t Distance;
extern bool servo_motor_flag;
extern bool lcd_flag;
extern volatile uint8_t Car_mod;


extern void Enable_stdby(void);
extern void Disable_stdby(void);


extern void Enable_PWM_CH3(void);
extern void Disable_PWM_CH3(void);

extern void Enable_PWM_CH4(void);
extern void Disable_PWM_CH4(void);


extern void PWM_Level_CH3(uint32_t level);
extern void PWM_Level_CH4(uint32_t level);

extern void Disable_PWM_TIM15_CH1(void);
extern void PWM_Level_TIM15_CH1(uint32_t level);

extern void PWM_Level_TIM15_CH1(uint32_t level);
#endif