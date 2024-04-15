#include "DHT22_Function.h"
#include "Timer_control.h"
#include "i2c-lcd.h"
#include <stdio.h>

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Display_Temp (float Temp) {
	char str[20] = {0};
	lcd_put_cur(1,0);
	sprintf(str, "T:%.1f", Temp);
	lcd_send_string(str);
	lcd_send_data('C');
}

void Display_Rh (float Rh) {
	char str[20] = {0};
	lcd_put_cur(1,8);
	sprintf(str, "H:%.2f", Rh);
	lcd_send_string(str);
	lcd_send_data('%');
}
/************************************ DHT22 FUNCTIONS ************************************/
void DHT22_Start(void) {
	Set_Pin_Output(DHT22_PORT, DHT22_PIN);						//Set the pin as output
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 0);			//pull the pin low
	Delay_timer16_10us(120);													//wait for > 1ms
	
	HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, 1);			//pull the pin high
	Delay_timer16_10us(3);														//wait for 30us
	
	Set_Pin_Input(DHT22_PORT, DHT22_PIN);						  //Set the pin as input
}

uint8_t DHT22_Check_Response(void) {
	uint8_t Response = 0;
	Delay_timer16_10us(4);														//wait for 40us
	if(!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) {	//if the pin is low
		Delay_timer16_10us(8);													//wait for 80us
		
		if((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) Response = 1;		//if the pin is high, response is ok
		else Response = -1;
	}
	
	while((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))); 						//wait for the pin to go low
	return Response;
}

uint8_t DHT22_Read(void) {
	uint8_t i = 0, j;
	for(j=0; j<8; j++) {
		while(!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));	//wait for the pin to go high
		Delay_timer16_10us(4);															//wait for 40us
		if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) {		//if the pin is low
			i &= ~(1<<(7-j));			//write 0
		}
		else i |= (1<<(7-j)); 	//if the pin is high, write 1
		while((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))); 	//wait for the pin to go low
	}
	return i;
}