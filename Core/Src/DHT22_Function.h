#include "main.h"
#include "stdbool.h"

#define DHT22_PORT 	GPIOC
#define DHT22_PIN 	GPIO_PIN_3

extern void Display_Temp (float Temp);
extern void Display_Rh (float Rh);
extern void DHT22_Start(void);
extern uint8_t DHT22_Check_Response(void);
extern uint8_t DHT22_Read(void);
