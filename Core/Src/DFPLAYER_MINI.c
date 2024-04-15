
#include "main.h"
#include "stdio.h"
#include "stdbool.h"
#include "DFPLAYER_MINI.h"
extern UART_HandleTypeDef huart3;
#define DF_UART &huart3


#define Source      0x02  // TF CARD

/*************************************** NO CHANGES AFTER THIS *************************************************/
bool isPause = true;
bool isPlaying = false;

# define Start_Byte 0x7E
# define End_Byte   0xEF
# define Version    0xFF
# define Cmd_Len    0x06
# define Feedback   0x00    //If need for Feedback: 0x01,  No Feedback: 0
void Send_cmd (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2)
{
	uint16_t Checksum = Version + Cmd_Len + cmd + Feedback + Parameter1 + Parameter2;
	Checksum = 0-Checksum;

	uint8_t CmdSequence[10] = { Start_Byte, Version, Cmd_Len, cmd, Feedback, Parameter1, Parameter2, (Checksum>>8)&0x00ff, (Checksum&0x00ff), End_Byte};

	HAL_UART_Transmit(DF_UART, CmdSequence, 10, HAL_MAX_DELAY);
}

void DF_PlayFromStart(void)
{
  Send_cmd(0x03, 0x00, 1);
  HAL_Delay(300);
}

void DF_Init (uint8_t volume)
{
	Send_cmd(0x3F, 0x00, Source);
	HAL_Delay(200);
	DF_volume_setup(volume);
}
void DF_Reset (void)
{
	Send_cmd(0x0C, 0, 0);
	HAL_Delay(200);
}

void DF_Next (void)
{
	Send_cmd(0x01, 0x00, 0x00);
	HAL_Delay(100);
}

void DF_Pause (void)
{
	Send_cmd(0x0E, 0, 0);
	HAL_Delay(100);
}

void DF_Previous (void)
{
	Send_cmd(0x02, 0, 0);
	HAL_Delay(100);
}

void DF_Playback (void)
{
	Send_cmd(0x0D, 0, 0);
	HAL_Delay(100);
}
void DF_volume_up (void) {
	Send_cmd(0x04, 0, 0);
	HAL_Delay(100);
}
void DF_volume_down (void) {
	Send_cmd(0x05, 0, 0);
	HAL_Delay(100);
}
void DF_stop_play (void) {
	Send_cmd(0x16, 0, 0);
	HAL_Delay(100);
}
void DF_design (uint8_t song_num) {
	Send_cmd(0x03, 0x00, song_num);
	HAL_Delay(100);
}
void DF_volume_setup(uint8_t volume) {
	Send_cmd(0x06, 0x00, volume);
	HAL_Delay(500);
}