
#ifndef INC_DFPLAYER_MINI_H_
#define INC_DFPLAYER_MINI_H_

#include "main.h"
enum song_num{
	song_off,
	song_1 = 2,
	song_2,
	song_3,
	song_4,
	song_5,
	song_6,
	song_7,
	song_8,
	song_9,
	song_10,
};

extern void Send_cmd (uint8_t cmd, uint8_t Parameter1, uint8_t Parameter2);
extern void DF_PlayFromStart(void);
extern void DF_Init (uint8_t volume);
extern void DF_Reset (void);
extern void DF_Next (void);
extern void DF_Pause (void);
extern void DF_Previous (void);
extern void DF_Playback (void);
extern void DF_volume_up (void);
extern void DF_volume_down (void);
extern void DF_stop_play (void);
extern void DF_design (uint8_t song_num);
extern void DF_volume_setup(uint8_t volume);

#endif /* INC_DFPLAYER_MINI_H_ */
