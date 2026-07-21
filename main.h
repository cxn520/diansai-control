#ifndef __main_h__
#define __main_h__

#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "System/oled.h"
#include "System/board.h"
#include "encoder.h"
#include "System/Emm_V5.h"
#include "car.h"
#include "System/wit.h"
#include "interrupt.h"
#include "LineTrack.h"
#include "stepper.h"
#include "camera.h"
#include <math.h>
extern volatile int target_speed_left,target_speed_right;
extern float pwmout_left,pwmout_right;
extern volatile int LineTrack_Flag;
extern volatile int Yaw_Flag;
extern int mode;
extern uint8_t Buzz_flag;
extern volatile bool turn_flag;
extern volatile int turn_cnt;
#endif
