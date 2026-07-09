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
#include "oled.h"
#include "board.h"
#include "encoder.h"
#include "Emm_V5.h"
#include "car.h"
#include "wit.h"
#include "interrupt.h"
#include "LineTrack.h"
#include "stepper.h"

extern int target_speed_left,target_speed_right;
extern float pwmout_left,pwmout_right;
extern int LineTrack_Flag; 
extern int Yaw_Flag;
#endif


