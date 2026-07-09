#ifndef _car_h_
#define _car_h_
#include "main.h"

extern int style;
extern int32_t OLED_leftpwm,OLED_rightpwm;
extern volatile int Encoder_left,Encoder_right;
//========================PMW===========================
void Set_duty(float duty,uint8_t channel);
void Set_frquency(uint32_t freq,uint8_t channel);

void Set_PWM_Left(float pwmleft);
void Set_PWM_Right(float pwmright);
//========================小车控制及PID===========================
void CarStop();
void Motor_Dir_Init(void);  // 电机方向引脚初始化
int Velocity_L(int32_t target_Left,int32_t encounter_Left);
int Velocity_R(int32_t target_right, int32_t encounter_right);
extern float target_angle_offset ;
//=========================================================


#endif
