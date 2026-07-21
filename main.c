/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "main.h"
volatile int target_speed_left =0;
volatile int target_speed_right=0;
volatile int LineTrack_Flag=0;
volatile int Yaw_Flag=0;
volatile bool turn_flag=0;
volatile int turn_cnt=0;
int mode=0;
int angle=0;
#define RECTANGLE_TURN_LINE_ARM_ERROR_DEG (35.0f)
extern void Camera_Init(void);
extern void Camera_PID_Run(void);
int main(void)
{
    SYSCFG_DL_init();

   
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);//使能timer
    DL_Timer_startCounter(TIMER_0_INST);//开始计数

    DL_Timer_startCounter(PWM_Left_INST);//开始左轮
    DL_Timer_startCounter(PWM_Right_INST);//开始右轮

    NVIC_EnableIRQ(GPIOB_INT_IRQn); //使能gpioB的中断
    WIT_Init();
    Interrupt_Init();
    OLED_Init();
    OLED_ColorTurn(0);//0正常显示，1 反色显示

    /* 串口2 - 步进电机通信 (PB15/PB16, RS485总线) */
    DL_UART_Main_enable(UART_STEPPER_INST);
    DL_UART_Main_enable(UART_CAMERA_INST);
    Camera_Init();
    Stepper_Init();
    delay_ms(200); 
    /* 上电主动回零；Stepper_Home 内部使用非同步命令，发送后立即执行。 */
    Stepper_Home(STEPPER_ADDR_CHASSIS, HOME_MODE_NEAR);
    delay_ms(50);
    Stepper_Home(STEPPER_ADDR_LIFT, HOME_MODE_NEAR);
    delay_ms(1000);
    while (1) 
    {
        Camera_PID_Run(); 
        Oled_Task();
        LineTrack();
        if(mode==1)
        {
          LineTrack_Flag=1;
          target_speed_left=5;
          target_speed_right=5;
          if ((hw1 == 1) && (hw2 == 1) && (hw3 == 1) && (hw4 == 1) &&(hw5 == 1) && (hw6 == 1) && (hw7 == 1) && (hw8 == 1)) //全部识别到白色 切换模式
          {
             Camera_SignalPrepareTurn(45.0f, 1.0f);
              target_speed_left=0;
              target_speed_right=0;
              turn_flag=1;
              mode=2;
          }
        }
        if(mode==2)
        {
           
            if(turn_cnt>=500)
            {
              turn_flag=0;
              turn_cnt=0;
              mode=3;
            }
        }
        if(mode==3)
        {
          LineTrack_Flag=1;
          target_speed_left=0;
          target_speed_right=3;
           if((hw1 == 0) || (hw2 == 0) || (hw3 == 0) || (hw4 == 0) ||(hw5 == 0) || (hw6 == 0) || (hw7 == 0) || (hw8 == 0)) //识别到任意黑色
            {
                 mode=1;
            }
        }
    }
}
