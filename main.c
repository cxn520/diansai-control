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

int target_speed_left =0;
int target_speed_right=0;
int LineTrack_Flag=0; 
int Yaw_Flag=0;
int mode=0;
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
    DL_UART_Main_enable(UART_2_INST);

    Stepper_Init();
    delay_ms(200); 
    Stepper_Move_Relative(STEPPER_ADDR_CHASSIS, STEPPER_DIR_CW, 6400); 
    Stepper_Move_Relative(STEPPER_ADDR_CHASSIS, STEPPER_DIR_CW, 6400); 
    while (1) 
    {
        Oled_Task();
        LineTrack();
        if(mode==0)
        {
            Yaw_Flag=0;
            LineTrack_Flag=0;
        }
        
        if(mode==1)  //任务1
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_angle_offset=0;
            target_speed_left=10;
            target_speed_right=10;
           if ((hw1 == 0) || (hw2 == 0) || (hw3 == 0) || (hw4 == 0) ||(hw5 == 0) || (hw6 == 0) || (hw7 == 0) || (hw8 == 0))
                {
                    CarStop();
                    Buzz_flag=1;
                    mode = 0;
                }
        }
        if(mode==2) //任务2-1阶段 直线阶段
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_angle_offset=0;
            target_speed_left=20;
            target_speed_right=20;
           if ((hw1 == 0) || (hw2 == 0) || (hw3 == 0) || (hw4 == 0) ||(hw5 == 0) || (hw6 == 0) || (hw7 == 0) || (hw8 == 0))
                {
                    
                    mode = 3;
                }
        }
        if(mode==3) //任务2-2阶段 循迹阶段 关闭角度环 开启循迹环
        {
            LineTrack_Flag=1;
            Yaw_Flag=0;
            target_speed_left=10;
            target_speed_right=10;
            if ((hw1 == 1) && (hw2 == 1) && (hw3 == 1) && (hw4 == 1) &&(hw5 == 1) && (hw6 == 1) && (hw7 == 1) && (hw8 == 1)) //全部识别到白色 停止
                {
                    CarStop();
                    Buzz_flag=1;
                    mode = 0; //回归开始
                }
        }

        if(mode==4)////任务3-1阶段 转弯37度
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_angle_offset=40;
            target_speed_left=0;
            target_speed_right=0;
            delay_ms(300);
            Distance_Reset();
            mode=5;
        }
        if(mode==5)//任务3-2阶段 直线行驶100cm
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_speed_left=10;
            target_speed_right=10;
            if(distance>=110)
            {
                mode=6;
            }
        }
        if(mode==6)//任务3-3阶段 转回开始的时候 继续直行
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_angle_offset=0;
            target_speed_left=10;
            target_speed_right=10;
            if ((hw1 == 0) || (hw2 == 0) || (hw3 == 0) || (hw4 == 0) ||(hw5 == 0) || (hw6 == 0) || (hw7 == 0) || (hw8 == 0)) //识别到任意黑色 
            {
                mode = 7; //开始
            }
        }
        if(mode==7)//任务3-3阶段 直行时遇到黑线开始 循迹
        {
             Yaw_Flag=0;
             LineTrack_Flag=1;
             target_speed_left=10;
             target_speed_right=10;
            if ((hw1 == 1) && (hw2 == 1) && (hw3 == 1) && (hw4 == 1) &&(hw5 == 1) && (hw6 == 1) && (hw7 == 1) && (hw8 == 1)) //全部识别到白色 停止
                {
                    Buzz_flag=1;
                    mode = 8; 
                }
        }
        if(mode==8)//任务3-4阶段 反向拐弯 角度环开启
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_angle_offset=137;
            target_speed_left=0;
            target_speed_right=0;
            delay_ms(300);
            Distance_Reset();
            mode=9;
        }
        if(mode==9)//任务3-5 反向拐弯后 直线100mm
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_speed_left=10;
            target_speed_right=10;
            if(distance>=115)
            {
                mode=10;
            }
        }
        if(mode==10)//任务3-6 再次拐弯直线行驶
        {
            Yaw_Flag=1;
            LineTrack_Flag=0;
            target_angle_offset=180;
            if ((hw1 == 0) || (hw2 == 0) || (hw3 == 0) || (hw4 == 0) ||(hw5 == 0) || (hw6 == 0) || (hw7 == 0) || (hw8 == 0)) //识别到任意黑色 
            {
                mode = 11; //开始
            }
        }
        if(mode==11)//任务3-7 循迹
        {
            Yaw_Flag=0;
            LineTrack_Flag=1;
            target_speed_left=10;
            target_speed_right=10;
            if ((hw1 == 1) && (hw2 == 1) && (hw3 == 1) && (hw4 == 1) &&(hw5 == 1) && (hw6 == 1) && (hw7 == 1) && (hw8 == 1)) //全部识别到白色 停止
                {
                    Buzz_flag=1;
                    mode = 4; 
                }
        }
    }
}
