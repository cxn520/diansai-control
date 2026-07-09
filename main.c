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
    while (1) 
    {
        Oled_Task();
        LineTrack();
       
        target_angle_offset=0;
        if(DL_GPIO_readPins(Key_PORT,Key_ONE_PIN )==0)
        {
            DL_GPIO_togglePins(LED_PORT,LED_PIN_22_PIN);
            target_speed_left=2;
            target_speed_right=2;
            LineTrack_Flag=1;
            while(DL_GPIO_readPins(Key_PORT,Key_ONE_PIN )==0);
        }
    }
}
