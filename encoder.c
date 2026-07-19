#include "encoder.h"
volatile int32_t Get_encoder_Left=0;
volatile int32_t Get_encoder_Right=0;

/* 小车累计行走距离，单位：cm。 */
volatile float distance = 0.0f;

void Distance_Update(int32_t left_count, int32_t right_count)
{
    float average_count = ((float)left_count + (float)right_count) * 0.5f;
    distance += average_count * DISTANCE_PER_ENCODER_COUNT_CM;
}

void Distance_Reset(void)
{
    distance = 0.0f;
}

uint8_t oled_buffer[32];

void Oled_Task()
{
    OLED_Refresh();
    
    // ================== 1. 显示 Left 编码器 ==================
    OLED_ShowString(0, 0, "Left:", 8, 0);
    if (Encoder_left < 0) {
        OLED_ShowString(34, 0, "-", 8, 0); // 负数：手动打印负号
        OLED_ShowNum(40, 0, (uint32_t)(-Encoder_left), 4, 8, 0); // 传绝对值
    } else {
        OLED_ShowString(34, 0, " ", 8, 0); // 正数：空格覆盖掉之前的负号残影
        OLED_ShowNum(40, 0, (uint32_t)Encoder_left, 4, 8, 0);
    }

    // ================== 2. 显示 Right 编码器 ==================
    OLED_ShowString(0, 8, "Right:", 8, 0);
    if (Encoder_right < 0) {
        OLED_ShowString(34, 8, "-", 8, 0);
        OLED_ShowNum(40, 8, (uint32_t)(-Encoder_right), 4, 8, 0);
    } else {
        OLED_ShowString(34, 8, " ", 8, 0);
        OLED_ShowNum(40, 8, (uint32_t)Encoder_right, 4, 8, 0);
    }

    // ================== 3. 显示 PWM ==================
    // 注意：经过之前的 PID 优化，pwmout 目前被限制在 0~100，绝对不会有负数，直接显示
    OLED_ShowString(0, 16, "Pwm_Left:", 8, 0);
    OLED_ShowNum(56, 16, (int32_t)pwmout_left, 3, 8, 0);
    
    OLED_ShowString(0, 24, "Pwm_Right:", 8, 0);
    OLED_ShowNum(56, 24, (int32_t)pwmout_right, 3, 8, 0);

    sprintf((char *)oled_buffer, "%-6.1f", wit_data.pitch);
    OLED_ShowString(5*8,32,oled_buffer,8,0);
    sprintf((char *)oled_buffer, "%-6.1f", wit_data.roll);
    OLED_ShowString(5*8,40,oled_buffer,8,0);
    sprintf((char *)oled_buffer, "%-6.1f", wit_data.yaw);
    OLED_ShowString(5*8,48,oled_buffer,8,0);
}



/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
void GROUP1_IRQHandler(void)
{
    int32_t gpioB_R=DL_GPIO_getEnabledInterruptStatus(GPIOB,ENCODER_RIGHT_E2A_PIN);
    int32_t gpioB_L=DL_GPIO_getEnabledInterruptStatus(GPIOB,ENCODER_LEFT_E1A_PIN);
    if( gpioB_L & ENCODER_LEFT_E1A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_LEFT_PORT, ENCODER_LEFT_E1B_PIN))
        {
            Get_encoder_Left++;
        }
        else 
        {
            Get_encoder_Left--;
        }
        DL_GPIO_clearInterruptStatus(GPIOB,ENCODER_LEFT_E1A_PIN);
    }
    if(  gpioB_R & ENCODER_RIGHT_E2A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_RIGHT_PORT, ENCODER_RIGHT_E2B_PIN))
        {
            Get_encoder_Right++;
        }
        else 
        {
            Get_encoder_Right--;
        }
        DL_GPIO_clearInterruptStatus(GPIOB,ENCODER_RIGHT_E2A_PIN);
    }
    

}
