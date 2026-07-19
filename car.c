#include "car.h"
//实际脉冲数量
volatile int Encoder_left ,Encoder_right;

//OLED显示的PWM值
int32_t OLED_leftpwm,OLED_rightpwm;

uint32_t period=32000;
// =================================左轮 (位置式) ===========================
float error_left = 0, last_error_left = 0, integral_left = 0;

// 【注意】位置式的参数量级与增量式完全不同，必须重新调参！
// 参考起始值：P决定基础推力，I消除静差，D抑制震荡。
float Positional_KP_left = 1.5, Positional_KI_left = 0.1, Positional_KD_left = 0;

// ==================================右轮 (位置式) ==================================
float error_right = 0, last_error_right = 0, integral_right = 0;

// 【注意】位置式的参数量级与增量式完全不同，必须重新调参！
float Positional_KP_right =1.5, Positional_KI_right = 0.1, Positional_KD_right =0;


/***************************************************************************
函数功能：电机的 位置式PID 闭环控制
入口参数：目标速度，当前编码器值
返回值  ：电机的PWM (绝对无负数)
***************************************************************************/
int Velocity_L(int32_t target_left, int32_t encounter_left) {
    float out_pwm_L;
    
    // 1. 计算当前误差 (比例项)
    error_left = target_left - encounter_left; 
    
    // 2. 误差累加 (积分项)
    integral_left += error_left;
    
    // 【核心保护：积分抗饱和限幅】防止积分项无限变大导致刹不住车
    // 假设 KI 是 0.1，我们限制积分项最多只能提供 80 的 PWM 输出 (80 / 0.1 = 800)
    if(integral_left > 800) integral_left = 800;
    if(integral_left < -800) integral_left = -800;
    
    // 3. 位置式 PID 公式计算输出
    // Out = Kp * e(k) + Ki * ∑e + Kd * (e(k) - e(k-1))
    out_pwm_L = (Positional_KP_left * error_left) + 
                (Positional_KI_left * integral_left) + 
                (Positional_KD_left * (error_left - last_error_left));
    
    // 4. 更新上次误差，供下次微分使用
    last_error_left = error_left;
    
    // 【限制】：严格控制在 0 ~ 100 之间，无负数
    if (out_pwm_L > 100) out_pwm_L = 100;
    if (out_pwm_L < 0) out_pwm_L = 0; 
    
    return (int)out_pwm_L;  
}

int Velocity_R(int32_t target_right, int32_t encounter_right) {
    float out_pwm_R;
    
    // 1. 计算当前误差 (比例项)
    error_right = target_right - encounter_right; 
    
    // 2. 误差累加 (积分项)
    integral_right += error_right;
    
    // 【核心保护：积分抗饱和限幅】
    if(integral_right > 800) integral_right = 800;
    if(integral_right < -800) integral_right = -800;
    
    // 3. 位置式 PID 公式计算输出
    out_pwm_R = (Positional_KP_right * error_right) + 
                (Positional_KI_right * integral_right) + 
                (Positional_KD_right * (error_right - last_error_right));
    
    // 4. 更新上次误差，供下次微分使用
    last_error_right = error_right;
    
    // 【限制】：严格控制在 0 ~ 100 之间，无负数
    if (out_pwm_R > 100) out_pwm_R = 100;
    if (out_pwm_R < 0) out_pwm_R = 0; 
    
    return (int)out_pwm_R;    
}
// ================================= 角度环 (位置式) ===========================
float error_angle = 0, last_error_angle = 0, integral_angle = 0;

// 【参数建议】因为不能反转，转向效率会变低，P值可能需要适当调大一点点
float Angle_KP =0.25, Angle_KI = 0, Angle_KD = 0.2;
float angle_derivative_filtered = 0;
const float angle_derivative_alpha = 0.15f;

float initial_yaw = 0.0f;
uint8_t yaw_initialized = 0;
float target_angle_offset = 0.0f;


/***************************************************************************
函数功能：角度位置式PID闭环计算
入口参数：目标偏转角度(相对上电)，当前偏航角(Yaw)
返回值  ：转向修正速度 (带符号的差速值)
***************************************************************************/
int Angle_Loop(float target_offset, float current_yaw) {
    float out_turn;
    
    // 1. 上电第一次调用时，记录初始偏航角
    if (!yaw_initialized) {
        initial_yaw = current_yaw;
        yaw_initialized = 1;
    }
    
    // 2. 计算绝对目标角度
    float target_yaw = initial_yaw + target_offset;
    
    // 3. 计算误差 (处理优弧劣弧)
    error_angle = target_yaw - current_yaw;
    while (error_angle > 180.0f)  error_angle -= 360.0f;
    while (error_angle < -180.0f) error_angle += 360.0f;
    
    // 4. 误差累加 (积分项)
    integral_angle += error_angle;
    if(integral_angle > 500) integral_angle = 500;
    if(integral_angle < -500) integral_angle = -500;
    
    // 5. 计算微分项 (防止角度回卷导致微分暴冲)
    float error_diff = error_angle - last_error_angle;
    while (error_diff > 180.0f)  error_diff -= 360.0f;
    while (error_diff < -180.0f) error_diff += 360.0f;

    // 6. 低通滤波微分项 (抑制传感器高频噪声)
    angle_derivative_filtered = angle_derivative_alpha * error_diff +
                               (1.0f - angle_derivative_alpha) * angle_derivative_filtered;

    // 7. PID 计算输出
    out_turn = (Angle_KP * error_angle) +
               (Angle_KI * integral_angle) +
               (Angle_KD * angle_derivative_filtered);

    // 8. 更新上次误差
    last_error_angle = error_angle;
    
    // 【限制转向输出幅度】由于单轮无法反转，转向差速不需要给太大，否则容易单侧暴冲
    if (out_turn > 50) out_turn = 50;
    if (out_turn < -50) out_turn = -50;
    
    return (int)out_turn;
}
//=================================================================设置=============================
void Set_duty(float duty,uint8_t channel) //设置pwm周期
{
    uint32_t comparevalue;
    if(duty>=1) duty=0.99;
    comparevalue=period-period*(1-duty);
    if(channel==0)
    DL_TimerG_setCaptureCompareValue(PWM_Left_INST,comparevalue, DL_TIMER_CC_0_INDEX);
    else if(channel==1)
    DL_TimerG_setCaptureCompareValue(PWM_Right_INST, comparevalue, DL_TIMER_CC_0_INDEX);
}

//=========================================================================================================

//CARCONTROL
void Set_PWM_Left(float pwmleft)
{
  DL_GPIO_setPins(CAR_PORT,CAR_BIN2_PIN);
  Set_duty(pwmleft/100.0,0);
}

void Set_PWM_Right(float pwmright)
{
  DL_GPIO_setPins(CAR_PORT,CAR_AIN2_PIN);
  Set_duty(pwmright/100.0,1);
}



void CarStop()
{
    Set_duty(0,0);
    Set_duty(0,1);
    DL_GPIO_clearPins(CAR_PORT,CAR_BIN2_PIN);
    DL_GPIO_clearPins(CAR_PORT,CAR_AIN2_PIN);
/* ---------- 强制冷启动 ---------- */
    target_speed_left = 0;
    target_speed_right = 0;
 
}
float pwmout_left,pwmout_right;//实际输出的pwm值

uint8_t cnt=0;

void TIMER_0_INST_IRQHandler()
{
    DL_TimerA_clearInterruptStatus(
        TIMER_0_INST, DL_TIMERA_INTERRUPT_ZERO_EVENT);

    cnt++;
    if(cnt >= 20)
    {     
        cnt = 0;
        
        // 1. 直接获取并清零编码器，防止竞态条件导致丢步
        // 假设这里关一下中断或者使用原子操作更佳，如果这是定时器读取则无需担心
        Encoder_left = Get_encoder_Left;
        Get_encoder_Left = 0;
        Encoder_right = Get_encoder_Right; 
        Get_encoder_Right = 0;

        /* 根据左右轮本周期平均脉冲数更新累计行走距离。 */
        Distance_Update(Encoder_left, Encoder_right);

        // 2. 赋予基础期望速度
        int16_t target_L = target_speed_left;
        int16_t target_R = target_speed_right;

        // 3. 外环控制 (角度环与循迹环互斥执行，或者根据需求叠加)
        if ((Yaw_Flag == 1)&&(LineTrack_Flag == 0))
        {
            float current_yaw = wit_data.yaw;
            int turn_speed = Angle_Loop(target_angle_offset, current_yaw);
            target_L -= turn_speed;   
            target_R += turn_speed;
        }
        else if ((LineTrack_Flag == 1)&&(Yaw_Flag == 0))
        {
            int8_t err = LinePID_Error();
            int16_t Line_pid = LinePID_Calc(err);
            target_L =target_L - Line_pid;  
            target_R =target_R + Line_pid;                
        }

   
        if (target_L < 0) target_L = 0;
        if (target_R < 0) target_R = 0;

        // 5. 传入速度环 PID 进行闭环计算
        pwmout_left = Velocity_L(target_L, Encoder_left);
        pwmout_right = Velocity_R(target_R, Encoder_right);

        // 6. 输出 PWM
        Set_PWM_Left(pwmout_left);
        Set_PWM_Right(pwmout_right);
    }
}


