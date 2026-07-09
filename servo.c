#include "servo.h"

/* 每个舵机的定时器实例和CCP通道索引 */
typedef struct {
    DL_Timer_Reg *timer;            /* TIMG 定时器基地址 */
    DL_Timer_CC_Index ccIndex;      /* CCP 通道索引 */
} ServoHW;

static const ServoHW servo_hw[] = {
    [SERVO_1] = { SERVO_G6_INST, DL_TIMER_CC_0_INDEX },  /* PA5, TIMG6 CCP0 */
    [SERVO_2] = { SERVO_G6_INST, DL_TIMER_CC_1_INDEX },  /* PA6, TIMG6 CCP1 */
    [SERVO_3] = { SERVO_G7_INST, DL_TIMER_CC_0_INDEX },  /* PA3, TIMG7 CCP0 */
    [SERVO_4] = { SERVO_G7_INST, DL_TIMER_CC_1_INDEX },  /* PA2, TIMG7 CCP1 */
};

/***************************************************************************
函数功能：初始化所有舵机（启动定时器）
入口参数：无
返回值  ：无
***************************************************************************/
void Servo_Init(void)
{
    DL_TimerG_startCounter(SERVO_G6_INST);
    DL_TimerG_startCounter(SERVO_G7_INST);
}

/***************************************************************************
函数功能：设置舵机角度
入口参数：id - 舵机编号, angle - 角度 (0-180)
返回值  ：无
***************************************************************************/
void Servo_SetAngle(ServoID id, uint8_t angle)
{
    uint16_t pulse;

    if (angle > 180) angle = 180;

    /* 线性映射: pulse = PULSE_MIN + angle * (PULSE_MAX - PULSE_MIN) / 180 */
    pulse = SERVO_PULSE_MIN + (uint16_t)((uint32_t)angle * (SERVO_PULSE_MAX - SERVO_PULSE_MIN) / 180);

    DL_TimerG_setCaptureCompareValue(servo_hw[id].timer, pulse, servo_hw[id].ccIndex);
}

/***************************************************************************
函数功能：设置舵机脉宽（直接写入比较值）
入口参数：id - 舵机编号, pulse - 计数值 (63-313)
返回值  ：无
***************************************************************************/
void Servo_SetPulse(ServoID id, uint16_t pulse)
{
    if (pulse < SERVO_PULSE_MIN) pulse = SERVO_PULSE_MIN;
    if (pulse > SERVO_PULSE_MAX) pulse = SERVO_PULSE_MAX;

    DL_TimerG_setCaptureCompareValue(servo_hw[id].timer, pulse, servo_hw[id].ccIndex);
}
