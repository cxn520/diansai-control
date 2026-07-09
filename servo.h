#ifndef __SERVO_H__
#define __SERVO_H__

#include "main.h"

/* 舵机编号 */
typedef enum {
    SERVO_1 = 0,    /* PA5, TIMG6 CCP0 */
    SERVO_2,        /* PA6, TIMG6 CCP1 */
    SERVO_3,        /* PA3, TIMG7 CCP0 */
    SERVO_4,        /* PA2, TIMG7 CCP1 */
} ServoID;

/* PWM 参数 (50Hz, 125kHz timer clock) */
#define SERVO_PERIOD        2500        /* 20ms 周期计数值 */
#define SERVO_PULSE_MIN     63          /* 0.5ms (0°) */
#define SERVO_PULSE_MAX     313         /* 2.5ms (180°) */
#define SERVO_PULSE_MID     188         /* 1.5ms (90°) */

void Servo_Init(void);                              /* 初始化所有舵机 */
void Servo_SetAngle(ServoID id, uint8_t angle);     /* 设置角度 (0-180) */
void Servo_SetPulse(ServoID id, uint16_t pulse);    /* 设置脉宽 (计数值, 63-313) */

#endif
