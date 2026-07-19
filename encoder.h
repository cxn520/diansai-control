#ifndef __encoder_h_
#define __encoder_h_

#include "main.h"

/*
 * 行走距离参数：轮径 65 mm，当前程序只统计编码器 A 相上升沿，
 * 因此 13 PPR 对应每个编码器轴转一圈产生 13 个软件计数。
 */
#define WHEEL_DIAMETER_CM                  (6.5f)
#define ENCODER_PULSES_PER_REV             (13.0f)
#define MOTOR_GEAR_RATIO                   (28.0f)
#define ENCODER_COUNTS_PER_WHEEL_REV       \
    (ENCODER_PULSES_PER_REV * MOTOR_GEAR_RATIO)
#define DISTANCE_PER_ENCODER_COUNT_CM      \
    (3.1415926f * WHEEL_DIAMETER_CM / ENCODER_COUNTS_PER_WHEEL_REV)

extern volatile int32_t Get_encoder_Left,Get_encoder_Right;

/* distance 的单位为 cm，正数表示前进，负数表示后退。 */
extern volatile float distance;

void Distance_Update(int32_t left_count, int32_t right_count);
void Distance_Reset(void);
void Oled_Task();
#endif
