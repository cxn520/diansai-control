#include "LineTrack.h"
uint8_t error=0;

uint8_t baixian_FLAG=0;

int hw1,hw2,hw3,hw4,hw5,hw6,hw7,hw8;//黑线0，白线1


// 八路传感器权重表（-4 ~ +4）
static const int8_t SENSOR_WEIGHT[8] = {-6, -4, -2, -1, 1,2 , 4, 6};

// 八路输入 → 误差 (-4~4)
int8_t LinePID_Error();

// 单轮 PID：输入误差 → 差速输出
int16_t LinePID_Calc(int8_t err);


void LineTrack(void)
{
    uint32_t sensor;

    sensor = DL_GPIO_readPins(Track_PORT, 
             Track_in1_PIN |
             Track_in2_PIN |
             Track_in3_PIN |
             Track_in4_PIN |
             Track_in5_PIN |
             Track_in6_PIN |
             Track_in7_PIN |
             Track_in8_PIN);

    hw1 = (sensor & Track_in1_PIN) ? 1 : 0;
    hw2 = (sensor & Track_in2_PIN) ? 1 : 0;
    hw3 = (sensor & Track_in3_PIN) ? 1 : 0;
    hw4 = (sensor & Track_in4_PIN) ? 1 : 0;
    hw5 = (sensor & Track_in5_PIN) ? 1 : 0;
    hw6 = (sensor & Track_in6_PIN) ? 1 : 0;
    hw7 = (sensor & Track_in7_PIN) ? 1 : 0;
    hw8 = (sensor & Track_in8_PIN) ? 1 : 0;
}

int8_t LinePID_Error()
{
    
int8_t err =      hw1 * SENSOR_WEIGHT[0] +
                  hw2 * SENSOR_WEIGHT[1] +
                  hw3 * SENSOR_WEIGHT[2] +
                  hw4 * SENSOR_WEIGHT[3] +
                  hw5 * SENSOR_WEIGHT[4] +
                  hw6 * SENSOR_WEIGHT[5] +
                  hw7 * SENSOR_WEIGHT[6] +
                  hw8 * SENSOR_WEIGHT[7];
    return err;       
}
int16_t LinePID_Calc(int8_t err)
{
    const float kp = 0.8f;   // 仅调这一个
    return (int16_t)(kp * err);
}