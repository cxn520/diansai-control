#ifndef _camera_h_
#define _camera_h_
#include "main.h"

void Camera_Init(void);
void Camera_PID_Reset(void);
void Camera_PID_Run(void);
void Camera_ControlTick1ms(void);
void Camera_Feedforward_Update(float distance_cm, uint8_t straight_active);
void Camera_Feedforward_Rearm(void);
void Camera_SignalPrepareTurn(float degrees, float sign);

/* CCS Expressions中可直接观察“摄像头误差+IMU误差→PID”的完整链路。 */
extern volatile float camera_imu_angle_debug;
extern volatile float camera_imu_delta_debug;
extern volatile int32_t camera_imu_error_debug;
extern volatile int32_t camera_combined_x_error_debug;
extern volatile int32_t camera_angle_output_debug;
extern volatile float camera_feedforward_distance_debug;
extern volatile int32_t camera_feedforward_error_debug;
extern volatile uint8_t camera_feedforward_triggered_debug;
extern volatile uint8_t camera_imu_suppressed_debug;
extern volatile uint8_t camera_target_valid;

#endif
