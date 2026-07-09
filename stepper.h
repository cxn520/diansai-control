#ifndef __STEPPER_H__
#define __STEPPER_H__

#include "main.h"
#include "Emm_V5.h"
#include <stdbool.h>

/* 步进电机地址 (RS485总线) */
#define STEPPER_ADDR_CHASSIS    0x01    /* 底盘步进电机 */
#define STEPPER_ADDR_LIFT       0x02   /* 升降步进电机 */

/* 回零模式 */
#define HOME_MODE_NEAR          0       /* 单圈就近回零 */
#define HOME_MODE_DIR           1       /* 单圈方向回零 */
#define HOME_MODE_COLLISION     2       /* 多圈无限位碰撞回零 */
#define HOME_MODE_LIMIT_SWITCH  3       /* 多圈有限位开关回零 */

/* 默认运动参数 */
#define STEPPER_DEFAULT_SPEED    500     /* 默认转速 RPM */
#define STEPPER_DEFAULT_ACC      50      /* 默认加速度 */
#define STEPPER_HOME_SPEED       300     /* 回零转速 RPM */
#define STEPPER_HOME_TIMEOUT     10000   /* 回零超时 ms */

/* 方向 */
#define STEPPER_DIR_CW           0       /* 顺时针 */
#define STEPPER_DIR_CCW          1       /* 逆时针 */

void Stepper_Init(void);                                      /* 初始化所有步进电机 */
void Stepper_Enable(uint8_t addr);                            /* 使能 */
void Stepper_Disable(uint8_t addr);                           /* 失能 */
void Stepper_Home(uint8_t addr, uint8_t mode);                /* 触发回零 */
void Stepper_Home_Interrupt(uint8_t addr);                    /* 中断回零 */
void Stepper_Set_Zero(uint8_t addr, bool save);               /* 设置当前位置为零点 */
void Stepper_Reset_CurPos(uint8_t addr);                      /* 清零当前位置 */
void Stepper_Move_Relative(uint8_t addr, uint8_t dir, uint32_t steps);       /* 相对移动 */
void Stepper_Move_Absolute(uint8_t addr, uint32_t position);                 /* 绝对移动 */
void Stepper_Move_Velocity(uint8_t addr, uint8_t dir, uint16_t speed);       /* 速度模式 */
void Stepper_Stop(uint8_t addr);                              /* 立即停止 */
void Stepper_Sync_Move(uint8_t addr);                         /* 多机同步触发 */
void Stepper_Unclog(uint8_t addr);                            /* 解除堵转保护 */

#endif
