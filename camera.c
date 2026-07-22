#include "main.h"

/*
 * 摄像头帧格式：$x-123y-12v1#。
 * x、y 均为相对于画面中心的有符号原始偏差，v 表示目标是否有效。
 * 为兼容串口测试，v 可省略，例如 $x10y20# 默认按有效目标处理。
 */

#define CAMERA_RX_BUFFER_SIZE              (24U)
#define CAMERA_COORDINATE_LIMIT            (10000)
#define CAMERA_TARGET_TIMEOUT_MS           (200U)

/*
 * 摄像头误差首次进入停止区时，立即取消驱动器尚未完成的相对运动。
 * 急停不重置 IMU 基准，后续仍持续用 IMU 航向变化进行世界方向锁定。
 */
#define CAMERA_STOP_THRESHOLD              (20)
#define CAMERA_IMU_LOOP_PERIOD_MS          (10U)
#define CAMERA_X_PULSES_PER_PIXEL          (5.0f)
#define CAMERA_Y_PULSES_PER_PIXEL          (0.5f)
#define CAMERA_PAN_PULSES_PER_DEGREE       (200.0f)
#define CAMERA_IMU_COMPENSATION_SIGN       (1.0f)
#define CAMERA_X_CAMERA_LIMIT_PULSES       (400.0f)
#define CAMERA_Y_CAMERA_LIMIT_PULSES       (400.0f)
#define CAMERA_X_IMU_LIMIT_PULSES          (36000.0f)
#define CAMERA_X_POSITIVE_DIRECTION        STEPPER_DIR_CW
#define CAMERA_X_NEGATIVE_DIRECTION        STEPPER_DIR_CCW
#define CAMERA_Y_POSITIVE_DIRECTION        STEPPER_DIR_CW
#define CAMERA_Y_NEGATIVE_DIRECTION        STEPPER_DIR_CCW

/* UART 中断与主循环之间共享的最新摄像头帧，仅限本模块使用。 */
static volatile int32_t s_camera_x = 0;
static volatile int32_t s_camera_y = 0;
static volatile uint8_t s_camera_target_valid = 0U;
static volatile uint8_t s_camera_new_frame = 0U;
static volatile uint16_t s_camera_target_age_ms =
    CAMERA_TARGET_TIMEOUT_MS + 1U;

/* UART 帧解析状态，仅在摄像头 UART 中断中访问。 */
static char s_camera_rx_buffer[CAMERA_RX_BUFFER_SIZE];
static uint8_t s_camera_rx_length = 0U;
static uint8_t s_camera_receiving = 0U;

/* 控制状态只保留“是否有未完成运动”和 IMU 最新基准。 */
static uint8_t s_camera_x_moving = 0U;
static uint8_t s_camera_y_moving = 0U;
static uint8_t s_camera_had_valid_target = 0U;
static uint8_t s_camera_x_in_stop_zone = 0U;
static uint8_t s_camera_y_in_stop_zone = 0U;
static uint8_t s_camera_timeout_active = 1U;
static volatile uint16_t s_camera_imu_elapsed_ms = 0U;
static float s_camera_last_imu_yaw = 0.0f;
static uint8_t s_camera_imu_initialized = 0U;

static int32_t Camera_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float Camera_Wrap180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static int32_t Camera_LimitAndRound(float value, float limit)
{
    if (value > limit)
    {
        value = limit;
    }
    else if (value < -limit)
    {
        value = -limit;
    }

    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static void Camera_ResetRxState(void)
{
    s_camera_rx_length = 0U;
    s_camera_receiving = 0U;
}

static uint8_t Camera_ParseSignedValue(uint8_t *index, int32_t *value)
{
    int32_t result = 0;
    uint8_t negative = 0U;
    uint8_t digit_count = 0U;

    if (*index >= s_camera_rx_length)
    {
        return 0U;
    }

    if (s_camera_rx_buffer[*index] == '-')
    {
        negative = 1U;
        (*index)++;
    }
    else if (s_camera_rx_buffer[*index] == '+')
    {
        (*index)++;
    }

    while ((*index < s_camera_rx_length) &&
           (s_camera_rx_buffer[*index] >= '0') &&
           (s_camera_rx_buffer[*index] <= '9'))
    {
        int32_t digit = (int32_t)(s_camera_rx_buffer[*index] - '0');

        if (result > ((CAMERA_COORDINATE_LIMIT - digit) / 10))
        {
            return 0U;
        }

        result = result * 10 + digit;
        (*index)++;
        digit_count++;
    }

    if (digit_count == 0U)
    {
        return 0U;
    }

    *value = (negative != 0U) ? -result : result;
    return 1U;
}

static uint8_t Camera_ParseFrame(void)
{
    uint8_t index = 0U;
    int32_t parsed_x;
    int32_t parsed_y;
    int32_t parsed_valid = 1;

    if ((s_camera_rx_length < 4U) ||
        (s_camera_rx_buffer[index] != 'x'))
    {
        return 0U;
    }

    index++;
    if (Camera_ParseSignedValue(&index, &parsed_x) == 0U)
    {
        return 0U;
    }

    if ((index >= s_camera_rx_length) ||
        (s_camera_rx_buffer[index] != 'y'))
    {
        return 0U;
    }

    index++;
    if (Camera_ParseSignedValue(&index, &parsed_y) == 0U)
    {
        return 0U;
    }

    if (index < s_camera_rx_length)
    {
        if (s_camera_rx_buffer[index] != 'v')
        {
            return 0U;
        }

        index++;
        if ((Camera_ParseSignedValue(&index, &parsed_valid) == 0U) ||
            ((parsed_valid != 0) && (parsed_valid != 1)))
        {
            return 0U;
        }
    }

    if (index != s_camera_rx_length)
    {
        return 0U;
    }

    s_camera_x = parsed_x;
    s_camera_y = parsed_y;
    s_camera_target_valid = (uint8_t)parsed_valid;
    s_camera_target_age_ms = 0U;
    s_camera_new_frame = 1U;
    return 1U;
}

static void Camera_ProcessRxByte(uint8_t byte)
{
    if (byte == '$')
    {
        s_camera_rx_length = 0U;
        s_camera_receiving = 1U;
        return;
    }

    if (s_camera_receiving == 0U)
    {
        return;
    }

    if (byte == '#')
    {
        (void)Camera_ParseFrame();
        Camera_ResetRxState();
        return;
    }

    if (s_camera_rx_length < (CAMERA_RX_BUFFER_SIZE - 1U))
    {
        s_camera_rx_buffer[s_camera_rx_length] = (char)byte;
        s_camera_rx_length++;
    }
    else
    {
        Camera_ResetRxState();
    }
}

static void Camera_StopAxis(uint8_t address, uint8_t *moving)
{
    if (*moving != 0U)
    {
        Stepper_Stop(address);
        *moving = 0U;
    }
}

static void Camera_StopAllAxes(void)
{
    Camera_StopAxis(STEPPER_ADDR_CHASSIS, &s_camera_x_moving);
    Camera_StopAxis(STEPPER_ADDR_LIFT, &s_camera_y_moving);
}

static int32_t Camera_GetImuCommand(uint8_t loop_due)
{
    float current_yaw;
    float yaw_delta;

    if (loop_due == 0U)
    {
        return 0;
    }

    current_yaw = wit_data.yaw;
    if (s_camera_imu_initialized == 0U)
    {
        s_camera_last_imu_yaw = current_yaw;
        s_camera_imu_initialized = 1U;
        return 0;
    }

    yaw_delta = Camera_Wrap180(current_yaw - s_camera_last_imu_yaw);
    s_camera_last_imu_yaw = current_yaw;

    return Camera_LimitAndRound(
        CAMERA_IMU_COMPENSATION_SIGN * yaw_delta *
            CAMERA_PAN_PULSES_PER_DEGREE,
        CAMERA_X_IMU_LIMIT_PULSES);
}

static void Camera_RunTracking(int32_t x_error, int32_t y_error,
                               uint8_t new_frame, uint8_t target_valid,
                               int32_t imu_command)
{
    int32_t x_command = imu_command;
    int32_t y_command = 0;
    uint8_t command_queued = 0U;

    if (new_frame != 0U)
    {
        if (target_valid == 0U)
        {
            if (s_camera_had_valid_target != 0U)
            {
                /* 目标刚丢失时只清除一次旧视觉位移，随后允许 IMU 接管。 */
                Camera_StopAllAxes();
                s_camera_had_valid_target = 0U;
                s_camera_x_in_stop_zone = 0U;
                s_camera_y_in_stop_zone = 0U;
                return;
            }
        }
        else
        {
            s_camera_had_valid_target = 1U;
            if (Camera_Abs32(x_error) <= CAMERA_STOP_THRESHOLD)
            {
                /*
                 * 只在首次进入中心区时取消未完成的视觉位移。
                 * 锁存后不再停止水平轴，确保后续 IMU 补偿可以持续执行。
                 */
                if (s_camera_x_in_stop_zone == 0U)
                {
                    Camera_StopAxis(STEPPER_ADDR_CHASSIS,
                                    &s_camera_x_moving);
                    x_command = 0;
                    s_camera_x_in_stop_zone = 1U;
                }
            }
            else
            {
                int32_t camera_x_command = Camera_LimitAndRound(
                    (float)x_error * CAMERA_X_PULSES_PER_PIXEL,
                    CAMERA_X_CAMERA_LIMIT_PULSES);

                x_command = Camera_LimitAndRound(
                    (float)camera_x_command + (float)imu_command,
                    CAMERA_X_IMU_LIMIT_PULSES);
                s_camera_x_in_stop_zone = 0U;
            }

            if (Camera_Abs32(y_error) <= CAMERA_STOP_THRESHOLD)
            {
                if (s_camera_y_in_stop_zone == 0U)
                {
                    Camera_StopAxis(STEPPER_ADDR_LIFT,
                                    &s_camera_y_moving);
                    s_camera_y_in_stop_zone = 1U;
                }
            }
            else
            {
                y_command = Camera_LimitAndRound(
                    (float)y_error * CAMERA_Y_PULSES_PER_PIXEL,
                    CAMERA_Y_CAMERA_LIMIT_PULSES);
                s_camera_y_in_stop_zone = 0U;
            }
        }
    }

    if (x_command != 0)
    {
        Stepper_Queue_Relative(STEPPER_ADDR_CHASSIS,
                               (x_command > 0) ? CAMERA_X_POSITIVE_DIRECTION :
                                                 CAMERA_X_NEGATIVE_DIRECTION,
                               (uint32_t)Camera_Abs32(x_command));
        s_camera_x_moving = 1U;
        command_queued = 1U;
    }

    if (y_command != 0)
    {
        Stepper_Queue_Relative(STEPPER_ADDR_LIFT,
                               (y_command > 0) ? CAMERA_Y_POSITIVE_DIRECTION :
                                                 CAMERA_Y_NEGATIVE_DIRECTION,
                               (uint32_t)Camera_Abs32(y_command));
        s_camera_y_moving = 1U;
        command_queued = 1U;
    }

    if (command_queued != 0U)
    {
        Stepper_Sync_Move(STEPPER_ADDR_BROADCAST);
    }
}

void Camera_Init(void)
{
    Camera_ResetRxState();
    s_camera_new_frame = 0U;
    s_camera_x = 0;
    s_camera_y = 0;
    s_camera_target_valid = 0U;
    s_camera_target_age_ms = CAMERA_TARGET_TIMEOUT_MS + 1U;
    s_camera_x_moving = 0U;
    s_camera_y_moving = 0U;
    s_camera_had_valid_target = 0U;
    s_camera_x_in_stop_zone = 0U;
    s_camera_y_in_stop_zone = 0U;
    s_camera_timeout_active = 1U;
    s_camera_imu_elapsed_ms = 0U;
    s_camera_last_imu_yaw = 0.0f;
    s_camera_imu_initialized = 0U;

    while (DL_UART_Main_isRXFIFOEmpty(UART_CAMERA_INST) == false)
    {
        (void)DL_UART_Main_receiveData(UART_CAMERA_INST);
    }

    NVIC_ClearPendingIRQ(UART_CAMERA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_CAMERA_INST_INT_IRQN);
}

void Camera_ControlTick1ms(void)
{
    if (s_camera_imu_elapsed_ms < CAMERA_IMU_LOOP_PERIOD_MS)
    {
        s_camera_imu_elapsed_ms++;
    }

    if (s_camera_target_age_ms <= CAMERA_TARGET_TIMEOUT_MS)
    {
        s_camera_target_age_ms++;
    }
}

void Camera_Track_Run(void)
{
    uint32_t primask;
    int32_t x_error = 0;
    int32_t y_error = 0;
    uint16_t target_age_ms;
    uint8_t new_frame;
    uint8_t target_valid;
    uint8_t imu_loop_due;
    int32_t imu_command;

    /* 短临界区确保主循环读取的是同一帧 x/y 和对应有效状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    new_frame = s_camera_new_frame;
    target_valid = s_camera_target_valid;
    target_age_ms = s_camera_target_age_ms;
    imu_loop_due = (s_camera_imu_elapsed_ms >=
                    CAMERA_IMU_LOOP_PERIOD_MS) ? 1U : 0U;
    if (imu_loop_due != 0U)
    {
        s_camera_imu_elapsed_ms = 0U;
    }
    if (new_frame != 0U)
    {
        x_error = s_camera_x;
        y_error = s_camera_y;
        s_camera_new_frame = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    imu_command = Camera_GetImuCommand(imu_loop_due);

    if (target_age_ms > CAMERA_TARGET_TIMEOUT_MS)
    {
        if (s_camera_timeout_active == 0U)
        {
            /* 只清除一次超时前的视觉位移，之后水平轴继续由 IMU 锁向。 */
            Camera_StopAllAxes();
            s_camera_had_valid_target = 0U;
            s_camera_x_in_stop_zone = 0U;
            s_camera_y_in_stop_zone = 0U;
            s_camera_timeout_active = 1U;
            return;
        }

        /* 禁止使用超时的旧坐标，但不能禁止 IMU 补偿。 */
        new_frame = 0U;
        target_valid = 0U;
    }
    else
    {
        s_camera_timeout_active = 0U;
    }

    if ((new_frame != 0U) || (imu_command != 0))
    {
        Camera_RunTracking(x_error, y_error, new_frame,
                           target_valid, imu_command);
    }
}

void UART_CAMERA_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_CAMERA_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_Main_isRXFIFOEmpty(UART_CAMERA_INST) == false)
            {
                Camera_ProcessRxByte(
                    DL_UART_Main_receiveData(UART_CAMERA_INST));
            }
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            DL_UART_Main_clearInterruptStatus(
                UART_CAMERA_INST, DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_BREAK_ERROR:
            DL_UART_Main_clearInterruptStatus(
                UART_CAMERA_INST, DL_UART_MAIN_INTERRUPT_BREAK_ERROR);
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            DL_UART_Main_clearInterruptStatus(
                UART_CAMERA_INST, DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            DL_UART_Main_clearInterruptStatus(
                UART_CAMERA_INST, DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            DL_UART_Main_clearInterruptStatus(
                UART_CAMERA_INST, DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
            Camera_ResetRxState();
            break;

        default:
            break;
    }
}
