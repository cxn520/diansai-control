#include "main.h"

/*
 * 摄像头数据帧格式：$x-123y-12#。
 * x、y 均是相对于画面中心的有符号偏差；本模块以 0 为两个轴的目标位置。
 */

/* --------------------------- 可调参数：串口协议 --------------------------- */
#define CAMERA_RX_BUFFER_SIZE             (24U)    /* 不含起始符和结束符的最大帧长度 */
#define CAMERA_COORDINATE_LIMIT            (10000)  /* 允许接收的 x/y 最大绝对值 */
#define CAMERA_REPLY_ENABLE                (1U)     /* 有效帧回传 "ok\r\n"，用于通信测试 */

/* ----------------------- 可调参数：1 号步进电机（水平） -------------------- */
#define CAMERA_X_PID_KP                    (5.0f)  /* 误差约 100 时接近单帧输出上限 */
#define CAMERA_X_PID_KI                    (0.0f)   /* 相对位移命令自身会累积，关闭I项防止持续移动 */
#define CAMERA_X_PID_KD                    (0.20f)  /* 提高首次响应，同时不过度放大抖动 */
#define CAMERA_X_PID_INTEGRAL_LIMIT        (400.0f)
#define CAMERA_X_PID_OUTPUT_LIMIT          (400.0f) /* 单帧最大相对运动脉冲数 */
#define CAMERA_X_PID_IMU_OUTPUT_LIMIT      (30000.0f) /* 容纳240脉冲/度时完整90°IMU/前馈补偿 */
#define CAMERA_X_DEADBAND                  (2)      /* x 在此范围内不再调整 */
#define CAMERA_X_MIN_PULSE                 (2U)     /* 过小的脉冲命令不发送 */
#define CAMERA_X_POSITIVE_DIRECTION        STEPPER_DIR_CW
#define CAMERA_X_NEGATIVE_DIRECTION        STEPPER_DIR_CCW

/* -------------------- 可调参数：IMU转角补偿（1号水平轴） -------------------
 * IMU相邻采样角度变化先换算成与camera_x相同的等效误差，再与camera_x相加，
 * 合成后的唯一误差才进入水平PID；IMU不再单独产生步进电机命令。
 * CAMERA_PAN_PULSES_PER_DEGREE使用云台实测的“车体每转1°，水平轴所需补偿脉冲数”。
 * IMU角度先按该参数换算成脉冲，再除以水平PID的Kp得到等效camera_x误差。
 */
#define CAMERA_ANGLE_LOOP_PERIOD_MS          (10U)     /* IMU角度变化采样周期 */
#define CAMERA_IMU_ANGLE_SOURCE              (wit_data.yaw)
#define CAMERA_PAN_PULSES_PER_DEGREE         (240.0f)
#define CAMERA_IMU_COMPENSATION_SIGN         (1.0f)   /* 车体转正角度，摄像头误差向负方向补偿 */
#define CAMERA_IMU_ERROR_PER_DEGREE          \
    (CAMERA_PAN_PULSES_PER_DEGREE / CAMERA_X_PID_KP)
#define CAMERA_IMU_PENDING_ERROR_LIMIT       \
    (360.0f * CAMERA_IMU_ERROR_PER_DEGREE)   /* 摄像头帧间可累计一整圈，不丢弃90°转弯 */

/* ---------------------- 可调参数：距离转弯前馈 ---------------------------
 * 直线行驶达到70 cm后，提前给水平轴一个向右90°的前馈误差。
 * 前馈先与camera_x、IMU误差相加，然后统一进入同一个水平PID。
 */
#define CAMERA_FEEDFORWARD_DISTANCE_CM        (70.0f)
#define CAMERA_FEEDFORWARD_TURN_DEG           (90.0f)
#define CAMERA_FEEDFORWARD_SIGN               (1.0f)  /* +1使用水平轴正方向提前右转 */

/* ----------------------- 可调参数：2 号步进电机（上下） -------------------- */
#define CAMERA_Y_PID_KP                    (5.0f)  /* 与水平轴保持一致的像素到脉冲增益 */
#define CAMERA_Y_PID_KI                    (0.0f)   /* 相对位移命令自身会累积，关闭I项防止持续移动 */
#define CAMERA_Y_PID_KD                    (0.20f)
#define CAMERA_Y_PID_INTEGRAL_LIMIT        (400.0f)
#define CAMERA_Y_PID_OUTPUT_LIMIT          (400.0f) /* 单帧最大相对运动脉冲数 */
#define CAMERA_Y_DEADBAND                  (2)      /* y 在此范围内不再调整 */
#define CAMERA_Y_MIN_PULSE                 (2U)     /* 过小的脉冲命令不发送 */
#define CAMERA_Y_POSITIVE_DIRECTION        STEPPER_DIR_CW
#define CAMERA_Y_NEGATIVE_DIRECTION        STEPPER_DIR_CCW

/* 摄像头串口中断与主循环之间共享的最新有效坐标。 */
volatile int32_t camera_x = 0;
volatile int32_t camera_y = 0;
volatile uint8_t camera_new_frame = 0U;
volatile uint8_t camera_reply_pending = 0U;
volatile uint32_t camera_valid_frame_count = 0U;
volatile uint32_t camera_invalid_frame_count = 0U;

/* CCS调试观察量：确认“摄像头误差+IMU等效误差→PID”的实际输入。 */
volatile float camera_imu_angle_debug = 0.0f;
volatile float camera_imu_delta_debug = 0.0f;
volatile int32_t camera_imu_error_debug = 0;
volatile int32_t camera_combined_x_error_debug = 0;
volatile int32_t camera_angle_output_debug = 0;
volatile float camera_feedforward_distance_debug = 0.0f;
volatile int32_t camera_feedforward_error_debug = 0;
volatile uint8_t camera_feedforward_triggered_debug = 0U;

/* 串口中断内部使用的接收状态，不会被主循环直接访问。 */
static char s_camera_rx_buffer[CAMERA_RX_BUFFER_SIZE];
static uint8_t s_camera_rx_length = 0U;
static uint8_t s_camera_receiving = 0U;

/* 两个轴各自独立的 PID 历史量，避免水平、上下控制互相影响。 */
typedef struct
{
    float integral;
    float last_error;
} CameraPIDState;

static CameraPIDState s_camera_x_pid = {0.0f, 0.0f};
static CameraPIDState s_camera_y_pid = {0.0f, 0.0f};

/* IMU变化可快于摄像头帧率，先累计，并在本次10ms控制周期立即合入PID。 */
static volatile uint16_t s_camera_angle_elapsed_ms = 0U;
static float s_camera_last_imu_angle = 0.0f;
static float s_camera_pending_imu_error = 0.0f;
static uint8_t s_camera_imu_angle_initialized = 0U;

/* 距离前馈每段直线只允许触发一次，转弯结束后由主状态机重新使能。 */
static float s_camera_pending_feedforward_error = 0.0f;
static uint8_t s_camera_feedforward_pending = 0U;
static uint8_t s_camera_feedforward_armed = 1U;

/* 供初始化函数调用的公开 PID 状态复位接口。 */
void Camera_PID_Reset(void);

/*
 * @brief 计算 32 位有符号数的绝对值。
 * @note  本模块的输入、输出均已经限幅，不会传入 INT32_MIN。
 */
static int32_t Camera_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/*
 * @brief 将IMU角度差限制到[-180, 180]，避免经过边界时产生错误大跳变。
 */
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

/*
 * @brief 复位串口帧接收状态。
 * @note  遇到超长帧、错误帧或串口错误时丢弃当前帧，等待下一帧 '$' 重新同步。
 */
static void Camera_ResetRxState(void)
{
    s_camera_rx_length = 0U;
    s_camera_receiving = 0U;
}

/*
 * @brief 从当前帧中读取一个有符号十进制数。
 * @param index 传入、返回当前解析位置。
 * @param value 解析成功后的数值。
 * @return 1 表示成功，0 表示格式错误或数值越界。
 */
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

/*
 * @brief 校验并解析完整载荷，例如 "x-123y-12"。
 * @note  本函数只在 UART3 中断内调用，解析长度被严格限制在 24 字节以内。
 */
static uint8_t Camera_ParseFrame(void)
{
    uint8_t index = 0U;
    int32_t parsed_x;
    int32_t parsed_y;

    if ((s_camera_rx_length < 4U) || (s_camera_rx_buffer[index] != 'x'))
    {
        return 0U;
    }

    index++;
    if (Camera_ParseSignedValue(&index, &parsed_x) == 0U)
    {
        return 0U;
    }

    if ((index >= s_camera_rx_length) || (s_camera_rx_buffer[index] != 'y'))
    {
        return 0U;
    }

    index++;
    if (Camera_ParseSignedValue(&index, &parsed_y) == 0U)
    {
        return 0U;
    }

    if (index != s_camera_rx_length)
    {
        return 0U;
    }

    camera_x = parsed_x;
    camera_y = parsed_y;
    camera_new_frame = 1U;
#if (CAMERA_REPLY_ENABLE != 0U)
    /* 中断只置位标志，回包在主循环完成，避免阻塞接收中断。 */
    camera_reply_pending = 1U;
#endif
    camera_valid_frame_count++;
    return 1U;
}

/*
 * @brief 接收并处理一个摄像头串口字节。
 * @note 只做帧同步与定长解析；禁止在中断中发送 RS485 电机命令。
 */
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
        if (Camera_ParseFrame() == 0U)
        {
            camera_invalid_frame_count++;
        }
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
        camera_invalid_frame_count++;
        Camera_ResetRxState();
    }
}

/*
 * @brief 计算单个轴的离散位置式 PID 输出。
 * @param error 摄像头给出的当前像素偏差，目标值固定为 0。
 * @return 带符号的相对脉冲数，正负号由调用者转换为电机方向。
 */
static int32_t Camera_PIDCalculate(int32_t error, CameraPIDState *state,
                                   float kp, float ki, float kd,
                                   float integral_limit, float output_limit,
                                   int32_t deadband)
{
    float output;
    float derivative;

    if (Camera_Abs32(error) <= deadband)
    {
        state->integral = 0.0f;
        state->last_error = 0.0f;
        return 0;
    }

    /*
     * 步进电机使用相对位移命令时，执行器位置本身已经完成了“积分”。
     * 当某轴Ki设为0时同步清空积分，避免该轴继续保存无意义的历史误差。
     */
    if (ki != 0.0f)
    {
        state->integral += (float)error;
        if (state->integral > integral_limit)
        {
            state->integral = integral_limit;
        }
        else if (state->integral < -integral_limit)
        {
            state->integral = -integral_limit;
        }
    }
    else
    {
        state->integral = 0.0f;
    }

    derivative = (float)error - state->last_error;
    output = kp * (float)error + ki * state->integral + kd * derivative;
    state->last_error = (float)error;

    if (output > output_limit)
    {
        output = output_limit;
    }
    else if (output < -output_limit)
    {
        output = -output_limit;
    }

    return (output >= 0.0f) ? (int32_t)(output + 0.5f) :
                              (int32_t)(output - 0.5f);
}

/*
 * @brief 计算一个轴的带符号 PID 脉冲输出，并过滤死区内的小命令。
 * @note 本函数只计算，不访问 RS485；两个轴计算完成后再统一发送。
 */
static int32_t Camera_RunAxisPID(int32_t error, CameraPIDState *state,
                                float kp, float ki, float kd,
                                float integral_limit, float output_limit,
                                int32_t deadband, uint32_t min_pulse)
{
    int32_t output = Camera_PIDCalculate(error, state, kp, ki, kd,
                                         integral_limit, output_limit, deadband);

    if (output == 0)
    {
        return 0;
    }

    if ((uint32_t)Camera_Abs32(output) < min_pulse)
    {
        return 0;
    }

    return output;
}

/*
 * @brief 累计IMU角度变化对应的摄像头等效误差。
 * @param imu_angle 当前IMU偏航角。
 *
 * 例如车体角度增加90°，等效误差为
 * -90 * CAMERA_PAN_PULSES_PER_DEGREE / CAMERA_X_PID_KP；
 * 程序先与camera_x相加，随后只调用一次水平PID。
 */
static void Camera_AccumulateImuError(float imu_angle)
{
    float imu_delta;

    camera_imu_angle_debug = imu_angle;
    if (s_camera_imu_angle_initialized == 0U)
    {
        s_camera_last_imu_angle = imu_angle;
        s_camera_imu_angle_initialized = 1U;
        camera_imu_delta_debug = 0.0f;
        return;
    }

    imu_delta = Camera_Wrap180(imu_angle - s_camera_last_imu_angle);
    s_camera_last_imu_angle = imu_angle;
    camera_imu_delta_debug = imu_delta;

    s_camera_pending_imu_error += CAMERA_IMU_COMPENSATION_SIGN * imu_delta *
                                  CAMERA_IMU_ERROR_PER_DEGREE;
    if (s_camera_pending_imu_error > CAMERA_IMU_PENDING_ERROR_LIMIT)
    {
        s_camera_pending_imu_error = CAMERA_IMU_PENDING_ERROR_LIMIT;
    }
    else if (s_camera_pending_imu_error < -CAMERA_IMU_PENDING_ERROR_LIMIT)
    {
        s_camera_pending_imu_error = -CAMERA_IMU_PENDING_ERROR_LIMIT;
    }
}

/*
 * @brief 根据当前直线里程触发一次向右转弯前馈。
 * @param distance_cm    当前直线累计里程，单位cm。
 * @param straight_active 1表示正在直线循迹，0表示停车或转弯。
 * @note 前馈只写入待合成误差，不直接控制步进电机；真正输出仍由水平PID完成。
 */
void Camera_Feedforward_Update(float distance_cm, uint8_t straight_active)
{
    float feedforward_error;

    camera_feedforward_distance_debug = distance_cm;
    if ((straight_active == 0U) ||
        (s_camera_feedforward_armed == 0U) ||
        (distance_cm < CAMERA_FEEDFORWARD_DISTANCE_CM))
    {
        return;
    }

    feedforward_error = CAMERA_FEEDFORWARD_SIGN *
                        CAMERA_FEEDFORWARD_TURN_DEG *
                        CAMERA_IMU_ERROR_PER_DEGREE;
    s_camera_pending_feedforward_error += feedforward_error;
    if (s_camera_pending_feedforward_error > CAMERA_IMU_PENDING_ERROR_LIMIT)
    {
        s_camera_pending_feedforward_error = CAMERA_IMU_PENDING_ERROR_LIMIT;
    }
    else if (s_camera_pending_feedforward_error < -CAMERA_IMU_PENDING_ERROR_LIMIT)
    {
        s_camera_pending_feedforward_error = -CAMERA_IMU_PENDING_ERROR_LIMIT;
    }

    /* 立即唤醒下一次主循环的水平PID，不依赖摄像头是否刚好有新帧。 */
    s_camera_feedforward_pending = 1U;
    s_camera_feedforward_armed = 0U;
    camera_feedforward_triggered_debug = 1U;
}

/*
 * @brief 转弯结束后重新允许下一段直线触发距离前馈。
 * @note 同时清除未消费的旧前馈，防止下一段继承上一段命令。
 */
void Camera_Feedforward_Rearm(void)
{
    s_camera_pending_feedforward_error = 0.0f;
    s_camera_feedforward_pending = 0U;
    s_camera_feedforward_armed = 1U;
    camera_feedforward_distance_debug = 0.0f;
    camera_feedforward_error_debug = 0;
    camera_feedforward_triggered_debug = 0U;
}

/*
 * @brief 把两个轴的相对位移先写入对应驱动器，再用广播命令同时触发。
 * @note 这样既避免连续的非同步命令互相干扰，也保证 x/y 同一帧同时起步。
 */
static void Camera_RunDualAxis(int32_t x_output, int32_t y_output)
{
    uint8_t command_queued = 0U;

    if (x_output != 0)
    {
        Stepper_Queue_Relative(STEPPER_ADDR_CHASSIS,
                               (x_output > 0) ? CAMERA_X_POSITIVE_DIRECTION :
                                                CAMERA_X_NEGATIVE_DIRECTION,
                               (uint32_t)Camera_Abs32(x_output));
        command_queued = 1U;
    }

    if (y_output != 0)
    {
        Stepper_Queue_Relative(STEPPER_ADDR_LIFT,
                               (y_output > 0) ? CAMERA_Y_POSITIVE_DIRECTION :
                                                CAMERA_Y_NEGATIVE_DIRECTION,
                               (uint32_t)Camera_Abs32(y_output));
        command_queued = 1U;
    }

    if (command_queued != 0U)
    {
        Stepper_Sync_Move(STEPPER_ADDR_BROADCAST);
    }
}

/*
 * @brief 向 K230 回传一帧有效数据的确认消息。
 * @note 使用 UART3 的阻塞发送，但仅在主循环中调用，不会占用串口接收中断。
 */
static void Camera_SendReply(void)
{
    static const uint8_t reply[] = {'o', 'k', '\r', '\n'};
    uint8_t index;

    for (index = 0U; index < sizeof(reply); index++)
    {
        DL_UART_Main_transmitDataBlocking(UART_CAMERA_INST, reply[index]);
    }
}

/*
 * @brief 初始化摄像头 UART 中断。
 * @note 必须在 main() 中的 SYSCFG_DL_init() 之后调用一次。
 */
void Camera_Init(void)
{
    Camera_ResetRxState();
    Camera_PID_Reset();
    camera_new_frame = 0U;
    camera_reply_pending = 0U;
    camera_x = 0;
    camera_y = 0;

    while (DL_UART_Main_isRXFIFOEmpty(UART_CAMERA_INST) == false)
    {
        (void)DL_UART_Main_receiveData(UART_CAMERA_INST);
    }

    NVIC_ClearPendingIRQ(UART_CAMERA_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_CAMERA_INST_INT_IRQN);
}

/*
 * @brief 摄像头角度环的1ms时基入口。
 * @note 只做饱和计数，应放在现有1ms定时器中断中，不能在这里发送串口命令。
 */
void Camera_ControlTick1ms(void)
{
    if (s_camera_angle_elapsed_ms < CAMERA_ANGLE_LOOP_PERIOD_MS)
    {
        s_camera_angle_elapsed_ms++;
    }
}

/*
 * @brief 清除两个步进电机 PID 的积分和微分历史量。
 * @note 切换视觉模式、停止云台或重新回零后可调用，防止旧积分造成突跳。
 */
void Camera_PID_Reset(void)
{
    s_camera_x_pid.integral = 0.0f;
    s_camera_x_pid.last_error = 0.0f;
    s_camera_y_pid.integral = 0.0f;
    s_camera_y_pid.last_error = 0.0f;
    s_camera_angle_elapsed_ms = 0U;
    s_camera_last_imu_angle = 0.0f;
    s_camera_pending_imu_error = 0.0f;
    s_camera_imu_angle_initialized = 0U;
    s_camera_pending_feedforward_error = 0.0f;
    s_camera_feedforward_pending = 0U;
    s_camera_feedforward_armed = 1U;
    camera_imu_angle_debug = 0.0f;
    camera_imu_delta_debug = 0.0f;
    camera_imu_error_debug = 0;
    camera_combined_x_error_debug = 0;
    camera_angle_output_debug = 0;
    camera_feedforward_distance_debug = 0.0f;
    camera_feedforward_error_debug = 0;
    camera_feedforward_triggered_debug = 0U;
}

/*
 * @brief 将最新摄像头误差、IMU变化和距离前馈合成后执行步进电机PID。
 * @note 水平轴可由三种输入触发；上下轴仍然只由摄像头新帧触发。
 */
void Camera_PID_Run(void)
{
    uint32_t primask;
    int32_t x;
    int32_t y;
    int32_t x_output = 0;
    int32_t y_output = 0;
    int32_t camera_error = 0;
    int32_t imu_error = 0;
    int32_t feedforward_error = 0;
    int32_t combined_x_error = 0;
    float x_output_limit;
    float x_kd;
    uint8_t new_frame;
    uint8_t reply_pending;
    uint8_t angle_loop_due;
    uint8_t feedforward_pending;
    uint8_t horizontal_control_due;

    /* 用短临界区取得同一帧的 x/y，并立刻允许 UART 中断继续接收。 */
    primask = __get_PRIMASK();
    __disable_irq();
    new_frame = camera_new_frame;
    reply_pending = camera_reply_pending;
    feedforward_pending = s_camera_feedforward_pending;
    angle_loop_due = (s_camera_angle_elapsed_ms >=
                      CAMERA_ANGLE_LOOP_PERIOD_MS) ? 1U : 0U;
    if (angle_loop_due != 0U)
    {
        s_camera_angle_elapsed_ms = 0U;
    }
    if ((new_frame == 0U) && (reply_pending == 0U) &&
        (angle_loop_due == 0U) && (feedforward_pending == 0U))
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    x = camera_x;
    y = camera_y;
    camera_new_frame = 0U;
    camera_reply_pending = 0U;
    s_camera_feedforward_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

#if (CAMERA_REPLY_ENABLE != 0U)
    if (reply_pending != 0U)
    {
        Camera_SendReply();
    }
#endif

    if (angle_loop_due != 0U)
    {
        /* 这里只累计IMU等效误差，不单独运行PID，也不直接发送电机命令。 */
        Camera_AccumulateImuError(CAMERA_IMU_ANGLE_SOURCE);
    }

    /*
     * 三种误差都按事件消费一次：摄像头坐标只在新帧到达时使用一次，
     * IMU变化取整后使用一次，距离前馈触发后使用一次。
     */
    camera_error = (new_frame != 0U) ? x : 0;
    imu_error = (s_camera_pending_imu_error >= 0.0f) ?
                (int32_t)(s_camera_pending_imu_error + 0.5f) :
                (int32_t)(s_camera_pending_imu_error - 0.5f);
    feedforward_error = (s_camera_pending_feedforward_error >= 0.0f) ?
                        (int32_t)(s_camera_pending_feedforward_error + 0.5f) :
                        (int32_t)(s_camera_pending_feedforward_error - 0.5f);

    horizontal_control_due = (uint8_t)((new_frame != 0U) ||
                                       (imu_error != 0) ||
                                       (feedforward_pending != 0U));
    if (horizontal_control_due != 0U)
    {
        /*
         * 仅在实际采用某项误差时从待处理量中扣除，保留不足1个误差单位的余量。
         * 这样10ms定时任务不会反复使用上一帧camera_x并重复发送相对位移。
         */
        s_camera_pending_imu_error -= (float)imu_error;
        s_camera_pending_feedforward_error -= (float)feedforward_error;

        /* 摄像头反馈、IMU反馈和距离前馈先合成，随后只执行一次水平PID。 */
        combined_x_error = camera_error + imu_error + feedforward_error;
        camera_imu_error_debug = imu_error;
        camera_combined_x_error_debug = combined_x_error;
        if (feedforward_pending != 0U)
        {
            camera_feedforward_error_debug = feedforward_error;
        }

        /* IMU或前馈参与时放宽上限，确保完整90°命令不被普通视觉限幅截断。 */
        x_output_limit = ((imu_error != 0) || (feedforward_error != 0)) ?
                         CAMERA_X_PID_IMU_OUTPUT_LIMIT :
                         CAMERA_X_PID_OUTPUT_LIMIT;

        /* 一次性前馈不应污染后续视觉反馈的积分/微分历史。 */
        if (feedforward_error != 0)
        {
            s_camera_x_pid.integral = 0.0f;
            s_camera_x_pid.last_error = 0.0f;
            x_kd = 0.0f;
        }
        else
        {
            x_kd = CAMERA_X_PID_KD;
        }
        x_output = Camera_RunAxisPID(combined_x_error, &s_camera_x_pid,
                                     CAMERA_X_PID_KP, CAMERA_X_PID_KI,
                                     x_kd, CAMERA_X_PID_INTEGRAL_LIMIT,
                                     x_output_limit, CAMERA_X_DEADBAND,
                                     CAMERA_X_MIN_PULSE);
        camera_angle_output_debug = x_output;

        if (feedforward_error != 0)
        {
            s_camera_x_pid.integral = 0.0f;
            s_camera_x_pid.last_error = 0.0f;
        }
    }

    if (new_frame != 0U)
    {
        /* 上下轴不受车体水平转角影响，只在摄像头新帧到达时执行。 */
        y_output = Camera_RunAxisPID(y, &s_camera_y_pid,
                                     CAMERA_Y_PID_KP, CAMERA_Y_PID_KI,
                                     CAMERA_Y_PID_KD, CAMERA_Y_PID_INTEGRAL_LIMIT,
                                     CAMERA_Y_PID_OUTPUT_LIMIT, CAMERA_Y_DEADBAND,
                                     CAMERA_Y_MIN_PULSE);
    }

    /* 水平组合误差PID和上下轴PID计算完成后，再统一触发两个步进电机。 */
    Camera_RunDualAxis(x_output, y_output);
}

/*
 * @brief UART3 接收中断：读取 FIFO 并完成摄像头协议帧解析。
 * @note UART3 的 RX 中断与波特率由 SysConfig 的 UART_CAMERA 实例生成。
 */
void UART_CAMERA_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_CAMERA_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (DL_UART_Main_isRXFIFOEmpty(UART_CAMERA_INST) == false)
            {
                Camera_ProcessRxByte(DL_UART_Main_receiveData(UART_CAMERA_INST));
            }
            break;

        case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            DL_UART_Main_clearInterruptStatus(UART_CAMERA_INST,
                                               DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR);
            camera_invalid_frame_count++;
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_BREAK_ERROR:
            DL_UART_Main_clearInterruptStatus(UART_CAMERA_INST,
                                               DL_UART_MAIN_INTERRUPT_BREAK_ERROR);
            camera_invalid_frame_count++;
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_PARITY_ERROR:
            DL_UART_Main_clearInterruptStatus(UART_CAMERA_INST,
                                               DL_UART_MAIN_INTERRUPT_PARITY_ERROR);
            camera_invalid_frame_count++;
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            DL_UART_Main_clearInterruptStatus(UART_CAMERA_INST,
                                               DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);
            camera_invalid_frame_count++;
            Camera_ResetRxState();
            break;

        case DL_UART_MAIN_IIDX_NOISE_ERROR:
            DL_UART_Main_clearInterruptStatus(UART_CAMERA_INST,
                                               DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
            camera_invalid_frame_count++;
            Camera_ResetRxState();
            break;

        default:
            break;
    }
}
