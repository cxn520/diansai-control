#include "main.h"

/*
 * 摄像头数据帧格式：$x-123y-12#。
 * x、y 均是相对于画面中心的有符号偏差；本模块以 0 为两个轴的目标位置。
 */

/* --------------------------- 可调参数：串口协议 --------------------------- */
#define CAMERA_RX_BUFFER_SIZE             (24U)    /* 不含起始符和结束符的最大帧长度 */
#define CAMERA_COORDINATE_LIMIT            (10000)  /* 允许接收的 x/y 最大绝对值 */

/* ----------------------- 可调参数：1 号步进电机（水平） -------------------- */
#define CAMERA_X_PID_KP                    (0.80f)
#define CAMERA_X_PID_KI                    (0.02f)
#define CAMERA_X_PID_KD                    (0.10f)
#define CAMERA_X_PID_INTEGRAL_LIMIT        (1200.0f)
#define CAMERA_X_PID_OUTPUT_LIMIT          (400.0f) /* 单帧最大相对运动脉冲数 */
#define CAMERA_X_DEADBAND                  (4)      /* x 在此范围内不再调整 */
#define CAMERA_X_MIN_PULSE                 (2U)     /* 过小的脉冲命令不发送 */
#define CAMERA_X_POSITIVE_DIRECTION        STEPPER_DIR_CW
#define CAMERA_X_NEGATIVE_DIRECTION        STEPPER_DIR_CCW

/* ----------------------- 可调参数：2 号步进电机（上下） -------------------- */
#define CAMERA_Y_PID_KP                    (0.80f)
#define CAMERA_Y_PID_KI                    (0.02f)
#define CAMERA_Y_PID_KD                    (0.10f)
#define CAMERA_Y_PID_INTEGRAL_LIMIT        (1200.0f)
#define CAMERA_Y_PID_OUTPUT_LIMIT          (400.0f) /* 单帧最大相对运动脉冲数 */
#define CAMERA_Y_DEADBAND                  (4)      /* y 在此范围内不再调整 */
#define CAMERA_Y_MIN_PULSE                 (2U)     /* 过小的脉冲命令不发送 */
#define CAMERA_Y_POSITIVE_DIRECTION        STEPPER_DIR_CW
#define CAMERA_Y_NEGATIVE_DIRECTION        STEPPER_DIR_CCW

/* 摄像头串口中断与主循环之间共享的最新有效坐标。 */
volatile int32_t camera_x = 0;
volatile int32_t camera_y = 0;
volatile uint8_t camera_new_frame = 0U;
volatile uint32_t camera_valid_frame_count = 0U;
volatile uint32_t camera_invalid_frame_count = 0U;

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

    state->integral += (float)error;
    if (state->integral > integral_limit)
    {
        state->integral = integral_limit;
    }
    else if (state->integral < -integral_limit)
    {
        state->integral = -integral_limit;
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
 * @brief 将一个轴的带符号 PID 输出转换为相对脉冲命令。
 * @note 此函数会调用阻塞式 RS485 发送接口，因此只能在主循环中调用。
 */
static void Camera_RunAxisPID(int32_t error, CameraPIDState *state,
                              uint8_t motor_addr, uint8_t positive_direction,
                              uint8_t negative_direction, float kp, float ki,
                              float kd, float integral_limit, float output_limit,
                              int32_t deadband, uint32_t min_pulse)
{
    int32_t output = Camera_PIDCalculate(error, state, kp, ki, kd,
                                         integral_limit, output_limit, deadband);
    uint32_t pulse;

    if (output == 0)
    {
        return;
    }

    pulse = (uint32_t)Camera_Abs32(output);
    if (pulse < min_pulse)
    {
        return;
    }

    Stepper_Move_Relative(motor_addr,
                          (output > 0) ? positive_direction : negative_direction,
                          pulse);
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
 * @brief 清除两个步进电机 PID 的积分和微分历史量。
 * @note 切换视觉模式、停止云台或重新回零后可调用，防止旧积分造成突跳。
 */
void Camera_PID_Reset(void)
{
    s_camera_x_pid.integral = 0.0f;
    s_camera_x_pid.last_error = 0.0f;
    s_camera_y_pid.integral = 0.0f;
    s_camera_y_pid.last_error = 0.0f;
}

/*
 * @brief 对最新的一帧摄像头数据执行两个步进电机 PID。
 * @note 每个有效帧只执行一次；若主循环落后，只使用最新帧以避免云台追踪滞后。
 */
void Camera_PID_Run(void)
{
    uint32_t primask;
    int32_t x;
    int32_t y;

    /* 用短临界区取得同一帧的 x/y，并立刻允许 UART 中断继续接收。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (camera_new_frame == 0U)
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
    if (primask == 0U)
    {
        __enable_irq();
    }

    /* 1 号为左右控制，2 号为上下控制。 */
    Camera_RunAxisPID(x, &s_camera_x_pid, STEPPER_ADDR_CHASSIS,
                      CAMERA_X_POSITIVE_DIRECTION, CAMERA_X_NEGATIVE_DIRECTION,
                      CAMERA_X_PID_KP, CAMERA_X_PID_KI, CAMERA_X_PID_KD,
                      CAMERA_X_PID_INTEGRAL_LIMIT, CAMERA_X_PID_OUTPUT_LIMIT,
                      CAMERA_X_DEADBAND, CAMERA_X_MIN_PULSE);

    Camera_RunAxisPID(y, &s_camera_y_pid, STEPPER_ADDR_LIFT,
                      CAMERA_Y_POSITIVE_DIRECTION, CAMERA_Y_NEGATIVE_DIRECTION,
                      CAMERA_Y_PID_KP, CAMERA_Y_PID_KI, CAMERA_Y_PID_KD,
                      CAMERA_Y_PID_INTEGRAL_LIMIT, CAMERA_Y_PID_OUTPUT_LIMIT,
                      CAMERA_Y_DEADBAND, CAMERA_Y_MIN_PULSE);
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
