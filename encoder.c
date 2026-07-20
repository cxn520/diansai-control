#include "encoder.h"
volatile int32_t Get_encoder_Left=0;
volatile int32_t Get_encoder_Right=0;

/* 小车累计行走距离，单位：cm。 */
volatile float distance = 0.0f;

void Distance_Update(int32_t left_count, int32_t right_count)
{
    float average_count = ((float)left_count + (float)right_count) * 0.5f;
    distance += average_count * DISTANCE_PER_ENCODER_COUNT_CM;
}

void Distance_Reset(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 与编码器GPIO中断、20ms距离累计保持原子，避免复位后混入转弯残余脉冲。 */
    __disable_irq();
    distance = 0.0f;
    Get_encoder_Left = 0;
    Get_encoder_Right = 0;
    __set_PRIMASK(primask);
}

#define OLED_MENU_TASK_COUNT              4U
#define OLED_KEY_DEBOUNCE_MS              20U

/*
 * TASK 与全局变量 mode 的对应关系只在这里修改。
 * 当前 main.c 中 mode=3 是 TASK2 的第二阶段，因此 TASK3、TASK4
 * 默认从 mode=4、mode=5 开始，避免与 TASK2 的内部状态冲突。
 */
static const int OLED_TASK_MODE_MAP[OLED_MENU_TASK_COUNT] =
{
    1,  /* TASK1 */
    2,  /* TASK2 */
    4,  /* TASK3 */
    8   /* TASK4 */
};

#define OLED_KEY_EVENT_UP                 (1U << 0)
#define OLED_KEY_EVENT_DOWN               (1U << 1)
#define OLED_KEY_EVENT_CONFIRM            (1U << 2)

typedef struct
{
    uint8_t stable_pressed;
    uint8_t debounce_count;
} OledKeyState_t;

static uint8_t oled_task_cursor = 0U;
static uint8_t oled_initialized = 0U;
static uint8_t oled_dirty = 1U;
static uint8_t oled_display_enabled = 1U;

/* 该变量同时被 1 ms 定时器中断和主循环访问。 */
static volatile uint8_t oled_key_events = 0U;

static OledKeyState_t oled_key_up = {0U, 0U};
static OledKeyState_t oled_key_down = {0U, 0U};
static OledKeyState_t oled_key_confirm = {0U, 0U};

/*
 * 对一个低电平有效按键进行 20 ms 消抖。只在稳定按下时产生一次事件，
 * 长按不会连续触发，必须释放后才能产生下一次按下事件。
 */
static void Oled_DebounceKey(OledKeyState_t *key,
                             uint8_t pressed,
                             uint8_t event_mask)
{
    if (pressed == key->stable_pressed)
    {
        key->debounce_count = 0U;
        return;
    }

    if (++key->debounce_count >= OLED_KEY_DEBOUNCE_MS)
    {
        key->stable_pressed = pressed;
        key->debounce_count = 0U;

        if (pressed != 0U)
        {
            oled_key_events |= event_mask;
        }
    }
}

void Oled_KeyScan_1ms(void)
{
    uint8_t up_pressed;
    uint8_t down_pressed;
    uint8_t confirm_pressed;

    up_pressed = (uint8_t)(DL_GPIO_readPins(Key_THREE_PORT,
                                            Key_THREE_PIN) == 0U);
    down_pressed = (uint8_t)(DL_GPIO_readPins(Key_TWO_PORT,
                                              Key_TWO_PIN) == 0U);
    confirm_pressed = (uint8_t)(DL_GPIO_readPins(Key_ONE_PORT,
                                                 Key_ONE_PIN) == 0U);

    Oled_DebounceKey(&oled_key_up, up_pressed, OLED_KEY_EVENT_UP);
    Oled_DebounceKey(&oled_key_down, down_pressed, OLED_KEY_EVENT_DOWN);
    Oled_DebounceKey(&oled_key_confirm, confirm_pressed,
                     OLED_KEY_EVENT_CONFIRM);
}

/* 原子取走中断产生的按键事件。 */
static uint8_t Oled_TakeKeyEvents(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t events;

    __disable_irq();
    events = oled_key_events;
    oled_key_events = 0U;
    __set_PRIMASK(primask);

    return events;
}

static uint8_t Oled_FindTaskCursor(int selected_mode)
{
    uint8_t task_index;

    for (task_index = 0U; task_index < OLED_MENU_TASK_COUNT; task_index++)
    {
        if (OLED_TASK_MODE_MAP[task_index] == selected_mode)
        {
            return task_index;
        }
    }

    return 0U;
}

static void Oled_DrawTaskPage(void)
{
    OLED_ClearBuffer();
    OLED_ShowString(0, 0, "TASK SELECT", 8, 1);

    OLED_ShowString(0, 12,
                    (oled_task_cursor == 0U) ? "> TASK1" : "  TASK1",
                    8, (oled_task_cursor == 0U) ? 0U : 1U);
    OLED_ShowString(0, 23,
                    (oled_task_cursor == 1U) ? "> TASK2" : "  TASK2",
                    8, (oled_task_cursor == 1U) ? 0U : 1U);
    OLED_ShowString(0, 34,
                    (oled_task_cursor == 2U) ? "> TASK3" : "  TASK3",
                    8, (oled_task_cursor == 2U) ? 0U : 1U);
    OLED_ShowString(0, 45,
                    (oled_task_cursor == 3U) ? "> TASK4" : "  TASK4",
                    8, (oled_task_cursor == 3U) ? 0U : 1U);

    OLED_ShowString(0, 56, "UP/DN  OK:START", 8, 1);
    OLED_Refresh();
}

void Oled_Task(void)
{
    uint8_t events = Oled_TakeKeyEvents();

    if (oled_initialized == 0U)
    {
        oled_initialized = 1U;
        oled_task_cursor = Oled_FindTaskCursor(mode);
        oled_dirty = 1U;
    }

    /* 任务运行期间保持 OLED 关闭，并且不进行任何显示刷新。 */
    if (mode != 0)
    {
        if (oled_display_enabled != 0U)
        {
            OLED_DisPlay_Off();
            oled_display_enabled = 0U;
        }
        return;
    }

    /* 任务结束回到 mode=0 时，自动重新点亮并显示选择菜单。 */
    if (oled_display_enabled == 0U)
    {
        OLED_DisPlay_On();
        oled_display_enabled = 1U;
        /* 丢弃任务结束瞬间残留的按键事件，防止菜单被误操作。 */
        events = 0U;
    }

    if ((events & OLED_KEY_EVENT_UP) != 0U)
    {
        oled_task_cursor = (oled_task_cursor == 0U) ?
                           (OLED_MENU_TASK_COUNT - 1U) :
                           (oled_task_cursor - 1U);
        oled_dirty = 1U;
    }
    else if ((events & OLED_KEY_EVENT_DOWN) != 0U)
    {
        oled_task_cursor = (uint8_t)((oled_task_cursor + 1U) %
                                     OLED_MENU_TASK_COUNT);
        oled_dirty = 1U;
    }

    if ((events & OLED_KEY_EVENT_CONFIRM) != 0U)
    {
        int selected_mode = OLED_TASK_MODE_MAP[oled_task_cursor];

        /* 严格按“先关 OLED，再进入任务”的顺序执行。 */
        OLED_DisPlay_Off();
        oled_display_enabled = 0U;
        /* OLED 显存会保留菜单，任务结束重新开屏时无需再次整屏刷新。 */
        oled_dirty = 0U;
        mode = selected_mode;
        return;
    }

    if (oled_dirty != 0U)
    {
        oled_dirty = 0U;
        Oled_DrawTaskPage();
    }
}



/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
void GROUP1_IRQHandler(void)
{
    int32_t gpioB_R=DL_GPIO_getEnabledInterruptStatus(GPIOB,ENCODER_RIGHT_E2A_PIN);
    int32_t gpioB_L=DL_GPIO_getEnabledInterruptStatus(GPIOB,ENCODER_LEFT_E1A_PIN);
    if( gpioB_L & ENCODER_LEFT_E1A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_LEFT_PORT, ENCODER_LEFT_E1B_PIN))
        {
            Get_encoder_Left++;
        }
        else 
        {
            Get_encoder_Left--;
        }
        DL_GPIO_clearInterruptStatus(GPIOB,ENCODER_LEFT_E1A_PIN);
    }
    if(  gpioB_R & ENCODER_RIGHT_E2A_PIN)
    {
        if(DL_GPIO_readPins(ENCODER_RIGHT_PORT, ENCODER_RIGHT_E2B_PIN))
        {
            Get_encoder_Right++;
        }
        else 
        {
            Get_encoder_Right--;
        }
        DL_GPIO_clearInterruptStatus(GPIOB,ENCODER_RIGHT_E2A_PIN);
    }
    

}
