#include "Timer.h"

/* 毫秒时基计数：由 TIM2 中断每 1ms 累加，其他模块通过 Millis() 读取 */
volatile uint32_t g_msTick = 0;

/* 1ms 周期回调指针（例如 Key_Scan），未注册时为空 */
static void (*s_tick_cb)(void) = 0;

void Timer_SetTickCallback(void (*cb)(void))
{
    s_tick_cb = cb;
}

uint32_t Millis(void)
{
    return g_msTick;
}

/* 启动 TIM2 作为 1ms 系统时基 */
void Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef t;
    NVIC_InitTypeDef n;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    t.TIM_Prescaler     = 7200 - 1;      /* 72MHz / 7200 = 10kHz */
    t.TIM_Period        = 10 - 1;        /* 10kHz / 10 = 1kHz，即每 1ms 溢出一次 */
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &t);

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    n.NVIC_IRQChannel = TIM2_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 3;   
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);

    TIM_Cmd(TIM2, ENABLE);
}

/* ===== TIM2 中断服务：每 1ms 累加时基并调用已注册的回调 ===== */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        g_msTick++;
        if (s_tick_cb) s_tick_cb();
    }
}
