#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f10x.h"

/* ==================== Timer：1ms 系统时基 ====================
   基于 TIM2，与基于 SysTick 轮询的 Delay 模块互不干扰。
   对外提供毫秒计数 g_msTick / Millis()，以及一个每 1ms 执行一次的回调槽，
   按键等模块可把周期性扫描挂在该回调上，无需各自占用定时器。
   ========================================================== */

extern volatile uint32_t g_msTick;         /* 上电以来的毫秒数，在中断中累加 */

void     Timer_Init(void);                 /* 启动 TIM2：每 1ms 中断一次并累加 g_msTick */
uint32_t Millis(void);                     /* 返回 g_msTick，供非阻塞计时使用 */

/* 注册每 1ms 在 TIM2 中断中执行一次的回调（例如 Key_Scan），传 0 取消。
   回调运行在中断上下文，必须短小且不可阻塞（不得使用 Delay_ms 或长循环）。
   当前只提供一个回调槽，多个模块需要 1ms 钩子时应自行在该回调内分发。 */
void     Timer_SetTickCallback(void (*cb)(void));

#endif
