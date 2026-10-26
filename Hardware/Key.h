#ifndef __KEY_H
#define __KEY_H
#include "stm32f10x.h"

/* ===== 引脚配置 =====
   两个板载按键实际接在 PB3/PB4（丝印标注为 PB1/PB2），需关闭 JTAG、保留 SWD 后作为 GPIO 使用；
   按键一端接地，引脚采用内部上拉，按下时读到低电平。 */
#define KEY1_GPIO   GPIOB
#define KEY1_PIN    GPIO_Pin_4     
#define KEY2_GPIO   GPIOB
#define KEY2_PIN    GPIO_Pin_3     

#define KEY_DEBOUNCE_MS   20       /* 消抖时间：跳变电平需连续保持该时长才被确认为新状态 */
#define KEY_LONGPRESS_MS  800      /* 长按判定时间：按下持续超过该时长产生一次长按事件 */

/* 本模块依赖 Timer 提供的 1ms 时基，初始化顺序为先 Timer_Init()、后 Key_Init()。
   毫秒计时与非阻塞延时使用 Timer.h 中的 Millis()。 */
void     Key_Init(void);            /* 配置按键 GPIO，并把 Key_Scan 注册为 Timer 的 1ms 回调 */
void     Key_Scan(void);            /* 按键扫描，由 Timer 每 1ms 调用一次 */
uint8_t  Key_Click(uint8_t id);     /* 读取一次短按事件，id 取 1 或 2，读取后标志自动清零 */
uint8_t  Key_LongPress(uint8_t id); /* 读取一次长按事件，id 取 1 或 2，读取后标志自动清零 */
uint8_t  Key_AnyPending(void);      /* 查询是否存在未读取的按键事件（不取走事件） */
uint8_t  Key_Level(uint8_t id);     /* 读取消抖后的稳定电平：0 表示按下，1 表示松开 */

#endif
