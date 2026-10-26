#include "Key.h"
#include "Timer.h"          /* 由 Timer 模块提供毫秒时基与 1ms 回调注册接口 */

/* ===== 单个按键的状态 ===== */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    volatile uint8_t stable;    /* 消抖后的稳定电平：1 表示松开，0 表示按下；中断写入、主循环读取 */
    uint8_t  lastRaw;           /* 上一次读到的原始电平 */
    uint8_t  longFired;         /* 本次按下是否已产生过长按事件，用于避免重复触发 */
    volatile uint8_t clickEvt;  /* 短按事件标志，读取后清零；中断置位、主循环清零 */
    volatile uint8_t longEvt;   /* 长按事件标志，读取后清零；中断置位、主循环清零 */
    uint32_t lastChange;        /* 原始电平最近一次发生变化的时刻（ms） */
    uint32_t pressStart;        /* 确认按下的时刻（ms），长按判定以此为起点 */
} Key_t;

static Key_t keys[2] = {
    {KEY1_GPIO, KEY1_PIN, 1, 1, 0, 0, 0, 0, 0},
    {KEY2_GPIO, KEY2_PIN, 1, 1, 0, 0, 0, 0, 0},
};
#define NKEY  2

static uint8_t RawRead(uint8_t i)
{
    return GPIO_ReadInputDataBit(keys[i].port, keys[i].pin);
}

void Key_Init(void)
{
    GPIO_InitTypeDef g;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* PB3/PB4 默认为 JTAG 引脚，需关闭 JTAG 并保留 SWD 后才能作为普通 GPIO 使用 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    g.GPIO_Mode  = GPIO_Mode_IPU;              /* 内部上拉：松开为高电平，按下接地为低电平 */
    g.GPIO_Speed = GPIO_Speed_50MHz;
    g.GPIO_Pin   = KEY1_PIN | KEY2_PIN;        
    GPIO_Init(KEY1_GPIO, &g);

    /* 将按键扫描注册为 Timer 的 1ms 中断回调，调用前需已完成 Timer_Init() */
    Timer_SetTickCallback(Key_Scan);
}

/* 按键扫描：由 Timer 的 1ms 回调驱动，完成非阻塞消抖与边沿、长按检测。
   时间判定使用毫秒时基的绝对时间戳，不依赖回调被严格等间隔调用。 */
void Key_Scan(void)
{
    uint8_t i;
    uint32_t now = g_msTick;
    for (i = 0; i < NKEY; i++)
    {
        Key_t *k = &keys[i];
        uint8_t raw = RawRead(i);

        if (raw != k->lastRaw)                 /* 电平发生跳变：记录时刻，暂不确认 */
        {
            k->lastRaw = raw;
            k->lastChange = now;
        }
        else if ((now - k->lastChange) >= KEY_DEBOUNCE_MS && raw != k->stable)
        {                                      /* 新电平持续超过消抖时间：确认为稳定状态 */
            k->stable = raw;
            if (raw == 0)                      /* 确认按下：按键接地，读到低电平 */
            {
                k->pressStart = now;
                k->longFired = 0;
            }
            else if (!k->longFired)            /* 确认松开且未达到长按判定时间：记录一次短按事件 */
            {
                k->clickEvt = 1;
            }
        }

        /* 长按判定：持续按下达到 KEY_LONGPRESS_MS 时产生一次事件 */
        if (k->stable == 0 && !k->longFired &&
            (now - k->pressStart) >= KEY_LONGPRESS_MS)
        {
            k->longFired = 1;
            k->longEvt = 1;
        }
    }
}

uint8_t Key_Click(uint8_t id)
{
    uint8_t e;
    if (id < 1 || id > NKEY) return 0;
    __disable_irq();                       /* 临界区：避免读取与清零之间被中断置位而丢失事件 */
    e = keys[id - 1].clickEvt;
    keys[id - 1].clickEvt = 0;
    __enable_irq();
    return e;
}

uint8_t Key_LongPress(uint8_t id)
{
    uint8_t e;
    if (id < 1 || id > NKEY) return 0;
    __disable_irq();
    e = keys[id - 1].longEvt;
    keys[id - 1].longEvt = 0;
    __enable_irq();
    return e;
}

uint8_t Key_AnyPending(void)
{
    /* 仅查询是否存在未读取的按键事件，不取走事件。
       采集循环据此提前结束本次多帧测量，使按键无需等待整轮采集完成。 */
    return (keys[0].clickEvt | keys[0].longEvt | keys[1].clickEvt | keys[1].longEvt) ? 1u : 0u;
}

uint8_t Key_Level(uint8_t id)
{
    if (id < 1 || id > NKEY) return 1;
    return keys[id - 1].stable;
}
