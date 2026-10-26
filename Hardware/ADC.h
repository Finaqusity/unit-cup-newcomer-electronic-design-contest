#ifndef __AD_H
#define __AD_H
#include "stm32f10x.h"

#define ADC_BUF_LEN  1024
extern uint16_t ADC_Buf[ADC_BUF_LEN];     
extern volatile uint8_t ADC_Ready;       

/* 采样率档位：仅给出目标采样率 fs，ARR 由 ADC 模块内部反算，保证整数分频、fs 精确。
   缓冲区长度固定不变，换挡只改变采样率。 */
typedef struct { uint32_t fs_hz; } AdRange;

void    AD_Init(void);
void    AD_Start(void);
void    AD_SetRange(const AdRange *r);      /* 运行中换挡即改变采样率（修改 TIM3 溢出周期） */
uint8_t AD_WaitFrame(uint32_t timeout_ms);  /* 等待一帧采满：返回 1 成功、0 超时 */
#endif
