#include "stm32f10x.h"
#include "ADC.h"
#include "Timer.h"

/* ===== 采样链路 =====
   TIM3 溢出事件经 TRGO 触发 ADC1 转换；每次转换结果由 DMA 搬运到定长缓冲区 ADC_Buf；
   缓冲区写满时 DMA 传输完成中断置位 ADC_Ready，主循环按帧取用数据。 */

#define ADC_BUF_LEN  1024
uint16_t ADC_Buf[ADC_BUF_LEN];          /* 采样缓冲区：长度固定，换挡只改变采样率 */
volatile uint8_t ADC_Ready = 0;         /* 一帧采满标志，由 DMA 传输完成中断置位 */

/* 采样率由 TIM3 的溢出周期决定：fs = AD_TIM_CLK / (ARR + 1)。
 * AD_TIM_CLK 取 3.6MHz，对 20k/200k/400k 均可整除，保证整数分频、fs 精确，
 * 因此由 fs 反算的信号频率 f = fs / P 不引入系统误差。 */
#define AD_TIM_CLK     3600000u
#define AD_TIM_PSC     (72000000u / AD_TIM_CLK - 1u)     /* 72MHz 经 20 分频得到 3.6MHz */
#define AD_ARR_OF(fs)  ((uint16_t)(AD_TIM_CLK / (fs) - 1u))
#define AD_DEFAULT_FS  200000u                            /* 上电默认采样率 200kHz */

void AD_Init(void)
{
    /* ===== ADC ===== */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);                   /* ADC 时钟 = 72MHz/6 = 12MHz，低于 14MHz 上限 */

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;              /* PA0 对应 ADC1_IN0，模拟信号输入 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 采样时间取 13.5 个 ADC 时钟周期：相比 1.5 周期有更宽的采样保持窗口，
       信号源阻抗偏高时采样电容也能充分充电，降低建立误差。
       最长转换时间 13.5+12.5=26 个 ADCCLK（约 2.17µs），小于 400kHz 档 2.5µs 的触发周期，
       因此三档采样率下都不会出现转换重叠。 */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_13Cycles5);

    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  /* 每个 TRGO 触发完成一次转换 */
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ExternalTrigConvCmd(ADC1, ENABLE);               /* 允许外部触发，转换节拍完全由 TIM3 决定 */

    /* ===== DMA1_Channel1 ===== */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;   /* 源地址：ADC1 数据寄存器 */
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)ADC_Buf;     /* 目的地址：采样缓冲区首地址 */
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable; /* 外设地址保持不变 */
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;      /* 缓冲区地址递增 */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* 与 ADC 转换结果等宽（16 位） */
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_BufferSize         = ADC_BUF_LEN;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;       /* 装满一帧后停止，由 AD_Start 重装计数并重启 */
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);      /* 使能传输完成（TC）中断，帧满时通知主循环 */
    ADC_DMACmd(ADC1, ENABLE);                            /* 转换结束后由 ADC 发出 DMA 请求，搬运无需 CPU 干预 */

    /* ===== TIM3 TRGO 触发 ===== */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_InitStructure;
    TIM_InitStructure.TIM_Prescaler = AD_TIM_PSC;              /* 72MHz / (19+1) = 3.6MHz 计数器时钟 */
    TIM_InitStructure.TIM_Period    = AD_ARR_OF(AD_DEFAULT_FS); /* ARR 由目标采样率反算：3.6MHz/18 = 200kHz */
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_InitStructure);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);// 溢出事件经 TRGO 输出，作为 ADC 的启动触发
    TIM_Cmd(TIM3, ENABLE);

    /* ===== NVIC：使能 DMA1_Channel1 中断 ===== */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* ===== ADC 自校准 =====
       先复位校准再启动校准；两处轮询在校准完成时退出，耗时微秒级。 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
}

/* 重新启动一帧采集：重装 DMA 传输计数并使能通道，触发源保持为 TIM3 */
void AD_Start(void)
{
    DMA_Cmd(DMA1_Channel1, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel1, ADC_BUF_LEN);
    ADC_Ready = 0;                       /* 先清标志再使能，避免读到上一帧的残留标志 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

/* 运行时切换采样率：仅修改 TIM3 自动重装值，缓冲区长度保持不变 */
void AD_SetRange(const AdRange *r)
{
    uint16_t arr = AD_ARR_OF(r->fs_hz);
    TIM_SetAutoreload(TIM3, arr);
    TIM_SetCounter(TIM3, 0);            /* 计数器清零，避免第一次触发沿用旧周期的残余计数 */
}

/* 等待一帧采集完成：返回 1 表示缓冲区已填满，返回 0 表示超时，避免无限制等待。
   一帧时长 = 缓冲区点数 / 采样率，超时应不小于该时长并留有余量。 */
uint8_t AD_WaitFrame(uint32_t timeout_ms)
{
    uint32_t t0 = Millis();
    while (!ADC_Ready) {
        if (Millis() - t0 > timeout_ms) return 0;
    }
    return 1;
}

/* DMA1_Channel1 传输完成中断：一帧的 ADC_BUF_LEN 个点全部写入缓冲区后进入 */
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1) != RESET) {
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        DMA_Cmd(DMA1_Channel1, DISABLE);   /* 正常模式装满即停，等待 AD_Start 重启 */
        ADC_Ready = 1;                      /* 置位采满标志，通知主循环取用本帧数据 */
    }
}
 

