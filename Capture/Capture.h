#ifndef __CAPTURE_H
#define __CAPTURE_H
/* ===== Capture.h —— 采集调度层（自动量程 + 首帧有效门控 + 帧管理） =====
 * 定位：全工程中唯一同时依赖 ADC 与 Measure 的层次。
 *   上层（main）只索取结果，不关心采样率档位选择与过渡帧丢弃过程。
 *   依赖方向严格单向：Capture → {ADC, Measure}，ADC 与 Measure 之间互不引用。
 */
#include <stdint.h>
#include "Measure.h"

typedef struct
{
    Signal_t sig;           /* 测量结果 */
    uint32_t fs_used;       /* 本次实际采用的采样率(Hz)，用于核对自动量程的选择结果 */
    uint16_t disp_start;    /* 显示数据起始下标；当前实现恒为 0 */
    uint16_t disp_len;      /* 显示长度(点数)：折叠模式下为折叠周期点数，无有效数据时退回整帧长度 */
    float    raw_mean;      /* DC：本次多帧平均的原始码值；标定采锚与显示使用同一次平均结果 */
} CaptureResult;

void Capture_Init(void);        /* 初始化 ADC，并设置默认量程 */
void Capture_Restart(void);     /* 模式或量程切换后调用：丢弃切换产生的过渡帧 */
void Capture_Unlock(void);      /* 使 AC/Wave 的档位锁定失效：换模式、进出标定(CAL)等信号可能变化时调用，下次重新执行自动量程 */

CaptureResult Capture_DC(void);                 /* 直流：多帧均值换算，目标误差 <10mV */
CaptureResult Capture_AC(uint8_t fast);         /* 交流：Vpp+频率+RMS（内部含自动量程与档位锁定）。fast=1 为信号未稳时的少帧快档，0 为稳态全帧 */
CaptureResult Capture_Wave(uint8_t fast);       /* 波形显示：指标同 AC，波形点见 Capture_WaveBuf。fast 含义同上 */

/* 相干折叠得到的单周期波形（原始码值），由 Capture_Wave 产出，供显示层直接绘制。
   噪声比原始单帧低 √(帧数×每帧周期数)，任意波形的形状完整保留。 */
extern uint16_t Capture_WaveBuf[];   /* 折叠后的单周期缓冲（原始码值） */
extern uint16_t Capture_WaveLen;     /* 有效点数 */
extern uint16_t Capture_WaveLo[];    /* 每列覆盖区间的下包络（原始码值） */
extern uint16_t Capture_WaveHi[];    /* 每列覆盖区间的上包络（原始码值） */
extern uint8_t  Capture_WaveEnv;     /* 1 表示波形过密，显示层应改用 OLED_DrawWaveEnv 绘制竖线包络 */

#endif
