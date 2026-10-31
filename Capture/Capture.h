#ifndef __CAPTURE_H
#define __CAPTURE_H
/*=============================================================
 * Capture.h —— 采集调度层接口（自动量程、首次有效数据门控与帧管理）
 *
 * 定位：本工程中唯一同时依赖 ADC 与 Measure 两个模块的位置。
 *   应用层只向本层请求测量结果，采样方式、采样率档位与过渡帧丢弃均由本层决定。
 *   依赖方向保持单向：Capture → {ADC, Measure}，ADC 与 Measure 之间没有相互引用。
 *============================================================*/
#include <stdint.h>
#include "Measure.h"

typedef struct
{
    Signal_t sig;           /* 测量结果 */
    uint32_t fs_used;       /* 本次实际采用的采样率（Hz），供结果上报与核对使用 */
    uint16_t disp_start;    /* 波形显示裁剪起点（ADC_Buf 下标）；非波形模式下为 0 */
    uint16_t disp_len;      /* 波形显示长度（样本数），为按整数周期裁剪后的结果 */
    float    raw_mean;      /* Capture_DC 多帧平均后的原始码值；标定采锚与显示共用同一估计值 */
} CaptureResult;

void Capture_Init(void);        /* 初始化 ADC 并设置默认量程 */
void Capture_Restart(void);     /* 模式或量程切换后调用，丢弃切换过程中产生的过渡帧 */
void Capture_Unlock(void);      /* 使 AC 与 Wave 的量程锁定失效；在信号可能发生变化的场合（模式切换、进入或退出标定）调用，下次测量重新执行自动量程 */

CaptureResult Capture_DC(void);                 /* 直流测量：由均值换算电压，目标误差小于 10mV */
CaptureResult Capture_AC(uint8_t fast);         /* 交流测量：输出 Vpp、频率与 RMS，内部含自动量程与量程锁定。fast=1 表示信号未稳定时采用较少帧数，fast=0 表示稳态下的全部帧数 */
CaptureResult Capture_Wave(uint8_t fast);       /* 波形显示：指标定义与 Capture_AC 相同，波形数据见 Capture_WaveBuf；fast 的含义同上 */

/* 相干折叠得到的单周期波形（码值），由 Capture_Wave 产出，供应用层直接绘制。
   折叠后的噪声幅度比原始帧低 √(帧数×每帧周期数)，任意波形的形状都完整保留。
   码值为 PA0 处的 ADC 原始码，其中包含直流偏置；绘制按 Vpp 伸缩、按偏置平移的波形
   不需要额外测量：将码值经标定链换算为真实输入电压后，再按格线换算为行号即可。 */
#define WAVE_FOLD_PTS 128              /* 相位折叠桶数，等于显示列数；应用层据此确定缓冲尺寸 */

extern uint16_t Capture_WaveBuf[];   /* 折叠后的单周期缓冲（码值） */
extern uint16_t Capture_WaveLen;     /* 有效点数 */
extern uint16_t Capture_WaveLo[];    /* 每列覆盖区间的下包络（码值） */
extern uint16_t Capture_WaveHi[];    /* 每列覆盖区间的上包络（码值） */
extern uint8_t  Capture_WaveEnv;     /* 1 表示波形过密，应用层应改用包络（竖线）方式绘制 */
extern float    Capture_WaveCyc;     /* 当前屏幕实际显示的周期数，用于计算 t/div */

/* ---- 波形坐标系（纵轴自动分档）：像素几何参数集中定义在下列常量中 ----
   波形页布局：y=0..15 为两行 5×7 字符（数值、V/div 与 t/div 标注），y=16..63 为波形绘制区域。
   垂直方向共 WAVE_DIV_V 格，每格高度为 WAVE_PXDIV 像素；最后一格受屏幕底边限制高度少 1px。 */
#define WAVE_BOX_X    0u
#define WAVE_BOX_W    128u
#define WAVE_BOX_Y    16
#define WAVE_BOX_H    48
#define WAVE_DIV_V    8            /* 垂直格数 */
#define WAVE_DIV_H    8            /* 水平格数（用于 t/div 换算） */
#define WAVE_PXDIV    6            /* 每格像素高度，等于 WAVE_BOX_H/WAVE_DIV_V */

uint16_t Capture_WaveRows(uint8_t *ylo, uint8_t *yhi, uint16_t maxn, uint8_t *out_kdiv, int16_t *out_y0v);
uint16_t Capture_WaveDivMv(uint8_t kdiv);        /* 返回该档 V/div 对应的 mV 数，供验算 */
char    *Capture_WaveDivText(uint8_t kdiv);      /* 返回该档的标注文字，例如 "200mV" */

#endif
