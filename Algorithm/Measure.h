#ifndef __MEASURE_H
#define __MEASURE_H
/* ============================================================
 * Measure.h —— 周期信号测量算法（纯函数实现，不访问硬件与显示）
 * 模块划分：标定 / 单帧统计 / 特征提取(测频、有效值) / 顶层封装(DC、AC、Wave)
 * 约定：buf 为 0~4095 的原始 AD 码值，n 为采样点数，fs 为采样率(Hz)。
 *       本模块不做波形分类，故结果结构体中不含 type 字段。
 * 编译：ARMCC v5（uAC6=0），本工程按 C99 风格书写（循环内声明变量）；
 *       需在 Keil Options -> C/C++ 加入 --c99，或改用 AC6。
 * ============================================================ */
#include <stdint.h>

/* 单次测量的最终结果 */
typedef struct
{
    float   vdc;    /* 直流分量，单位 V（真实输入端，已经标定） */
    float   vpp;    /* 峰峰值，单位 V */
    float   freq;   /* 基波频率，单位 Hz */
    float   rms;    /* 交流有效值，单位 V */
    uint8_t valid;  /* 结果是否有效：校验通过后上层才允许显示 */
} Signal_t;

/* ===== 标定：AD 码值 -> 真实输入电压 ===== */
float Measure_ToVolt(float Vadc);          /* ADC 引脚电压(V) -> 真实输入电压(V)，查标定表所得 */
float Measure_RawToVolt(uint16_t raw);     /* AD 码值 -> 真实输入电压(V) */
float Measure_MeanToVolt(float raw_mean);  /* 平均码值(可含小数) -> 电压(V)，供多帧平均后换算 */

/* ===== 现场重标定叠加（基线表 + Vadc 域锚点修正）=====
   修正量施加在查表之前的 Vadc 域，原理与实现见 Measure.c。 */
void    Measure_ClearCal(void);                       /* 清空锚点，退回基线表 */
int     Measure_AddCal(float vadc_mv, float vin_mv, float grid_mv);  /* 采入锚点：实测 Vadc(mV) 与真实输入(mV)；grid_mv 暂未使用；返回 1=新增或更新，0=被拒绝 */
uint8_t Measure_CalCount(void);                       /* 当前生效的锚点数 */

/* ===== 锚点导出与重建（纯函数，供外部存储层如 Flash 做掉电持久化；单位 mV）=====
   Measure_CalGet    : 读取第 i 个锚点(i 取 0..count-1)的实测 Vadc 与目标 Vin，越界返回 0。
   Measure_CalRestore: 用持久化的 (u, tgt) 数组重建整组锚点：先清空，再逐点经 AddCal 装回
                       (内部用 g0_vadc 反算 v、按 u 升序插入、单调性检查与同点判定)，
                       返回成功装入的锚点数。损坏或倒序的数据会在 AddCal 中被拒绝，故无需额外校验。 */
uint8_t Measure_CalGet(uint8_t i, float *u_mv, float *tgt_mv);
uint8_t Measure_CalRestore(const float *u_mv, const float *tgt_mv, uint8_t n);

/* ===== 单帧统计：一次遍历得到 mean / vmax / vmin ===== */
void  Measure_Stats(const uint16_t *buf, uint16_t n,
                    float *mean, uint16_t *vmax, uint16_t *vmin);

/* ===== 特征提取（输入 buf 与 mean，内部不另开大数组以节省 RAM）===== */
float Measure_Freq(const uint16_t *buf, uint16_t n, float fs,
                   float mean, uint16_t *crossings);
/* 扩展版：额外输出首个上升过零的样本位置 out_first 与每周期样本数 out_P，
   供整数周期裁剪使用；out_first 与 out_P 均可传 NULL。 */
float Measure_FreqInfo(const uint16_t *buf, uint16_t n, float fs, float mean,
                       uint16_t *crossings, float *out_first, float *out_P);
float Measure_RMS(const uint16_t *buf, uint16_t n, float mean); /* 返回 AD 码域中相对均值的有效值 */

/* ===== 顶层封装：汇总为 Signal_t ===== */
Signal_t Measure_DC(const uint16_t *buf, uint16_t n);
Signal_t Measure_AC(const uint16_t *buf, uint16_t n, float fs);
Signal_t Measure_Wave(const uint16_t *buf, uint16_t n, float fs);
void    Measure_Deimpulse(uint16_t *buf, uint16_t n);  /* 就地抑制孤立尖峰：与 5 点中位数之差大于 0.25×稳健跨度的样本
                                                          被中位数替换，保留真实峰值与真实边沿；供采集取帧后的预处理 */
void    Measure_RobustSpan(const uint16_t *buf, uint16_t n, uint16_t *hi, uint16_t *lo);
                                                        /* 输出削去孤立尖峰后的上、下平顶电平(AD 码)。
                                                           用统计量而非原始 max/min：边沿过冲与打轨的
                                                           单点毛刺会把 max/min 差值放大到接近满量程，
                                                           使触发门限落到波形范围之外而无法过零。 */
/* Wave 模式：波形点由应用层(User/main.c)把 buf 直接交给 OLED_DrawWave 绘制，此处仅给出 Vpp/freq 等指标 */

/* 谐波重构去噪：对 n=128 的折叠单周期(AD 码)做 FFT，按噪声底门限滤除宽带噪声后逆变换，
   结果写入 out(128 点码)。保留真实谐波且不削弱真实峰值，避免固定阶数截断带来的 Gibbs 过冲；
   n 不为 128 时原样拷贝。 */
void Measure_ReconHarm(const uint16_t *in, uint16_t n, uint16_t *out);

/* 一维全变差(TV/ROF)去噪(Chambolle 对偶投影迭代)：在压制噪声的同时保留边界(方波台阶、
   三角波尖角不失真)。当前采集流程并未使用它：波形模式的 Vpp 直接取折叠后的真实
   max/min，孤立尖峰由 Measure_Deimpulse 在时域剔除；此函数保留供离线对比实验。
   lam 为去噪强度，量级约为噪声标准差的 2 倍。 */
void Measure_TVDenoise(const uint16_t *in, uint16_t n, float lam, uint16_t *out);

#endif
