/* =========================================================
 * Measure.c —— 周期信号测量算法实现（声明见 Measure.h）
 *
 * 设计要点：
 *  1) 纯函数：输入数组、输出标量，不包含任何硬件头文件，
 *     可用合成测试数据在 PC 上离线验证。
 *  2) RAM 占用：STM32F103C8T6 仅有 20KB SRAM。测频与求 RMS 需要
 *     去直流后的序列 x[i]=buf[i]-mean，此处不另开长度为 n 的
 *     int16 数组（2048 点约 4KB），而是在同一循环内即时计算。
 *  3) 非线性标定：Vpp 须由 vmax、vmin 各自换算为电压后再相减，
 *     不可先求 (vmax-vmin) 再乘单一系数。
 *  4) RMS 的平方和使用 uint64_t：4095^2×2048 超出 uint32 范围。
 *
 * 编译：本文件按 C99 风格书写（在 for 语句内声明变量）。需在 Keil
 *   Options -> C/C++ -> Misc Controls 中加入 --c99；ARMCC v5 默认按
 *   C90 编译，会对 for(uint16_t i=...) 报错。
 * ========================================================= */
#include "Measure.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>     /* NULL：Measure_Freq 等函数的可选出参允许传空指针 */

/* ===== 可配置参数 ===== */
#define ADC_VREF        3.3f        /* ADC 基准电压，即 VDDA */
#define ADC_FULLSCALE   4095.0f     /* 12 位 ADC 的满量程码值 */

/* 前端旁路开关：0=查表标定（正常工作）；1=直连，PA0 电压即输入电压（用于接线与标定验证） */
#ifndef FRONT_END_BYPASS
#define FRONT_END_BYPASS   0
#endif

/* ===== 标定：设备实测 Vadc->Vin 的分段线性查表 =====
   数据来源：2026-10-16 设备自测标定。前端旁路（bypass）模式下，输入信号按 0.1V 步进
   从 -6.0V 逐点加到 +5.5V，每点读取设备自身的 ADC 读数（OLED 显示的 Vadc），
   共 116 个折点。
   以设备自身读数建表可同时吸收 ADC 的增益误差、失调误差、积分非线性(INL) 与前端
   电路非线性。采用分段线性而非多项式：折点处误差接近零，段间线性插值在 0.1V 段内的
   曲率残差小于 1mV，整体精度高于单一多项式拟合，且不依赖多项式阶数与系数的选取。
   -3V 与 +3V 附近节点间距不均（-2.9V 处间距 14mV、+3.0V 处间距 39mV）来自器件的
   真实非线性，按实测原值保留。对该区域做平滑会偏离器件实际工作点，并在拐点处引入约
   20mV 的系统性偏差，也不能降低噪声放大，故不采用；该区域的读数抖动由多帧平均与
   抖动（dither）抑制。 */
#define CAL_N  116
static const int16_t CAL_VADC_MV[CAL_N] = {   /* 设备端 Vadc(mV)，作为查表横轴，要求单调递增 */
    125, 152, 179, 205, 231, 258, 284, 311, 337, 364,
    390, 417, 443, 470, 497, 523, 550, 576, 602, 629,
    655, 682, 709, 735, 762, 788, 814, 841, 868, 894,
    921, 935, 961, 988, 1014, 1040, 1067, 1093, 1120, 1146,
    1173, 1199, 1225, 1252, 1279, 1305, 1332, 1358, 1385, 1411,
    1438, 1469, 1495, 1522, 1548, 1575, 1601, 1628, 1653, 1679,
    1707, 1732, 1759, 1786, 1813, 1839, 1866, 1892, 1919, 1945,
    1967, 1994, 2021, 2047, 2073, 2100, 2126, 2153, 2179, 2206,
    2233, 2259, 2286, 2312, 2339, 2365, 2392, 2418, 2445, 2471,
    2510, 2537, 2563, 2590, 2616, 2642, 2669, 2695, 2722, 2748,
    2774, 2801, 2828, 2855, 2881, 2908, 2935, 2961, 2987, 3014,
    3040, 3067, 3093, 3120, 3146, 3172
};
static const int16_t CAL_VIN_MV[CAL_N] = {    /* 与上表逐项对应的真实输入电压(mV) */
    -6000, -5900, -5800, -5700, -5600, -5500, -5400, -5300, -5200, -5100,
    -5000, -4900, -4800, -4700, -4600, -4500, -4400, -4300, -4200, -4100,
    -4000, -3900, -3800, -3700, -3600, -3500, -3400, -3300, -3200, -3100,
    -3000, -2900, -2800, -2700, -2600, -2500, -2400, -2300, -2200, -2100,
    -2000, -1900, -1800, -1700, -1600, -1500, -1400, -1300, -1200, -1100,
    -1000, -900, -800, -700, -600, -500, -400, -300, -200, -100,
    0, 100, 200, 300, 400, 500, 600, 700, 800, 900,
    1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900,
    2000, 2100, 2200, 2300, 2400, 2500, 2600, 2700, 2800, 2900,
    3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800, 3900,
    4000, 4100, 4200, 4300, 4400, 4500, 4600, 4700, 4800, 4900,
    5000, 5100, 5200, 5300, 5400, 5500
};

/* ===== 标定内部工具函数 ===== */
static float rawf_to_vadc(float raw) { return raw * ADC_VREF / ADC_FULLSCALE; }
static float rawf_to_volt(float raw) { return Measure_ToVolt(rawf_to_vadc(raw)); }

/* 基线表：设备 Vadc(V) -> 真实输入 Vin(V)，分段线性；表范围外用首段/末段斜率线性外推 */
static float table_vin(float Vadc)
{
    float x = Vadc * 1000.0f;                     /* 转为 mV，与标定表横轴同单位 */
    if (x <= (float)CAL_VADC_MV[0]) {             /* 低于表首折点：用首段斜率线性外推 */
        float k = (float)(CAL_VIN_MV[1] - CAL_VIN_MV[0]) /
                  (float)(CAL_VADC_MV[1] - CAL_VADC_MV[0]);
        return ((float)CAL_VIN_MV[0] + k * (x - (float)CAL_VADC_MV[0])) / 1000.0f;
    }
    if (x >= (float)CAL_VADC_MV[CAL_N - 1]) {     /* 高于表末折点：用末段斜率线性外推 */
        float k = (float)(CAL_VIN_MV[CAL_N-1] - CAL_VIN_MV[CAL_N-2]) /
                  (float)(CAL_VADC_MV[CAL_N-1] - CAL_VADC_MV[CAL_N-2]);
        return ((float)CAL_VIN_MV[CAL_N-1] + k * (x - (float)CAL_VADC_MV[CAL_N-1])) / 1000.0f;
    }
    for (int i = 0; i < CAL_N - 1; i++) {         /* 落在 [i, i+1) 段内：段间线性插值 */
        if (x < (float)CAL_VADC_MV[i + 1]) {
            float t = (x - (float)CAL_VADC_MV[i]) /
                      (float)(CAL_VADC_MV[i + 1] - CAL_VADC_MV[i]);
            return ((float)CAL_VIN_MV[i] + t * (float)(CAL_VIN_MV[i + 1] - CAL_VIN_MV[i])) / 1000.0f;
        }
    }
    return 0.0f;
}

/* ===== 现场重标定（表前 / Vadc 域 / 分段线性）=====
   目的：补偿"当前信号发生器 + 当前连接线材"相对于建表状态的系统性偏差（偏置与
   分压比增益），测量以发生器示数为准。该偏差物理上产生在 PA0/Vadc 域，经过基线表时
   会被 -3V、+3V 附近等斜率陡峭段按局部斜率不均匀放大，因此修正必须施加在查表之前的
   Vadc 域；若改在 Vin 输出域做仿射修正，则无法抵消这种与局部斜率相关的残差，
   在陡坡段的误差会明显变大。
   每个锚点记录 (u=实测 Vadc, v=基线表对该真实输入本应有的 Vadc，即 g0(target))，
   f: u -> v 为单调分段线性映射，运行时 Vin = table_vin( f(Vadc) )。
   锚点数量少且分布分散（建议取 0V、±3V、±5V），并由 16 帧平均得到，不会引入单点噪声；
   基线表数组保持只读，锚点数为 0 时 f 退化为恒等映射，结果即纯基线表输出。 */
#define CAL_ANCH_MAX 24          /* 锚点数量上限：0.5V 栅格下 -5..+5V 共 21 点，留有余量 */
static float an_u[CAL_ANCH_MAX];      /* 锚点横轴：实测 Vadc(mV)，按 u 升序存放 */
static float an_v[CAL_ANCH_MAX];      /* 锚点纵轴：基线表对应的 Vadc = g0(target)(mV) */
static float an_tgt[CAL_ANCH_MAX];    /* 锚点目标值：真实输入 Vin(mV)，用于同点判定与单调性检查 */
static uint8_t an_n = 0;              /* 当前生效的锚点数 */

/* 正向映射 g0：真实输入 Vin(mV) -> 基线表期望的 Vadc(mV)。按均匀 0.1V 网格线性插值，
   网格范围外用首段/末段斜率线性外推。 */
static float g0_vadc(float vin_mv)
{
    float fi = (vin_mv + 6000.0f) / 100.0f;               /* 浮点栅格下标（-6.0V 对应下标 0） */
    if (fi <= 0.0f) { float k = (float)(CAL_VADC_MV[1]-CAL_VADC_MV[0]); return (float)CAL_VADC_MV[0] + k*fi; }
    if (fi >= CAL_N-1) { float k = (float)(CAL_VADC_MV[CAL_N-1]-CAL_VADC_MV[CAL_N-2]); return (float)CAL_VADC_MV[CAL_N-1] + k*(fi-(CAL_N-1)); }
    int i = (int)fi; if (i > CAL_N-2) i = CAL_N-2;
    float t = fi - (float)i;
    return (float)CAL_VADC_MV[i] + t * (float)(CAL_VADC_MV[i+1]-CAL_VADC_MV[i]);
}

/* 表前修正 f：实测 Vadc(mV) -> 校正后 Vadc(mV)，锚点间分段线性；范围外按首段/末段斜率线性外推，
 *  不作钳位：钳位会将最高锚点以上的输入全部压平为该锚点值，使 5V 被读成 3V，同时令 3V 锚点失效。
 *  无锚点时 f 为恒等映射，单锚点时等价于纯偏置修正。 */
static float cal_f(float vadc_mv)
{
    if (an_n == 0) return vadc_mv;
    if (an_n == 1) return vadc_mv + (an_v[0] - an_u[0]);
    if (vadc_mv <= an_u[0])
        return an_v[0] + (an_v[1]-an_v[0])/(an_u[1]-an_u[0]) * (vadc_mv - an_u[0]);
    if (vadc_mv >= an_u[an_n-1])
        return an_v[an_n-1] + (an_v[an_n-1]-an_v[an_n-2])/(an_u[an_n-1]-an_u[an_n-2]) * (vadc_mv - an_u[an_n-1]);
    for (int i = 0; i < an_n-1; i++)
        if (vadc_mv < an_u[i+1]) {
            float t = (vadc_mv - an_u[i]) / (an_u[i+1] - an_u[i]);
            return an_v[i] + t * (an_v[i+1] - an_v[i]);
        }
    return an_v[an_n-1];
}

void Measure_ClearCal(void) { an_n = 0; }

/* 三组配对数组一起按 u 升序重排：同点重测时 u 会小幅移动，需重新保持有序。 */
static void an_sort(void)
{
    for (int i = 1; i < an_n; i++) {
        float u=an_u[i], v=an_v[i], g=an_tgt[i]; int j=i-1;
        while (j>=0 && an_u[j] > u) { an_u[j+1]=an_u[j]; an_v[j+1]=an_v[j]; an_tgt[j+1]=an_tgt[j]; j--; }
        an_u[j+1]=u; an_v[j+1]=v; an_tgt[j+1]=g;
    }
}

int Measure_AddCal(float vadc_mv, float vin_mv, float grid_mv)
{
    (void)grid_mv;                             /* 同点判定按固定容差进行，栅格步长暂不使用 */
    float v = g0_vadc(vin_mv);                 /* 基线表中该真实输入本应有的 Vadc(mV) */

    /* 同点判定：目标 Vin 与已有锚点是否为同一格点（格点取 100mV 或 500mV 的整数倍，
       不同格点之间至少相差 100mV）。命中则就地更新且不增加锚点数，以支持单点重测。 */
    int mi = -1;
    for (int i = 0; i < an_n; i++)
        if (fabsf(an_tgt[i] - vin_mv) < 50.0f) { mi = i; break; }

    /* 单调性检查：按 Vadc 排序后 target 必须严格递增，否则 f 非单调，反查结果会出现跳变。 */
    {
        float lo = -1e9f, hi = 1e9f;
        for (int i = 0; i < an_n; i++) {
            if (i == mi) continue;
            if (an_u[i] < vadc_mv && an_tgt[i] > lo) lo = an_tgt[i];
            if (an_u[i] > vadc_mv && an_tgt[i] < hi) hi = an_tgt[i];
        }
        if (!(vin_mv > lo && vin_mv < hi)) return 0;   /* 将破坏单调性：拒绝该锚点 */
    }

    if (mi >= 0) { an_u[mi]=vadc_mv; an_v[mi]=v; an_tgt[mi]=vin_mv; an_sort(); return 1; }
    if (an_n >= CAL_ANCH_MAX) return 0;
    int pos = an_n;
    for (int i = 0; i < an_n; i++) if (vadc_mv < an_u[i]) { pos = i; break; }
    for (int i = an_n; i > pos; i--) { an_u[i]=an_u[i-1]; an_v[i]=an_v[i-1]; an_tgt[i]=an_tgt[i-1]; }
    an_u[pos]=vadc_mv; an_v[pos]=v; an_tgt[pos]=vin_mv; an_n++;
    return 1;
}

uint8_t Measure_CalCount(void) { return an_n; }

/* 导出第 i 个锚点的原始数据（实测 Vadc、目标 Vin），供外部存储层（如 Flash）掉电保存。 */
uint8_t Measure_CalGet(uint8_t i, float *u_mv, float *tgt_mv)
{
    if (i >= an_n) return 0;
    if (u_mv)   *u_mv   = an_u[i];
    if (tgt_mv) *tgt_mv = an_tgt[i];
    return 1;
}

/* 由持久化的 (u, tgt) 数组重建锚点组：先清空，再逐点经 Measure_AddCal 装回（内部用 g0_vadc
   反算 v、按 u 升序插入、单调性检查与同点判定），返回成功装入的锚点数。
   损坏或倒序的数据会在 Measure_AddCal 中被拒绝，因此无需额外校验。 */
uint8_t Measure_CalRestore(const float *u_mv, const float *tgt_mv, uint8_t n)
{
    Measure_ClearCal();
    uint8_t ok = 0;
    for (uint8_t i = 0; i < n && an_n < CAL_ANCH_MAX; i++)
        if (Measure_AddCal(u_mv[i], tgt_mv[i], 0.0f)) ok++;
    return ok;
}


float Measure_ToVolt(float Vadc)
{
#if FRONT_END_BYPASS
    return Vadc;                                  /* 直连模式：PA0 电压即视为输入电压 */
#else
    return table_vin( cal_f(Vadc * 1000.0f) / 1000.0f );   /* 先在 Vadc 域做表前修正 f，再查基线表 */
#endif
}

float Measure_RawToVolt(uint16_t raw)
{
    return rawf_to_volt((float)raw);
}

float Measure_MeanToVolt(float raw_mean)
{
    return rawf_to_volt(raw_mean);        /* 平均码值通常带小数，按浮点路径换算 */
}

/* ===== 单帧统计：一次遍历得到 mean / vmax / vmin ===== */
void Measure_Stats(const uint16_t *buf, uint16_t n,
                   float *mean, uint16_t *vmax, uint16_t *vmin)
{
    if (n == 0) { *mean = 0.0f; *vmax = 0; *vmin = 0; return; }

    /* 跳过下标 0：帧切换后首个 DMA 槽可能仍是上一帧的残留值或毛刺，
       以免其污染 max/min，并把误差直接放大到 Vpp。 */
    uint16_t i0 = (n > 1) ? 1u : 0u;
    uint32_t sum = 0;                             /* 4095×1024 约 4.2e6，uint32 足够 */
    uint16_t mx = 0, mn = 0xFFFF;
    for (uint16_t i = i0; i < n; i++) {
        uint16_t v = buf[i];
        sum += v;
        if (v > mx) mx = v;
        if (v < mn) mn = v;
    }
    *mean = (float)sum / (float)(n - i0);
    *vmax = mx;
    *vmin = mn;
}

/* ===== 特征提取：测频（施密特迟滞上升沿 + 线性插值 + 首末平均） ===== */
static void robust_minmax(const uint16_t *buf, uint16_t n, uint16_t *hi, uint16_t *lo);  /* 定义在本文件后方，取稳健跨度 */

float Measure_FreqInfo(const uint16_t *buf, uint16_t n, float fs,
                       float mean, uint16_t *crossings,
                       float *out_first, float *out_P)
{
    /* 原理 f = fs / P，P 为一个周期所占的采样点数，仅在等间隔采样下成立。
       施密特迟滞：信号先降到低门限、再上穿高门限才计一个上升沿，用于抑制小信号时
       噪声在阈值附近的多次虚假过零（否则过零计数偏大、P 偏小、频率读数不稳定）。 */
    float first = 0.0f, last = 0.0f;
    uint16_t cnt = 0;
    if (n < 3) { if (crossings) *crossings = 0; return 0.0f; }

    /* 迟滞电平锚在截峰后的两个平顶 rlo、rhi 上，取跨度的 35% 与 65% 处，分别为预备电平与触发电平。
       不采用 mean±阈值：均值只在信号上下对称时才位于中点，占空比不等于 50% 的脉冲波其均值偏向
       高电平；占空比达 0.9 时 mean+0.3×半幅 已高于波顶，上升沿无法被检出，过零数为 0，
       整帧被判为无信号。占空比为 50% 时两种写法数值相同（mean=(rlo+rhi)/2，
       触发电平 = rlo+0.65×跨度），故正弦与方波的测量结果不变。
       跨度同样须取截峰后的稳健值：原始 max/min 会被方波边沿过冲与触及电源/地轨的毛刺撑大，
       使电平落到真实范围之外。 */
    uint16_t rhi = 0, rlo = 0;
    robust_minmax(buf, n, &rhi, &rlo);
    float lo = (float)rlo, hi = (float)rhi, span = hi - lo;
    float lvl_arm  = lo + span * 0.35f;             /* 必须先降到此处，才允许计入下一个上升沿 */
    float lvl_trig = lo + span * 0.65f;             /* 上穿此处计一个上升沿；各周期同一电平，周期估计更一致 */
    if (lvl_trig - lvl_arm < 4.0f) {                /* 极小信号：回退为围绕中点 ±2 码的最小迟滞带 */
        float mid = 0.5f * (lo + hi);
        lvl_arm = mid - 2.0f; lvl_trig = mid + 2.0f;
    }
    (void)mean;                                     /* 电平已锚定在平顶上，不再依赖均值；形参保留以兼容调用方 */

    int armed = 0;                                  /* 已降到预备电平以下，允许计入下一个上升沿 */
    for (uint16_t i = 1; i + 1 < n; i++) {
        float x0 = (float)buf[i];
        float x1 = (float)buf[i + 1];
        if (x0 < lvl_arm) armed = 1;                /* 已足够低：置预备标志 */
        if (armed && x0 < lvl_trig && x1 >= lvl_trig) {   /* 上穿触发电平 */
            float frac = (lvl_trig - x0) / (x1 - x0);/* 在触发电平处线性插值，定位过零时刻 */
            float t    = (float)i + frac;
            if (cnt == 0) first = t;
            last = t;
            cnt++;
            armed = 0;                              /* 需再次降到预备电平才计下一个 */
        }
    }

    if (crossings) *crossings = cnt;
    if (cnt < 2 || fs <= 0.0f) return 0.0f;       /* 过零次数不足：判为无效（需降低 fs 或加长采样窗） */

    float P = (last - first) / (float)(cnt - 1);  /* 首末平均：把过零时刻的误差分摊到多个周期 */
    if (P <= 0.0f) return 0.0f;

    if (out_first) *out_first = first;            /* 供整数周期裁剪：首个上升过零的样本位置 */
    if (out_P)     *out_P     = P;                /* 每个周期所占的样本数 */
    return fs / P;
}

float Measure_Freq(const uint16_t *buf, uint16_t n, float fs,
                   float mean, uint16_t *crossings)
{
    return Measure_FreqInfo(buf, n, fs, mean, crossings, NULL, NULL);
}

/* ===== 特征提取：有效值（AD 码域，相对均值） ===== */
float Measure_RMS(const uint16_t *buf, uint16_t n, float mean)
{
    /* 返回相对直流分量的交流有效值，单位仍为 AD 码；电压换算由上层完成 */
    if (n == 0) return 0.0f;

    uint64_t acc = 0;                             /* 平方和须用 uint64，否则会溢出 */
    int32_t m = (int32_t)(mean + 0.5f);
    for (uint16_t i = 0; i < n; i++) {
        int32_t d = (int32_t)buf[i] - m;
        acc += (uint64_t)(d * d);
    }
    return sqrtf((float)acc / (float)n);
}

/* ===== 顶层封装：结果汇总为 Signal_t ===== */
Signal_t Measure_DC(const uint16_t *buf, uint16_t n)
{
    Signal_t s = { .vdc = 0, .vpp = 0, .freq = 0, .rms = 0, .valid = 0 };
    if (n == 0) return s;

    float mean; uint16_t vmax, vmin;
    Measure_Stats(buf, n, &mean, &vmax, &vmin);

    s.vdc   = rawf_to_volt(mean);   /* 整帧平均使噪声按 1/sqrt(N) 衰减，配合标定可达 10mV 量级精度 */
    /* 饱和与断线判定使用均值而非绝对 min/max：帧切换后首个 DMA 样本可能为 0 或残值，
       用 vmin 会使整帧被误判。仅当均值接近 0（输入断开）或接近满量程（超出量程）时判为无效。 */
    s.valid = (uint8_t)((mean > 8.0f) && (mean < 4087.0f));
    return s;
}

Signal_t Measure_AC(const uint16_t *buf, uint16_t n, float fs)
{
    /* 交流模式按正弦处理：用整数周期的 RMS 反推幅值，比 max/min 更稳定准确。
       max/min 同时受单点噪声（结果偏大）与非整周期采样漏掉峰值（结果偏小且随相位波动）影响。 */
    Signal_t s = { .vdc = 0, .vpp = 0, .freq = 0, .rms = 0, .valid = 0 };
    if (n == 0) return s;

    float mean; uint16_t vmax, vmin, cnt = 0;
    Measure_Stats(buf, n, &mean, &vmax, &vmin);
    float first = 0.0f, P = 0.0f;
    s.freq = Measure_FreqInfo(buf, n, fs, mean, &cnt, &first, &P);

    /* 取整数个周期窗口以消除频谱泄漏，统计区间为 [ws, ws+wl) */
    uint32_t i0  = (uint32_t)first;
    uint32_t K   = (P > 1.0f && i0 < n) ? (uint32_t)((float)(n - i0) / P) : 0u;
    uint32_t len = (uint32_t)(K * P + 0.5f);
    uint32_t ws, wl;
    if (K >= 2 && len >= 2 && i0 + len <= n) { ws = i0; wl = len; }
    else                                     { ws = 0;  wl = (uint32_t)n; }

    /* 幅值在标定后的 Vin 域统计，并围绕 Vin 的真均值求 RMS（而不是取码均值后反查）：
       码均值反查在 -3V、+3V 等表曲率较大的区段与真均值相差一个 Jensen 项，会抬高交流 RMS，
       使 Vpp 偏大并随直流偏置变化。
       单遍累加 V 与 V^2 求均值和方差（var = E[V^2] - E[V]^2 即交流分量的平方）；正弦 Vpp = 2√2·RMS。 */
    float sv = 0.0f, svv = 0.0f;
    for (uint32_t k = 0; k < wl; k++) {
        float v = rawf_to_volt((float)buf[ws + k]);
        sv  += v;
        svv += v * v;
    }
    float nf = (float)wl;
    float meanv = sv / nf;
    float var = svv / nf - meanv * meanv;
    if (var < 0.0f) var = 0.0f;
    float rmsv = sqrtf(var);
    s.vdc = meanv;                        /* 直流分量：Vin 的真均值 */
    s.rms = rmsv;                         /* 交流有效值(V)：围绕真均值，已去直流 */
    s.vpp = 2.82842712f * rmsv;           /* 正弦 Vpp = 2√2·RMS，即 2 倍峰值 */

    s.valid = (uint8_t)((cnt >= 2) && (s.vpp > 0.0f) && (fs > 0.0f)
                        && (mean > 8.0f) && (mean < 4087.0f));
    return s;
}

/* 分位截峰：两遍直方图取 0.5% 与 99.5% 分位作为谷值与峰值，抑制噪声造成的顶宽失真；
   该处理对任意波形都适用，方波平顶与三角波斜顶仍能取到真实峰谷。 */
#define RB 64
static void robust_minmax(const uint16_t *buf, uint16_t n, uint16_t *hi, uint16_t *lo)
{
    uint16_t mx = 0, mn = 0xFFFF;
    for (uint16_t i = 1; i < n; i++) { uint16_t v = buf[i]; if (v > mx) mx = v; if (v < mn) mn = v; }
    uint32_t span = (uint32_t)mx - mn;
    if (n < 16 || span == 0) { *hi = mx; *lo = mn; return; }   /* 样本过少或无摆幅：回退为绝对 max/min */

    uint16_t hist[RB];
    for (int b = 0; b < RB; b++) hist[b] = 0;
    for (uint16_t i = 1; i < n; i++) {
        uint32_t idx = (uint32_t)(buf[i] - mn) * RB / (span + 1);
        if (idx >= RB) idx = RB - 1;
        hist[idx]++;
    }
    uint32_t total = (uint32_t)n - 1;
    uint32_t trim = total / 200u; if (trim < 1u) trim = 1u;    /* 两端各舍弃 0.5% 的样本 */

    uint32_t acc = 0; int blo = 0, bhi = RB - 1;               /* 累积直方图定出 0.5% 与 99.5% 分位所在柱 */
    for (int b = 0; b < RB; b++) { acc += hist[b]; if (acc > trim) { blo = b; break; } }
    acc = 0;
    for (int b = RB - 1; b >= 0; b--) { acc += hist[b]; if (acc > trim) { bhi = b; break; } }

    *lo = (uint16_t)(mn + (uint32_t)blo * (span + 1) / RB);
    *hi = (uint16_t)(mn + (uint32_t)(bhi + 1) * (span + 1) / RB);
}

/* 孤立尖峰抑制：只移除明显偏离局部趋势的样本，不改变真实峰值与真实边沿。
   判据：样本与 5 点中位数之差大于 T（T = 0.25×稳健跨度，下限 8 码）时，用中位数替换该样本。
   正弦顶部（窗口中位数接近峰值，差值仅来自曲率）与方波斜边（线性段的中位数即中心值）均不触发；
   触及电源/地轨的毛刺与严重边沿过冲会触发。处理就地修改 buf（采集缓冲，下一帧会被覆盖）。 */
void Measure_Deimpulse(uint16_t *buf, uint16_t n)
{
    if (n < 5) return;
    uint16_t hi = 0, lo = 0;
    robust_minmax(buf, n, &hi, &lo);
    float T = (float)(hi - lo) * 0.25f;
    if (T < 8.0f) T = 8.0f;
    for (uint16_t i = 2; i + 2 < n; i++) {
        uint16_t a[5] = { buf[i-2], buf[i-1], buf[i], buf[i+1], buf[i+2] };
        for (int k = 1; k < 5; k++) {                    /* 5 元插入排序，取中位数 a[2] */
            uint16_t x = a[k]; int j = k - 1;
            while (j >= 0 && a[j] > x) { a[j+1] = a[j]; j--; }
            a[j+1] = x;
        }
        uint16_t med = a[2];
        int d = (int)buf[i] - (int)med; if (d < 0) d = -d;
        if ((float)d > T) buf[i] = med;
    }
}

Signal_t Measure_Wave(const uint16_t *buf, uint16_t n, float fs)
{
    /* 波形显示模式处理任意波形：Vpp 取分位截峰后的真实峰峰值，不按正弦用 RMS 反推。
       屏上波形由应用层（User/main.c）直接把 buf 交给 OLED_DrawWave 绘制，此处只给出 Vpp/freq/RMS 指标。 */
    Signal_t s = { .vdc = 0, .vpp = 0, .freq = 0, .rms = 0, .valid = 0 };
    if (n == 0) return s;

    float mean; uint16_t vmax, vmin, cnt = 0;
    Measure_Stats(buf, n, &mean, &vmax, &vmin);
    uint16_t rhi, rlo;
    robust_minmax(buf, n, &rhi, &rlo);                          /* 分位截峰后的谷值与峰值 */
    s.vdc  = rawf_to_volt(mean);
    s.vpp  = Measure_RawToVolt(rhi) - Measure_RawToVolt(rlo);   /* 非线性标定：各自换算为电压后再相减 */
    s.freq = Measure_Freq(buf, n, fs, mean, &cnt);
    float rms_raw = Measure_RMS(buf, n, mean);
    float hi = mean + rms_raw; if (hi > ADC_FULLSCALE) hi = ADC_FULLSCALE;   /* 上限钳到满量程码值 */
    s.rms = fabsf(rawf_to_volt(hi) - s.vdc);                    /* 在 Vin 域取均值到 mean+RMS 的电位差 */

    s.valid = (uint8_t)((cnt >= 2) && (s.vpp > 0.0f) && (fs > 0.0f)
                        && (mean > 8.0f) && (mean < 4087.0f));
    return s;
}

/* ===== 谐波重构去噪（供波形模式 Vpp）=====
   折叠后的单周期是周期信号：能量集中在基波与少数谐波，噪声则展布在宽带。做 128 点 FFT，
   把幅度低于"噪声底（各谐波幅度中位数）×HARM_GATE"的频点连同其共轭镜像一起置零，保留直流与
   真实谐波后再逆变换，因此压噪的同时不削弱真实峰值（固定阶数截断会在方波边沿产生 Gibbs 过冲）。
   正弦只保留基波；方波与三角波保留其衰减到噪声底以内的谐波，平顶与尖角不失真。 */
#define HARM_GATE 3.0f

static void fft_inplace(float *re, float *im, int n, int inv)
{
    for (int i = 1, j = 0; i < n; i++) {                 /* 位反转置换 */
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {             /* 蝶形运算；inv=1 时取共轭旋转做逆变换 */
        float ang = (inv ? 2.0f : -2.0f) * 3.14159265f / (float)len;
        float wr = cosf(ang), wi = sinf(ang), cr = 1.0f, ci = 0.0f;
        for (int i = 0; i < n; i += len) {
            cr = 1.0f; ci = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                float ur = re[i + k], ui = im[i + k];
                float vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci;
                float vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
                re[i + k] = ur + vr; im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
                float ncr = cr * wr - ci * wi; ci = cr * wi + ci * wr; cr = ncr;
            }
        }
    }
    if (inv) { float s = 1.0f / (float)n; for (int i = 0; i < n; i++) { re[i] *= s; im[i] *= s; } }
}

void Measure_ReconHarm(const uint16_t *in, uint16_t n, uint16_t *out)
{
    if (n != 128) { for (int i = 0; i < n; i++) out[i] = in[i]; return; }   /* 仅支持 128 点（2^7），其余长度原样拷贝 */
    static float re[128], im[128];
    for (int i = 0; i < 128; i++) { re[i] = (float)in[i]; im[i] = 0.0f; }
    fft_inplace(re, im, 128, 0);

    float mag[65];
    for (int k = 0; k <= 64; k++) mag[k] = sqrtf(re[k] * re[k] + im[k] * im[k]);   /* 取前半谱幅度 */
    float tmp[64];
    for (int k = 1; k <= 64; k++) tmp[k - 1] = mag[k];        /* 噪声底取各谐波幅度的中位数 */
    for (int i = 1; i < 64; i++) { float x = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > x) { tmp[j + 1] = tmp[j]; j--; } tmp[j + 1] = x; }
    float thr = HARM_GATE * tmp[31];                          /* tmp[31] 即中位数，门限 = HARM_GATE × 噪声底 */

    for (int k = 1; k <= 64; k++) {                           /* 低于门限的频点连同其共轭镜像一起置零 */
        if (mag[k] < thr) {
            re[k] = im[k] = 0.0f;
            re[128 - k] = im[128 - k] = 0.0f;
        }
    }
    fft_inplace(re, im, 128, 1);

    for (int i = 0; i < 128; i++) {
        float v = re[i];
        if (v < 0.0f) v = 0.0f; if (v > 4095.0f) v = 4095.0f;
        out[i] = (uint16_t)(v + 0.5f);
    }
}

/* ===== 全变差(TV/ROF)去噪（波形模式 Vpp 默认方法）=====
   求解 min_x 0.5*||x-y||^2 + lambda*||Dx||_1，用 Chambolle 对偶投影迭代，lambda 即形参 lam。
   属保边去噪：压掉宽带噪声的同时保留方波的台阶与三角波的尖角（Fourier 截断会带来 Gibbs
   过冲并圆化边缘），因此对任意波形的峰谷估计都稳定。 */
void Measure_TVDenoise(const uint16_t *in, uint16_t n, float lam, uint16_t *out)
{
    if (n < 3 || n > 256) { for (int i = 0; i < n; i++) out[i] = in[i]; return; }   /* 超出支持长度则原样拷贝 */
    static float x[256], p[256];
    float tau = 0.24f;                              /* 迭代步长，需小于 0.5 以保证收敛 */
    for (int i = 0; i < n - 1; i++) p[i] = 0.0f;
    for (int it = 0; it < 200; it++) {              /* 固定迭代次数，兼顾收敛速度与耗时 */
        for (int i = 0; i < n; i++) {               /* x = y - D^T p，其中 (D^T p)[i] = p[i-1] - p[i] */
            float dtp; if (i == 0) dtp = -p[0]; else if (i == n - 1) dtp = p[n - 2]; else dtp = p[i - 1] - p[i];
            x[i] = (float)in[i] - dtp;
        }
        for (int i = 0; i < n - 1; i++) {           /* p 投影到 |p| <= lam：p = Proj(p + tau*Dx) */
            float v = p[i] + tau * (x[i + 1] - x[i]);
            if (v > lam) v = lam; if (v < -lam) v = -lam;
            p[i] = v;
        }
    }
    for (int i = 0; i < n; i++) {                   /* 结果钳回 0~4095 码值范围后取整输出 */
        float v = x[i]; if (v < 0.0f) v = 0.0f; if (v > 4095.0f) v = 4095.0f;
        out[i] = (uint16_t)(v + 0.5f);
    }
}
