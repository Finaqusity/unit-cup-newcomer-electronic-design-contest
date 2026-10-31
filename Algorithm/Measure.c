/*=============================================================
 * Measure.c —— 周期信号测量算法实现（对应 Measure.h）
 *
 * 设计要点：
 *  1) 纯函数接口：只接受数组输入并返回标量结果，不包含任何硬件头文件，因此可脱离目标板，
 *     用合成波形数据在 PC 端单元测试中先行验证。
 *  2) 节省 RAM：C8T6 只有 20KB。测频与求 RMS 需要去直流 x[i]=buf[i]-mean，
 *     但不额外分配长度为 n 的 int16 数组（2048 点约 4KB，开销过大），
 *     而是在循环内即时计算并直接使用。
 *  3) 非线性标定：Vpp 必须由 vmax 与 vmin 各自换算成电压后再相减，
 *     不能先求 (vmax-vmin) 再乘系数。
 *  4) RMS 的平方和使用 uint64_t：4095^2×2048 会溢出 uint32。
 *
 * 本文件按 C99 风格编写（在 for 语句中声明变量）。需在 Keil 的
 *   Options → C/C++ → Misc Controls 中加入 "--c99"，否则 ARMCC v5
 *   默认按 C90 编译，会对 for(uint16_t i=...) 报错。
 *============================================================*/
#include "Measure.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>     /* NULL：Measure_Freq 向可选输出参数传递空指针时需要 */

/* ---------- 可调参数 ---------- */
#define ADC_VREF        3.3f        /* ADC 基准电压 VDDA */
#define ADC_FULLSCALE   4095.0f     /* 12 位满码值 */

/* 标定模式：0 表示查表标定（正常工作方式）；1 表示直连（把 PA0 电压当作输入电压，仅用于调试） */
#ifndef FRONT_END_BYPASS
#define FRONT_END_BYPASS   0
#endif

/* ---------- ⓪ 标定：设备实测 Vadc→Vin 的分段线性查表 ----------
   在 FRONT_END_BYPASS=1 条件下逐点读取设备自测的 Vadc，得到 116 个折点（输入按 0.1V 步进，范围 -6..+5.5V）。
   直接以设备自身的 ADC 读数建立映射，可自动吸收 ADC 的增益误差、失调误差、INL 以及前端电路的非线性；
   相比单一多项式拟合更准确：折点处误差接近 0，段间线性插值在 0.1V 段内的曲率残差小于 1mV。
   ±3V 附近节点间距不均（-2.9V 处间距 14、+3.0V 处间距 39）来自器件的真实非线性，保留实测原值。
   对该区域做局部平滑会偏离器件的实际工作点，在拐点处引入约 20mV 的系统误差，也不能降低噪声放大，
   因此不予平滑；该处的读数抖动只能依靠多帧平均或 dither 抑制。
   表值必须在输入电压与设备读数都稳定时采集：若读数不稳定（例如 5V 时出现 4773 这类异常码值），说明
   前端存在间歇性接触不良，与 MCU 无关；这种状态下整表重扫得到的数值（例如 0V→1702、5V→2979）不可用，
   硬件状态恢复正常后应重新扫描整表；当前采用的端点折点为 5V→3040。 */
#define CAL_N  116
static const int16_t CAL_VADC_MV[CAL_N] = {   /* 设备 Vadc（mV），必须单调递增 */
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
static const int16_t CAL_VIN_MV[CAL_N] = {    /* 对应的真实输入电压（mV） */
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

/* ---------- ⓪ 标定的内部辅助工具 ---------- */
static float rawf_to_vadc(float raw) { return raw * ADC_VREF / ADC_FULLSCALE; }
static float rawf_to_volt(float raw) { return Measure_ToVolt(rawf_to_vadc(raw)); }

/* 基线标定表：设备 Vadc(V) → Vin(V)，分段线性；超出表范围时用首段或末段的斜率外推 */
static float table_vin(float Vadc)
{
    float x = Vadc * 1000.0f;                     /* 换算为设备 Vadc（mV） */
    if (x <= (float)CAL_VADC_MV[0]) {
        float k = (float)(CAL_VIN_MV[1] - CAL_VIN_MV[0]) /
                  (float)(CAL_VADC_MV[1] - CAL_VADC_MV[0]);
        return ((float)CAL_VIN_MV[0] + k * (x - (float)CAL_VADC_MV[0])) / 1000.0f;
    }
    if (x >= (float)CAL_VADC_MV[CAL_N - 1]) {
        float k = (float)(CAL_VIN_MV[CAL_N-1] - CAL_VIN_MV[CAL_N-2]) /
                  (float)(CAL_VADC_MV[CAL_N-1] - CAL_VADC_MV[CAL_N-2]);
        return ((float)CAL_VIN_MV[CAL_N-1] + k * (x - (float)CAL_VADC_MV[CAL_N-1])) / 1000.0f;
    }
    for (int i = 0; i < CAL_N - 1; i++) {         /* 落在 [i, i+1) 区间内，做线性插值 */
        if (x < (float)CAL_VADC_MV[i + 1]) {
            float t = (x - (float)CAL_VADC_MV[i]) /
                      (float)(CAL_VADC_MV[i + 1] - CAL_VADC_MV[i]);
            return ((float)CAL_VIN_MV[i] + t * (float)(CAL_VIN_MV[i + 1] - CAL_VIN_MV[i])) / 1000.0f;
        }
    }
    return 0.0f;
}

/* --- 现场重标定（表前、Vadc 域、分段线性）：用于补偿不同信号发生器与不同线材带来的系统差。
   测量以发生器的示数为基准，而当前发生器与线材相对建表时的状态会有偏置与增益（分压比）差异；
   这种差异物理上发生在 PA0/Vadc 域，并按 ±3V 附近大斜率区段的局部斜率被不均匀放大，因此必须在查表之前的
   Vadc 域修正，而不是在 Vin 输出域做仿射变换——后者在 ±3V 处的实测残差为 9.5，前者为 1.3。
   每个锚点保存 (u=实测 Vadc, v=基表中该真实输入对应的 Vadc=g0(target))；f: u→v 为单调分段线性映射，
   运行时 Vin = table_vin( f(Vadc) )。锚点数量少且分布分散（推荐 0、±3、±5），并取自 16 帧平均，
   因此不会把单点噪声带入标定；基表数组只读，f 为恒等映射（0 个锚点）时即等同于纯基表。
   锚点式修正只调整映射的整体形状，不会用逐点残差覆盖基表已有的非线性。 --- */
#define CAL_ANCH_MAX 48          /* 锚点数量上限：0.5V 栅格 −5..+5（21 点）加上被测段 0.1V 加密（21 点）共 37 点，另留余量。
                                    锚点数据由存储层分两块写入 Flash。 */
static float an_u[CAL_ANCH_MAX];      /* 锚点：实测 Vadc（mV），按 u 升序存放 */
static float an_v[CAL_ANCH_MAX];      /* 锚点：基表中对应的 Vadc = g0(target)（mV） */
static float an_tgt[CAL_ANCH_MAX];    /* 锚点：真实输入 target（mV），用于去重与单调性判定 */
static uint8_t an_n = 0;

/* 正向映射 g0：真实 Vin（mV）→ 基表期望的 Vadc（mV）；按均匀 0.1V 网格线性插值，超出范围时用首末段斜率外推 */
static float g0_vadc(float vin_mv)
{
    float fi = (vin_mv + 6000.0f) / 100.0f;               /* 对应的浮点下标 */
    if (fi <= 0.0f) { float k = (float)(CAL_VADC_MV[1]-CAL_VADC_MV[0]); return (float)CAL_VADC_MV[0] + k*fi; }
    if (fi >= CAL_N-1) { float k = (float)(CAL_VADC_MV[CAL_N-1]-CAL_VADC_MV[CAL_N-2]); return (float)CAL_VADC_MV[CAL_N-1] + k*(fi-(CAL_N-1)); }
    int i = (int)fi; if (i > CAL_N-2) i = CAL_N-2;
    float t = fi - (float)i;
    return (float)CAL_VADC_MV[i] + t * (float)(CAL_VADC_MV[i+1]-CAL_VADC_MV[i]);
}

/* 表前修正 f：实测 Vadc（mV）→ 校正后的 Vadc（mV），由锚点分段线性确定；超出锚点范围时按首段或末段
 *   斜率线性外推，不做钳位——钳位会把最高锚点以上的输入全部压平到该锚点值，导致 5V 被读成 3V 并覆盖 3V 锚点。
 *   没有锚点时 f 为恒等映射，只有一个锚点时 f 为纯偏置修正。 */
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

/* 将锚点按 u 升序重排（三个数组配对移动）。同一格点重测时 u 会发生小幅移动，因此需要恢复顺序。 */
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
    (void)grid_mv;
    float v = g0_vadc(vin_mv);                 /* 基表中该真实输入对应的 Vadc（mV） */

    /* 同一格点判定：目标 Vin 与已有锚点是否为同一格点（target 均为 100mV 或 500mV 的整数倍，不同格点至少相差 100mV）。
       命中时就地更新该锚点且数量不增加，从而支持对同一测量点重复标定。 */
    int mi = -1;
    for (int i = 0; i < an_n; i++)
        if (fabsf(an_tgt[i] - vin_mv) < 50.0f) { mi = i; break; }

    /* 单调性检查：按 Vadc 排序后 target 必须严格递增，否则 f 不再单调，反查时会出现倒序跳变。 */
    {
        float lo = -1e9f, hi = 1e9f;
        for (int i = 0; i < an_n; i++) {
            if (i == mi) continue;
            if (an_u[i] < vadc_mv && an_tgt[i] > lo) lo = an_tgt[i];
            if (an_u[i] > vadc_mv && an_tgt[i] < hi) hi = an_tgt[i];
        }
        if (!(vin_mv > lo && vin_mv < hi)) return 0;   /* 会破坏单调性的锚点不予接受 */
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

/* 导出第 i 个锚点的原始数据（实测 Vadc 与目标 Vin），供外部存储层写入 Flash 持久化。 */
uint8_t Measure_CalGet(uint8_t i, float *u_mv, float *tgt_mv)
{
    if (i >= an_n) return 0;
    if (u_mv)   *u_mv   = an_u[i];
    if (tgt_mv) *tgt_mv = an_tgt[i];
    return 1;
}

/* 从持久化的 (u,tgt) 重建锚点组：先清空，再逐点通过 Measure_AddCal 装回（内部用 g0_vadc 反算 v、
   按 u 升序插入、并做单调性检查与同点去重），返回成功装入的点数。损坏或顺序异常的点会被自动拒绝。 */
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
    return Vadc;                                  /* 直连模式：PA0 电压即为信号电压 */
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
    return rawf_to_volt(raw_mean);        /* 平均码值通常带小数，直接用浮点换算 */
}

/* ---------- ① 单帧统计：一次遍历得到 mean/vmax/vmin ---------- */
void Measure_Stats(const uint16_t *buf, uint16_t n,
                   float *mean, uint16_t *vmax, uint16_t *vmin)
{
    if (n == 0) { *mean = 0.0f; *vmax = 0; *vmin = 0; return; }

    /* 跳过下标 0：刚重启帧的首个 DMA 槽可能是上一帧的残留或毛刺，避免它影响 max/min（进而影响 Vpp） */
    uint16_t i0 = (n > 1) ? 1u : 0u;
    uint32_t sum = 0;                             /* 4095×1024≈4.2e6，uint32 的范围足够 */
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

/* ---------- ② 特征：测频（施密特迟滞上升沿 + 线性插值 + 首末平均） ---------- */
static void robust_minmax(const uint16_t *buf, uint16_t n, uint16_t *hi, uint16_t *lo);  /* 前向声明：定义在后方，此处用于取稳健跨度 */

float Measure_FreqInfo(const uint16_t *buf, uint16_t n, float fs,
                       float mean, uint16_t *crossings,
                       float *out_first, float *out_P)
{
    /* 原理为 f = fs / P，其中 P 是一个周期占用的采样点数（只在等间隔采样时成立）。
       施密特迟滞：信号先下到低电平、再上穿高电平才计为一个上升沿，用于抑制小信号下噪声在阈值附近的
       多次虚假过零；否则 cnt 偏大、P 偏小，频率读数不稳定。 */
    float first = 0.0f, last = 0.0f;
    uint16_t cnt = 0;
    if (n < 3) { if (crossings) *crossings = 0; return 0.0f; }

    /* 迟滞电平锚在两条平顶（截峰后的 rlo 与 rhi）上，取跨度的 35% 与 65% 两处，分别为预备电平与触发电平。
       不采用 mean±阈值 的原因：mean 只有在信号上下对称时才位于正中；占空比不等于 50% 的脉冲波，其均值
       偏向高电平，duty=0.9 时 mean+0.3·半幅 = 1.1A 已高于波顶 A，上穿不会发生，cnt=0 使整帧被丢弃并
       显示"无信号"（PC 端单元测试结果：duty≥0.90 时 1kHz 与 20kHz 全部 valid=0，duty=0.8 时正常）。
       占空比为 50% 时两种写法的数值完全相同（mean=(rlo+rhi)/2，触发电平 = rlo+0.65·span），正弦波与方波的行为不变。
       跨度同样必须使用截峰后的稳健值：使用原始 max/min 时，方波边沿过冲与触顶毛刺会占满整个跨度，使电平超出真实范围。 */
    uint16_t rhi = 0, rlo = 0;
    robust_minmax(buf, n, &rhi, &rlo);
    float lo = (float)rlo, hi = (float)rhi, span = hi - lo;
    float lvl_arm  = lo + span * 0.35f;             /* 信号必须先下到此电平，才允许计入下一个上升沿 */
    float lvl_trig = lo + span * 0.65f;             /* 上穿此电平计为一个上升沿；各周期用同一电平，周期长度才一致 */
    if (lvl_trig - lvl_arm < 4.0f) {                /* 极小信号：退回到围绕中点 ±2 码的最小迟滞带 */
        float mid = 0.5f * (lo + hi);
        lvl_arm = mid - 2.0f; lvl_trig = mid + 2.0f;
    }
    (void)mean;                                     /* 电平已锚在平顶上，不再依赖均值（保留形参以兼容调用方） */

    int armed = 0;                                  /* 已下到预备电平以下，准备计入一个上升沿 */
    for (uint16_t i = 1; i + 1 < n; i++) {
        float x0 = (float)buf[i];
        float x1 = (float)buf[i + 1];
        if (x0 < lvl_arm) armed = 1;                /* 低于预备电平，进入预备状态 */
        if (armed && x0 < lvl_trig && x1 >= lvl_trig) {   /* 上穿触发电平 */
            float frac = (lvl_trig - x0) / (x1 - x0);/* 在触发电平处插值，各周期同一电平保证周期一致 */
            float t    = (float)i + frac;
            if (cnt == 0) first = t;
            last = t;
            cnt++;
            armed = 0;                              /* 需要再次下到预备电平才计入下一个上升沿 */
        }
    }

    if (crossings) *crossings = cnt;
    if (cnt < 2 || fs <= 0.0f) return 0.0f;       /* 上升沿过少时结果无效（需要降低采样率或加长采集窗口） */

    float P = (last - first) / (float)(cnt - 1);  /* 用首末两个过零点求平均，把误差分摊到多个周期 */
    if (P <= 0.0f) return 0.0f;

    if (out_first) *out_first = first;            /* 供显示裁剪使用：首个上升过零点的样本位置 */
    if (out_P)     *out_P     = P;                /* 每个周期占用的采样点数 */
    return fs / P;
}

float Measure_Freq(const uint16_t *buf, uint16_t n, float fs,
                   float mean, uint16_t *crossings)
{
    return Measure_FreqInfo(buf, n, fs, mean, crossings, NULL, NULL);
}

/* ---------- ② 特征：有效值（AD 码域，偏离均值） ---------- */
float Measure_RMS(const uint16_t *buf, uint16_t n, float mean)
{
    /* 返回相对直流分量的交流有效值，单位仍为 AD 码；电压换算由上层完成 */
    if (n == 0) return 0.0f;

    uint64_t acc = 0;                             /* 平方和必须用 uint64 累加 */
    int32_t m = (int32_t)(mean + 0.5f);
    for (uint16_t i = 0; i < n; i++) {
        int32_t d = (int32_t)buf[i] - m;
        acc += (uint64_t)(d * d);
    }
    return sqrtf((float)acc / (float)n);
}

/* ---------- ③ 顶层封装 → Signal_t ---------- */
Signal_t Measure_DC(const uint16_t *buf, uint16_t n)
{
    Signal_t s = { .vdc = 0, .vpp = 0, .freq = 0, .rms = 0, .valid = 0 };
    if (n == 0) return s;

    float mean; uint16_t vmax, vmin;
    Measure_Stats(buf, n, &mean, &vmax, &vmin);

    s.vdc   = rawf_to_volt(mean);   /* 多点平均使噪声按 √N 下降，配合标定满足误差小于 10mV 的要求 */
    /* 饱和与断线判定使用均值而不是绝对 min/max：刚重启帧的首个 DMA 样本可能是 0 或残留值，
       用 vmin 判定会使整帧被误判为无效。只有均值接近下限（输入断线）或接近上限（超出量程）时才判为无效。 */
    s.valid = (uint8_t)((mean > 8.0f) && (mean < 4087.0f));
    return s;
}

Signal_t Measure_AC(const uint16_t *buf, uint16_t n, float fs)
{
    /* 交流模式按正弦波处理：用整数周期 RMS 反推幅值，比 max/min 更稳定准确。
       max/min 同时受单点噪声（结果偏大）与非整周期造成的峰值漏采（结果偏小且随相位抖动）影响。 */
    Signal_t s = { .vdc = 0, .vpp = 0, .freq = 0, .rms = 0, .valid = 0 };
    if (n == 0) return s;

    float mean; uint16_t vmax, vmin, cnt = 0;
    Measure_Stats(buf, n, &mean, &vmax, &vmin);
    float first = 0.0f, P = 0.0f;
    s.freq = Measure_FreqInfo(buf, n, fs, mean, &cnt, &first, &P);

    /* 取整数个周期的窗口以避免频谱泄漏，统计区间为 [ws, ws+wl) */
    uint32_t i0  = (uint32_t)first;
    uint32_t K   = (P > 1.0f && i0 < n) ? (uint32_t)((float)(n - i0) / P) : 0u;
    uint32_t len = (uint32_t)(K * P + 0.5f);
    uint32_t ws, wl;
    if (K >= 2 && len >= 2 && i0 + len <= n) { ws = i0; wl = len; }
    else                                     { ws = 0;  wl = (uint32_t)n; }

    /* 幅值在标定后的 Vin 域统计，并绕 Vin 的真实均值求 RMS（而不是对码均值做反查）：
       码均值反查在 ±3V 等曲率较大处与真实均值相差一个 Jensen 量，会抬高交流 RMS，使 Vpp 偏大并随偏置变化。
       单遍累加 V 与 V² 以求均值和方差（var=E[V²]-E[V]² 即交流分量的平方）；正弦波 Vpp=2√2·RMS。 */
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
    s.vdc = meanv;                        /* 直流分量即 Vin 的真实均值 */
    s.rms = rmsv;                         /* 交流有效值（V），绕真实均值计算，已去除直流分量 */
    s.vpp = 2.82842712f * rmsv;           /* 正弦波 Vpp = 2√2·RMS = 2 倍峰值 */

    s.valid = (uint8_t)((cnt >= 2) && (s.vpp > 0.0f) && (fs > 0.0f)
                        && (mean > 8.0f) && (mean < 4087.0f));
    return s;
}

/* 分位截峰：用两遍直方图取 0.5% 与 99.5% 分位作为谷与峰，抑制噪声造成的顶宽展宽；
   对任意波形都成立，方波的平顶与三角波的斜顶仍能取到真实峰值。 */
#define RB 64
static void robust_minmax(const uint16_t *buf, uint16_t n, uint16_t *hi, uint16_t *lo)
{
    uint16_t mx = 0, mn = 0xFFFF;
    for (uint16_t i = 1; i < n; i++) { uint16_t v = buf[i]; if (v > mx) mx = v; if (v < mn) mn = v; }
    uint32_t span = (uint32_t)mx - mn;
    if (n < 16 || span == 0) { *hi = mx; *lo = mn; return; }   /* 样本过少或没有摆幅时退回到绝对 max/min */

    uint16_t hist[RB];
    for (int b = 0; b < RB; b++) hist[b] = 0;
    for (uint16_t i = 1; i < n; i++) {
        uint32_t idx = (uint32_t)(buf[i] - mn) * RB / (span + 1);
        if (idx >= RB) idx = RB - 1;
        hist[idx]++;
    }
    uint32_t total = (uint32_t)n - 1;
    uint32_t trim = total / 200u; if (trim < 1u) trim = 1u;    /* 两端各截去 0.5% */

    uint32_t acc = 0; int blo = 0, bhi = RB - 1;
    for (int b = 0; b < RB; b++) { acc += hist[b]; if (acc > trim) { blo = b; break; } }
    acc = 0;
    for (int b = RB - 1; b >= 0; b--) { acc += hist[b]; if (acc > trim) { bhi = b; break; } }

    *lo = (uint16_t)(mn + (uint32_t)blo * (span + 1) / RB);
    *hi = (uint16_t)(mn + (uint32_t)(bhi + 1) * (span + 1) / RB);
}

/* 剔除孤立尖峰：只修正相对局部趋势突出的样本，不改变真实峰值与真实边沿。
   判据：样本与 5 点中位数之差大于 T（T = 0.25×稳健跨度，下限 8 码）时，用中位数替换该样本。
   正弦波顶部（窗口中位数接近峰值，差值只含曲率项）与方波斜边（线性段的中位数等于中心值）都不会触发；
   触顶或触地的毛刺以及边沿大幅过冲会触发。处理就地修改 buf（采集缓冲在下一帧即被覆盖）。 */
void Measure_Deimpulse(uint16_t *buf, uint16_t n)
{
    if (n < 5) return;
    uint16_t hi = 0, lo = 0;
    robust_minmax(buf, n, &hi, &lo);
    float T = (float)(hi - lo) * 0.25f;
    if (T < 8.0f) T = 8.0f;
    for (uint16_t i = 2; i + 2 < n; i++) {
        uint16_t a[5] = { buf[i-2], buf[i-1], buf[i], buf[i+1], buf[i+2] };
        for (int k = 1; k < 5; k++) {                    /* 5 个元素插入排序后取中位数 */
            uint16_t x = a[k]; int j = k - 1;
            while (j >= 0 && a[j] > x) { a[j+1] = a[j]; j--; }
            a[j+1] = x;
        }
        uint16_t med = a[2];
        int d = (int)buf[i] - (int)med; if (d < 0) d = -d;
        if ((float)d > T) buf[i] = med;
    }
}

/* 对外提供截峰后的稳健跨度，供采集层作为逐帧一致性的判据。
   原始 max/min 由边沿过冲与孤立毛刺决定（能否采到过冲峰取决于采样相位），
   截峰后的分位统计代表两条平顶的位置，随采样相位基本不变。 */
void Measure_RobustSpan(const uint16_t *buf, uint16_t n, uint16_t *hi, uint16_t *lo)
{
    if (hi) *hi = 0;
    if (lo) *lo = 0;
    if (buf == NULL || n < 2) return;
    robust_minmax(buf, n, hi, lo);
}

Signal_t Measure_Wave(const uint16_t *buf, uint16_t n, float fs)
{
    /* 波形显示模式：处理任意波形，Vpp 取真实峰峰值（用分位截峰抑制噪声），不能按正弦波用 RMS 推算。
       屏上波形由应用层直接取 buf 交给显示函数绘制，本函数只给出 Vpp、freq 与 RMS 指标。 */
    Signal_t s = { .vdc = 0, .vpp = 0, .freq = 0, .rms = 0, .valid = 0 };
    if (n == 0) return s;

    float mean; uint16_t vmax, vmin, cnt = 0;
    Measure_Stats(buf, n, &mean, &vmax, &vmin);
    uint16_t rhi, rlo;
    robust_minmax(buf, n, &rhi, &rlo);                          /* 分位截峰后的谷与峰 */
    s.vdc  = rawf_to_volt(mean);
    s.vpp  = Measure_RawToVolt(rhi) - Measure_RawToVolt(rlo);   /* 标定存在非线性：各自换算后再相减 */
    s.freq = Measure_Freq(buf, n, fs, mean, &cnt);
    float rms_raw = Measure_RMS(buf, n, mean);
    float hi = mean + rms_raw; if (hi > ADC_FULLSCALE) hi = ADC_FULLSCALE;
    s.rms = fabsf(rawf_to_volt(hi) - s.vdc);

    s.valid = (uint8_t)((cnt >= 2) && (s.vpp > 0.0f) && (fs > 0.0f)
                        && (mean > 8.0f) && (mean < 4087.0f));
    return s;
}

/* ===== 谐波重构去噪（供波形模式的 Vpp 使用）=====
   折叠得到的单周期是周期信号：能量集中在基波与少数谐波上，噪声为宽带。对 128 点做 FFT，
   把幅度低于"噪声底（中位数）×HARM_GATE"的频点连同其共轭镜像一起置零，保留直流与真实谐波，
   再作逆变换，可在不削减真实峰值的前提下去除噪声（固定阶数截断会在方波边沿产生 Gibbs 过冲）。
   正弦波只保留基波，接近无失真；方波与三角波保留幅度衰减到噪声底以内的谐波，平顶与尖角不失真。 */
#define HARM_GATE 3.0f

static void fft_inplace(float *re, float *im, int n, int inv)
{
    for (int i = 1, j = 0; i < n; i++) {                 /* 位反转置换 */
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { float t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {             /* 蝶形运算 */
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
    if (n != 128) { for (int i = 0; i < n; i++) out[i] = in[i]; return; }   /* 仅支持 128 点（2 的 7 次方） */
    static float re[128], im[128];
    for (int i = 0; i < 128; i++) { re[i] = (float)in[i]; im[i] = 0.0f; }
    fft_inplace(re, im, 128, 0);

    float mag[65];
    for (int k = 0; k <= 64; k++) mag[k] = sqrtf(re[k] * re[k] + im[k] * im[k]);
    float tmp[64];
    for (int k = 1; k <= 64; k++) tmp[k - 1] = mag[k];        /* 噪声底取谐波幅度的中位数 */
    for (int i = 1; i < 64; i++) { float x = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > x) { tmp[j + 1] = tmp[j]; j--; } tmp[j + 1] = x; }
    float thr = HARM_GATE * tmp[31];                          /* tmp[31] 为升序排列后的中位数 */

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

/* ===== 全变差（TV/ROF）去噪 =====
   目标函数 min_x 0.5‖x−y‖² + λ‖Dx‖₁，用 Chambolle 对偶投影迭代求解。属于保边去噪：在压制宽带噪声的
   同时保留方波的台阶与三角波的尖角（Fourier 截断会带来 Gibbs 效应并把拐角削圆），因此任意波形的峰谷都稳定。 */
void Measure_TVDenoise(const uint16_t *in, uint16_t n, float lam, uint16_t *out)
{
    if (n < 3 || n > 256) { for (int i = 0; i < n; i++) out[i] = in[i]; return; }
    static float x[256], p[256];
    float tau = 0.24f;
    for (int i = 0; i < n - 1; i++) p[i] = 0.0f;
    for (int it = 0; it < 200; it++) {
        for (int i = 0; i < n; i++) {                 /* x = y − Dᵀp，其中 (Dᵀp)[i]=p[i-1]−p[i] */
            float dtp; if (i == 0) dtp = -p[0]; else if (i == n - 1) dtp = p[n - 2]; else dtp = p[i - 1] - p[i];
            x[i] = (float)in[i] - dtp;
        }
        for (int i = 0; i < n - 1; i++) {             /* p ← Proj_{|p|≤λ}(p + τ·Dx) */
            float v = p[i] + tau * (x[i + 1] - x[i]);
            if (v > lam) v = lam; if (v < -lam) v = -lam;
            p[i] = v;
        }
    }
    for (int i = 0; i < n; i++) {
        float v = x[i]; if (v < 0.0f) v = 0.0f; if (v > 4095.0f) v = 4095.0f;
        out[i] = (uint16_t)(v + 0.5f);
    }
}
