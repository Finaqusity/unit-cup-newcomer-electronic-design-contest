/* DcEstim.c —— 直流采样估计器实现（纯算法，PC 与 Keil 同源可编译）。
 * 仅在直流显示路径被调用，算法选择由头文件中的 DC_ESTIM 决定。 */
#include "DcEstim.h"

static float buf[DC_EST_WIN];     /* 最近若干次测量值（环形，[0] 为最新） */
static uint8_t cnt = 0;           /* 已填充的个数（不超过 DC_EST_WIN） */
static uint8_t seeded = 0;        /* EMA/HAMPEL/CONF/CONF2/ADAPT 的单值状态是否已初始化 */
static float   ex = 0.0f;         /* EMA/HAMPEL/CONF/CONF2/ADAPT 的内部估计状态 */
static int8_t  pend_dir = 0;      /* CONF 系列：挂起的偏差方向(+1/-1)，0 表示无 */
static uint8_t pend_n = 0;        /* CONF 系列：同向大偏差的连续帧数 */

static void ring_push(float v)
{
    for (int8_t i = (int8_t)cnt - 1; i > 0; i--) buf[i] = buf[i - 1];
    buf[0] = v;
    if (cnt < DC_EST_WIN) cnt++;
}

/* 把当前窗口升序排序到 out（不改动 buf），返回元素个数。始终编译：switch 的各分支都可能引用 */
static uint8_t sorted_copy(float *out)
{
    for (uint8_t i = 0; i < cnt; i++) out[i] = buf[i];
    for (uint8_t i = 1; i < cnt; i++) {
        float key = out[i]; int8_t j = (int8_t)i - 1;
        while (j >= 0 && out[j] > key) { out[j + 1] = out[j]; j--; }
        out[j + 1] = key;
    }
    return cnt;
}
static float med_sorted(const float *s, uint8_t n) { return s[n / 2]; }

void DcEstim_Reset(void)
{
    cnt = 0; seeded = 0; ex = 0.0f; pend_dir = 0; pend_n = 0;
    for (uint8_t i = 0; i < DC_EST_WIN; i++) buf[i] = 0.0f;
}

/* 挂起态：CONF/CONF2/ADAPT 用 pend_n 记录连续同向偏差的帧数，大于 0 表示正在确认一个候选台阶、
   估计值尚未切换到新电平。其余算法不修改 pend_n（恒为 0），因此本函数恒返回 0。 */
uint8_t DcEstim_Pending(void) { return (pend_n != 0) ? 1u : 0u; }

float DcEstim_Feed(float v)
{
    switch (DC_ESTIM) {

    case DC_EST_BLOCK: {           /* 窗口算术均值 */
        ring_push(v);
        float s = 0.0f; for (uint8_t i = 0; i < cnt; i++) s += buf[i];
        return s / (float)cnt;
    }

    case DC_EST_MEDIAN: {          /* 窗口中位数 */
        ring_push(v);
        float t[DC_EST_WIN]; uint8_t n = sorted_copy(t);
        return med_sorted(t, n);
    }

    case DC_EST_TRIM: {            /* 截尾均值：去掉最高与最低各 K 个后再平均 */
        ring_push(v);
        float t[DC_EST_WIN]; uint8_t n = sorted_copy(t);
        uint8_t k = DC_EST_TRIM_K;
        if (n <= (uint8_t)(2 * k)) k = 0;                 /* 样本数不足时退化为整体均值 */
        float s = 0.0f; uint8_t m = 0;
        for (uint8_t i = k; i < n - k; i++) { s += t[i]; m++; }
        return m ? s / (float)m : v;
    }

    case DC_EST_EMA: {             /* 指数滑动均值 */
        if (!seeded) { ex = v; seeded = 1; return ex; }
        ex += DC_EST_ALPHA * (v - ex);
        return ex;
    }

    case DC_EST_HAMPEL: {
        /* 滚动窗中位数与 MAD：偏离超过 K×MAD 的样本判为疑似野值并钳回中位数，再送入 EMA，
           同时获得野值抑制与平滑。 */
        ring_push(v);
        float t[DC_EST_WIN]; uint8_t n = sorted_copy(t);
        float med = med_sorted(t, n);
        float dev[DC_EST_WIN];
        for (uint8_t i = 0; i < n; i++) { float d = t[i] - med; dev[i] = d < 0 ? -d : d; }
        float dt[DC_EST_WIN]; for (uint8_t i = 0; i < n; i++) dt[i] = dev[i];
        for (uint8_t i = 1; i < n; i++) { float key = dt[i]; int8_t j = (int8_t)i - 1;
            while (j >= 0 && dt[j] > key) { dt[j + 1] = dt[j]; j--; } dt[j + 1] = key; }
        float mad = 1.4826f * dt[n / 2]; if (mad < 1e-4f) mad = 1e-4f;  /* 1.4826 使 MAD 在高斯噪声下与标准差一致；下限避免 MAD 为 0 时判据失效 */
        float yc = ( (v - med < 0 ? med - v : v - med) > DC_EST_MADK * mad ) ? med : v;
        if (!seeded) { ex = yc; seeded = 1; return ex; }
        ex += DC_EST_ALPHA * (yc - ex);
        return ex;
    }

    case DC_EST_CONF: {
        /* 确认门限（confirm-gate）：偏差超过门限且连续 DC_EST_CONFIRM 帧同向时判为真实跳变，
         *   直接采纳新电平；孤立的大偏差被丢弃；带内小偏差由 EMA 压低稳态抖动。
         *   该结构同时满足跟随阶跃快与抑制坏帧两项要求。 */
        if (!seeded) { ex = v; seeded = 1; return ex; }
        float d = v - ex;
        float ad = d < 0 ? -d : d;
        if (ad > DC_EST_JUMP) {
            int8_t sgn = (d > 0) ? 1 : -1;
            if (pend_dir == sgn) {
                if (++pend_n >= DC_EST_CONFIRM) { ex = v; pend_dir = 0; pend_n = 0; }  /* 确认成立：跳到新电平 */
            } else { pend_dir = sgn; pend_n = 1; }                                       /* 首次出现或反向：进入挂起，本帧不采纳 */
            return ex;
        }
        pend_dir = 0; pend_n = 0;                                                        /* 回到带内：清除挂起状态 */
        ex += DC_EST_ALPHA * d;
        return ex;
    }

    case DC_EST_CONF2: {
        /* CONF 的按偏差大小分档版本：
         *   大于 HUGE 的偏差远超噪声范围，判定为输入确实改变，立即采纳新值并清空记忆，1 帧跟上；
         *   介于 JUMP 与 HUGE 之间的偏差可能为真实台阶也可能为坏帧，仍需连续 CONFIRM 帧同向才跳变；
         *   小于 JUMP 的偏差按噪声处理，用 EMA 平滑以保持稳态读数稳定。 */
        if (!seeded) { ex = v; seeded = 1; return ex; }
        float d = v - ex;
        float ad = d < 0 ? -d : d;
        if (ad > DC_EST_HUGE) {                                                          /* 大偏差：判为输入变化，立即采纳并清空记忆 */
            ex = v; pend_dir = 0; pend_n = 0; return ex;
        }
        if (ad > DC_EST_JUMP) {                                                          /* 中等偏差：连续同向确认后才跳变 */
            int8_t sgn = (d > 0) ? 1 : -1;
            if (pend_dir == sgn) { if (++pend_n >= DC_EST_CONFIRM) { ex = v; pend_dir = 0; pend_n = 0; } }
            else { pend_dir = sgn; pend_n = 1; }
            return ex;
        }
        pend_dir = 0; pend_n = 0;
        ex += DC_EST_ALPHA * d;                                                          /* 小偏差：按噪声做 EMA 平滑 */
        return ex;
    }

    case DC_EST_ADAPT: {
        /* 持续性自适应：以连续同向偏离而非单次偏差幅度作为判据，因此既能跟随 20mV 量级的小台阶，
         *   又能抑制孤立坏帧。
         *   |d| > HUGE：巨大跳变，1 帧整值采纳并清空记忆。
         *   |d| > SHIFT（噪声地板的约 3σ）：候选真实台阶或毛刺，累计同向帧数，达到 CONFIRM 帧
         *     判为真台阶并整值采纳（ex=v）；未确认前保持不变，使单帧毛刺不影响读数。
         *   |d| ≤ SHIFT：按噪声处理，用慢系数低通平滑，低位读数保持稳定。 */
        if (!seeded) { ex = v; seeded = 1; pend_dir = 0; pend_n = 0; return ex; }
        float d = v - ex;
        float ad = d < 0 ? -d : d;
        if (ad > DC_EST_HUGE) { ex = v; pend_dir = 0; pend_n = 0; return ex; }
        if (ad > DC_EST_SHIFT) {
            int8_t sgn = (d > 0) ? 1 : -1;
            if (pend_dir == sgn) { if (pend_n < 255) pend_n++; }
            else { pend_dir = sgn; pend_n = 1; }
            if (pend_n >= DC_EST_CONFIRM) { ex = v; pend_dir = 0; pend_n = 0; }           /* 持续同向：判为真台阶并整值跟随 */
            return ex;                                                                    /* 未确认：估计值保持不变 */
        }
        pend_dir = 0; pend_n = 0;
        ex += DC_EST_A_SLOW * d;                                                          /* 带内噪声：慢系数平滑 */
        return ex;
    }

    case DC_EST_HISTLOCK:
    default: {
        /* 基线算法：近两次差值小于 DC_EST_TOL 时直接取最新值（可跟上 10mV 量级的台阶）；
           否则取窗口中位数，以剔除单帧毛刺。 */
        float prev = (cnt >= 2) ? buf[1] : 0.0f;
        ring_push(v);
        if (cnt >= 2) {
            float d = buf[0] - prev; if (d < 0) d = -d;
            if (d < DC_EST_TOL) return buf[0];
        }
        if (cnt >= 3) { float t[DC_EST_WIN]; uint8_t n = sorted_copy(t); return med_sorted(t, n); }
        if (cnt == 2) { float a = buf[0], b = buf[1]; return (a < b) ? b : a; } /* 仅有两个样本时取二者中的较大值 */
        return v;
    }
    }
}

char* DcEstim_Name(void)
{
    switch (DC_ESTIM) {
    case DC_EST_HISTLOCK: return "HIST";
    case DC_EST_MEDIAN:   return "MED";
    case DC_EST_BLOCK:    return "BLK";
    case DC_EST_TRIM:     return "TRIM";
    case DC_EST_EMA:      return "EMA";
    case DC_EST_HAMPEL:   return "HAMP";
    case DC_EST_CONF:     return "CONF";
    case DC_EST_CONF2:    return "CNF2";
    case DC_EST_ADAPT:    return "ADPT";
    default:              return "?";
    }
}
