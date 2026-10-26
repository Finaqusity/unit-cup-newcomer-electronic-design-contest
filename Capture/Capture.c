/* ===== Capture.c —— 采集调度层实现 =====
 * 职责：自动量程（粗测→选档→精测）、首帧有效门控（丢弃换挡过渡帧）与帧管理。
 * 依赖方向单向：本文件同时包含 ADC.h 与 Measure.h，是全工程中唯一衔接二者的层次，
 * ADC 与 Measure 之间不产生相互依赖。
 */
#include "Capture.h"
#include "ADC.h"
#include <stddef.h>     /* NULL：auto_range_settle 超时时返回空指针 */
#include <math.h>       /* floorf/sqrtf/fabsf：相干折叠与显示映射 */

/* 一帧的最长等待时间(ms)。最低档 20kHz 下 1024 点约需 51.2ms，此上限留出余量，
   同时避免信号断开时长时间阻塞。 */
#define CAP_TMO_MS  100u

/* 按键事件是否有未读记录（定义在 Key.c）。此处只声明不包含 Key.h：采集层不需要 GPIO 与
   消抖细节，PC 端测试也只需提供同名桩函数即可链接。多帧采集在帧间检查该事件，一旦有按键
   就以已采到的帧立即输出结果，把按键响应的最坏延迟从一次完整测量（DC 约 82ms，
   低频 AC/Wave 约 200~410ms）压缩到一帧时长。 */
extern uint8_t Key_AnyPending(void);

/* 三档采样率。每档 fs 均可被 TIM3 计数时钟 3.6MHz 整除，因此实际采样率与标称值精确相等，
   不会因分频取整引入频率测量系统误差。
   频段划分依据：缓冲长度 N 固定为 1024，同时满足每周期约 20 个以上采样点（保证过零检测
   与波形分辨率）和窗内约 4 个以上完整周期（保证频率与 RMS 统计量可靠）：
     R_LOW  约 80Hz–1kHz    R_MID 约 0.8–6kHz    R_HIGH 约 1.6–20kHz   */
static const AdRange R_LOW  = {  20000 };
static const AdRange R_MID  = { 200000 };
static const AdRange R_HIGH = { 400000 };

/* 档位锁定：记录 ADC 当前实际所在的采样率档，供连续的 AC/Wave 测量直接复用，
   避免每次重跑自动量程的三帧开销（丢过渡帧 + 粗测 + 换挡后再丢一帧）。
   复用即不换挡，不产生过渡帧，因此无需丢弃。
   锁定失效的三种情况：首次测量；上一次测得频率已越出当前档覆盖范围（pick_range 判为另一档）；
   被 Capture_Unlock、Capture_DC 或模式切换显式复位。NULL 表示未锁定、下次需完整扫描。 */
static const AdRange *g_lock = NULL;

/* Wave 折叠"混帧作废"的连续次数：作废 3 次后接受本次折叠，
   防止信号缓慢漂移时每次都判为混帧而使显示长期不更新。 */
static uint8_t wave_abort = 0;

/* 采集一帧：重新启动 DMA 传输并等待一帧采满。返回 1 表示成功，0 表示超时 */
static uint8_t grab(void)
{
    AD_Start();
    return AD_WaitFrame(CAP_TMO_MS);
}

/* 根据粗测频率与窗内周期数选择采样率档 */
static const AdRange* pick_range(float f, uint16_t cnt)
{
    if (cnt < 2)     return &R_LOW;    // 窗内不足 2 个周期：频率偏低，需要拉长观测窗口
    if (f < 1000.0f) return &R_LOW;    // 每周期采样点数不足：降低 fs 以提高时间分辨率
    if (f < 6000.0f) return &R_MID;
    return &R_HIGH;                    // 高频段：提高 fs 以维持每周期点数并避免混叠
}

void Capture_Init(void)
{
    AD_Init();
    AD_SetRange(&R_MID);
    g_lock = NULL;
}

/* 显式解除档位锁定：模式切换、进出标定（CAL）等信号来源或用途可能改变时调用，
   强制下一次测量重新执行完整自动量程。 */
void Capture_Unlock(void) { g_lock = NULL; }

/* 显式门控入口：模式或量程被外部改动后调用，丢弃一帧过渡数据 */
void Capture_Restart(void)
{
    grab();                            // 该帧仅用于让模拟前端稳定，直接丢弃
}

/* 直流多帧平均帧数：噪声按 √N 下降。16 帧（约 80ms）把 ADC 引脚电压抖动从约 1mV 降到约
   0.7mV，折算到输入端（分压比 3.85）约为 4mV→2.7mV。 */
#define DC_AVG_FRAMES 16

CaptureResult Capture_DC(void)
{
    CaptureResult r = { .sig = {0}, .fs_used = R_MID.fs_hz,
                        .disp_start = 0, .disp_len = ADC_BUF_LEN };
    AD_SetRange(&R_MID);
    g_lock = NULL;                         // DC 把 ADC 设回 MID 档，AC/Wave 的档位锁定随之失效
    grab();                              // 丢弃换挡后的过渡帧

    float acc = 0.0f;
    uint16_t done = 0;
    for (uint16_t k = 0; k < DC_AVG_FRAMES; k++) {
        if (done && Key_AnyPending()) break;      /* 已有按键等待：用已采到的帧立即输出 */
        if (!grab()) break;
        float mean; uint16_t vmax, vmin;
        Measure_Stats(ADC_Buf, ADC_BUF_LEN, &mean, &vmax, &vmin);
        acc += mean; done++;
    }
    if (done == 0) { r.sig.valid = 0; return r; }

    float mavg = acc / (float)done;
    r.raw_mean  = mavg;                        /* 供标定采锚：与显示使用同一次多帧平均结果 */
    r.sig.vdc   = Measure_MeanToVolt(mavg);
    r.sig.valid = (uint8_t)((mavg > 8.0f) && (mavg < 4087.0f));   // 用均值判断饱和，不受单个毛刺样本影响
    return r;
}

/* 自动量程并丢弃换挡过渡帧：返回选定的量程（超时返回 NULL），
   返回时缓冲区中已是一帧换挡后的稳定数据。 */
static const AdRange* auto_range_settle(void)
{
    AD_SetRange(&R_MID);
    grab();                              // 丢弃换挡过渡帧
    if (!grab()) return NULL;            // 粗测帧

    float mean; uint16_t vmax, vmin, cnt = 0;
    Measure_Stats(ADC_Buf, ADC_BUF_LEN, &mean, &vmax, &vmin);
    float f0 = Measure_Freq(ADC_Buf, ADC_BUF_LEN, (float)R_MID.fs_hz, mean, &cnt);

    const AdRange *sel = pick_range(f0, cnt);
    AD_SetRange(sel);
    grab();                              // 丢弃换挡后的过渡帧
    return sel;
}

/* 交流稳态多帧数：整数周期 RMS 单帧已基本无频谱泄漏，多帧只用于压低噪声。
   噪声进入 Vpp 是二阶小量（σ²/2V²），4 帧相对 8 帧的精度差异小于 0.01%（信号 ≥0.3Vpp），
   而低频档一帧约 51ms，取 4 帧可把该档刷新时间约减半。 */
#define AC_AVG_FULL  4
#define AC_AVG_FAST  2                  // 信号未稳定时的快档：结果由上层稳定门控拦下，仅用于保持跟随性
#define WAVE_AVG_FULL  8               // 波形稳态折叠帧数：Vpp 取实际峰峰值，噪声进入峰宽是一阶量，故保持 8 帧
#define WAVE_AVG_FAST  4               // 波形未稳快档：2 帧的折叠样本数不足，平顶抖动与梳状失真会明显可见，故取 4

CaptureResult Capture_AC(uint8_t fast)
{
    CaptureResult r = { .sig = {0}, .fs_used = R_MID.fs_hz,
                        .disp_start = 0, .disp_len = ADC_BUF_LEN };

    const AdRange *sel;
    if (g_lock) sel = g_lock;                 // 已锁定：复用该档，跳过自动量程的三帧丢弃/粗测开销
    else        sel = auto_range_settle();     // 首次、解锁或跨频段：执行完整自动量程（内含换挡丢弃帧）
    if (!sel) { g_lock = NULL; r.sig.valid = 0; return r; }
    g_lock = sel;
    r.fs_used = sel->fs_hz;

    /* 连续采集 frames 帧，逐帧调用 Measure_AC，对有效帧的 vdc/vpp/freq/rms 求平均 */
    uint16_t frames = fast ? AC_AVG_FAST : AC_AVG_FULL;
    float sv = 0, sp = 0, sf = 0, sr = 0;
    uint16_t done = 0;
    for (uint16_t k = 0; k < frames; k++) {
        if (done && Key_AnyPending()) break;      /* 已有按键等待：立即输出，无需采完全部帧 */
        if (!grab()) break;
        Measure_Deimpulse(ADC_Buf, ADC_BUF_LEN);     /* 剔除孤立尖峰与过冲：RMS 对单点大幅偏差呈平方敏感 */
        Signal_t si = Measure_AC(ADC_Buf, ADC_BUF_LEN, (float)sel->fs_hz);
        if (!si.valid) continue;
        sv += si.vdc; sp += si.vpp; sf += si.freq; sr += si.rms; done++;
    }
    if (done == 0) { g_lock = NULL; r.sig.valid = 0; return r; }

    r.sig.vdc   = sv / done;
    r.sig.vpp   = sp / done;
    r.sig.freq  = sf / done;
    r.sig.rms   = sr / done;
    r.sig.valid = 1;

    /* 频段自检：若测得频率已越出当前档覆盖的频段，则解除锁定，下次重新执行完整自动量程
       （含换挡丢弃帧）。本次测量可能在旧档完成，精度略降但结果通常仍有效，下一帧即校正。 */
    { const AdRange *need = pick_range(r.sig.freq, 999u); if (need != g_lock) g_lock = NULL; }
    return r;
}

/* ===== 相干折叠：多帧按相位叠加成一个单周期波形，供任意波 Vpp 计算与显示共用 =====
   噪声随机而信号周期重复，同相位样本平均后噪声按 √(帧数×每帧周期数) 下降，
   方波、三角波、正弦波等任意波形形状完整保留。 */
#define WAVE_FOLD_PTS 128
/* ===== 波形显示的疏密规则：调整周期密度只需修改 WAVE_C1K 与 WAVE_ALPHA =====
   赛题的波形显示项要求在 1k~10kHz 范围内显示任意波，并至少包含一个完整周期、无明显畸变或突变。
   一屏内的周期数按幂律压缩：  Cf = WAVE_C1K × (f / 1kHz)^WAVE_ALPHA
     · α=1   → 线性时基（示波器习惯）：10kHz 需画 16 个周期，每周期仅约 8 像素，
               方波边沿退化为色块，容易被判为显示畸变
     · α=0.6 → 默认取值：1kHz→1.2 个周期、5kHz→3.6、10kHz→5.9（每周期约 21 像素），
               疏密仍随频率单调增长但不会过密
   需要更稀疏的显示可减小 WAVE_C1K 或 WAVE_ALPHA；需要接近示波器的线性观感可把 WAVE_ALPHA 取 1。 */
#define WAVE_C1K      1.2f    /* 1kHz 时一屏显示的周期数（大于 1 才能满足"至少显示一个周期"） */
#define WAVE_ALPHA    0.6f    /* 疏密随频率增长的指数：0 为周期数恒定，1 为线性时基 */
#define WAVE_MAX_CYC  24      /* 周期数上限：频率异常时 powf 结果可能过大，此处截断以保证绘制稳定 */
#define WAVE_ENV_BIN  16      /* 每列覆盖的 bin 数达到此值（约每周期 8 像素）时改画 min/max 包络竖线；
                                 赛题规定的 1k~10kHz 范围不会触发，折线显示的形状最完整，
                                 包络仅作为超高频时的默认处理方式 */
uint16_t Capture_WaveLo[WAVE_FOLD_PTS];   /* 每列覆盖区间的下包络（原始码值） */
uint16_t Capture_WaveHi[WAVE_FOLD_PTS];   /* 每列覆盖区间的上包络（原始码值） */
uint8_t  Capture_WaveEnv = 0;             /* 1 表示按包络（竖线）绘制；0 表示按折线绘制 Capture_WaveBuf */
uint16_t Capture_WaveBuf[WAVE_FOLD_PTS];   /* 折叠得到的单周期波形（原始码值），供显示层直接绘制 */
uint16_t Capture_WaveLen = 0;
static float f_sum[WAVE_FOLD_PTS], f_wt[WAVE_FOLD_PTS], f_ss[WAVE_FOLD_PTS];

/* 把一帧按相位线性插值累加进折叠缓冲；first 为该帧首个上升过零位置，P 为每周期采样点数 */
static void fold_frame(const uint16_t *buf, uint16_t n, float first, float P)
{
    if (P <= 1.0f) return;
    for (uint16_t i = 1; i < n; i++) {
        float ph = (float)(i - first) / P;
        ph -= floorf(ph);                       /* 折叠到 0..1 相位区间 */
        float x = ph * (float)WAVE_FOLD_PTS;
        int b = (int)x; if (b >= WAVE_FOLD_PTS) b = WAVE_FOLD_PTS - 1;
        float fr = x - (float)b;
        int b1 = (b + 1) % WAVE_FOLD_PTS;
        float s = (float)buf[i], s2 = s * s;
        f_sum[b]  += s  * (1.0f - fr); f_wt[b]  += (1.0f - fr); f_ss[b]  += s2 * (1.0f - fr);
        f_sum[b1] += s  * fr;          f_wt[b1] += fr;          f_ss[b1] += s2 * fr;
    }
}

CaptureResult Capture_Wave(uint8_t fast)
{
    CaptureResult r = { .sig = {0}, .fs_used = R_MID.fs_hz,
                        .disp_start = 0, .disp_len = ADC_BUF_LEN };

    const AdRange *sel;
    if (g_lock) sel = g_lock;                 // 复用已锁定的档位，跳过自动量程的丢弃帧与粗测帧
    else        sel = auto_range_settle();
    if (!sel) { g_lock = NULL; r.disp_len = ADC_BUF_LEN; r.sig.valid = 0; return r; }
    g_lock = sel;
    r.fs_used = sel->fs_hz;
    float fs_hz = (float)sel->fs_hz;

    for (int p = 0; p < WAVE_FOLD_PTS; p++) { f_sum[p] = 0.0f; f_wt[p] = 0.0f; f_ss[p] = 0.0f; }

    uint16_t wframes = fast ? WAVE_AVG_FAST : WAVE_AVG_FULL;
    float f_acc = 0.0f; uint16_t done = 0;
    /* 混帧检测：一次折叠需要连续采集多帧并按相位叠加。若中途输入信号被改变，前半来自旧波形、
       后半来自新波形，折叠出的单周期即为两者的混合，表现为赛题禁止的畸变与突变。
       判据为逐帧与首帧比较频率和峰峰跨度（用原始码值跨度作为幅度的代理量）；
       噪声本身只会带来 1~2% 的起伏，故门限放宽到 3% 与 12% 以避免误判。
       不一致时本次折叠作废：不写显示缓冲，屏上保持上一次有效波形，由上层快轮询立即重新折叠。
       连续作废 3 次后接受本次结果，防止信号缓慢漂移时显示长期不更新。 */
    float f_ref = 0.0f, s_ref = 0.0f; uint8_t dirty = 0;
    for (uint16_t k = 0; k < wframes; k++) {
        if (done && Key_AnyPending()) break;      /* 已有按键等待：提前结束折叠帧的采集 */
        if (!grab()) break;
        /* 折叠路径不做削峰处理：削峰会改动样本值，使方波平顶与边沿出现失真，
           而显示要求逐样本保持输入波形形状。异常信号由 Measure_FreqInfo 的稳健阈值判定，
           该过程不修改任何样本值。 */
        float mean; uint16_t vmax, vmin, cnt = 0;
        Measure_Stats(ADC_Buf, ADC_BUF_LEN, &mean, &vmax, &vmin);
        float first = 0.0f, P = 0.0f;
        float fq = Measure_FreqInfo(ADC_Buf, ADC_BUF_LEN, fs_hz, mean, &cnt, &first, &P);
        if (cnt < 2 || fq <= 0.0f || P <= 1.0f) continue;
        float sp = (float)(vmax - vmin);
        if (done == 0) { f_ref = fq; s_ref = sp; }
        else if (fabsf(fq - f_ref) > 0.03f * f_ref || fabsf(sp - s_ref) > 0.12f * s_ref) { dirty = 1; break; }
        fold_frame(ADC_Buf, ADC_BUF_LEN, first, P);
        f_acc += fq; done++;
    }
    if (done == 0) { g_lock = NULL; r.disp_start = 0; r.disp_len = ADC_BUF_LEN; r.sig.valid = 0; return r; }
    if (dirty && wave_abort < 3) { wave_abort++; r.disp_start = 0; r.disp_len = WAVE_FOLD_PTS; r.sig.valid = 0; return r; }
    wave_abort = 0;

    /* 归一化得到单周期波形 period_buf[]。
       空 bin 不能用整帧均值填充：方波与矩形波的均值恰好位于两个平顶的正中间，
       填充后表现为自平顶垂向中线的尖刺，同时抬高 max/min，直接造成 Vpp 测量误差；
       权重过小的 bin（只落入一两个样本）同理，该样本可能正是毛刺或边沿的中间值。
       判据：权重不小于平均每 bin 权重的 0.5 倍才算可靠 bin，
       其余（空或欠采样）用左右最近的可靠 bin 做环形线性插值填充，
       使平顶保持平坦、边沿保持陡峭。 */
    static uint16_t period_buf[WAVE_FOLD_PTS];
    float wtot = 0.0f;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) wtot += f_wt[p];
    float wok = 0.5f * wtot / (float)WAVE_FOLD_PTS;      /* 可靠 bin 的权重下限 */

    uint8_t good[WAVE_FOLD_PTS];
    int ngood = 0;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) {
        good[p] = (f_wt[p] >= wok && f_wt[p] > 0.0f) ? 1u : 0u;
        if (good[p]) { period_buf[p] = (uint16_t)(f_sum[p] / f_wt[p] + 0.5f); ngood++; }
    }
    if (ngood == 0) { r.disp_start = 0; r.disp_len = ADC_BUF_LEN; r.sig.valid = 0; return r; }

    for (int p = 0; p < WAVE_FOLD_PTS; p++) {            /* 不可靠 bin 用左右最近可靠 bin 环形插值 */
        if (good[p]) continue;
        int bl = p, br = p;
        for (int k = 1; k < WAVE_FOLD_PTS; k++) { int q = p - k; if (q < 0) q += WAVE_FOLD_PTS; if (good[q]) { bl = q; break; } }
        for (int k = 1; k < WAVE_FOLD_PTS; k++) { int q = p + k; if (q >= WAVE_FOLD_PTS) q -= WAVE_FOLD_PTS; if (good[q]) { br = q; break; } }
        if (bl == br) { period_buf[p] = period_buf[bl]; continue; }
        int dl = (p - bl + WAVE_FOLD_PTS) % WAVE_FOLD_PTS;
        int dr = (br - p + WAVE_FOLD_PTS) % WAVE_FOLD_PTS;
        period_buf[p] = (uint16_t)((float)period_buf[bl] * (float)dr / (float)(dl + dr)
                                 + (float)period_buf[br] * (float)dl / (float)(dl + dr) + 0.5f);
    }
    float fsum = 0.0f; int fcnt = 0;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) { fsum += (float)period_buf[p]; fcnt++; }
    float fmean = fsum / (float)fcnt;

    /* Vpp：直接取折叠单周期的 max/min。折叠已按 √(帧数×每帧周期数) 压低噪声，峰峰值足够稳定。
       全变差去噪、谐波重构、σ 偏差修正等方案对精度改善有限，而计算耗时明显增加
       （全变差需 200 次迭代并含排序），故本路径不采用；
       Measure_TVDenoise 与 Measure_ReconHarm 作为可选实现保留在算法库中，此处不调用。 */
    uint16_t mx = 0, mn = 0xFFFF;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) { if (period_buf[p] > mx) mx = period_buf[p]; if (period_buf[p] < mn) mn = period_buf[p]; }
    r.sig.vpp   = Measure_RawToVolt(mx) - Measure_RawToVolt(mn);

    float fsq = 0.0f;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) { float d = (float)period_buf[p] - fmean; fsq += d * d; }
    float frms = sqrtf(fsq / (float)WAVE_FOLD_PTS);
    r.sig.vdc   = Measure_MeanToVolt(fmean);
    r.sig.freq  = f_acc / (float)done;
    float hr = fmean + frms; if (hr > 4095.0f) hr = 4095.0f;
    r.sig.rms   = fabsf(Measure_MeanToVolt(hr) - r.sig.vdc);
    r.sig.valid = 1;

    /* 频段自检（同 AC）：测得频率越出当前档则解锁，下次重新执行完整自动量程 */
    { const AdRange *need = pick_range(r.sig.freq, 999u); if (need != g_lock) g_lock = NULL; }

    /* 显示：把折叠得到的单周期按频率平铺成 C 个周期（固定时间窗），使疏密随频率变化，
       避免不同频率下出现与输入无关的静态波形。
       周期数取分数并配合线性插值取样：C 不再四舍五入为整数，疏密随频率连续单调变化，
       不会出现逐级跳变；相邻列也不会重复或跳过 bin
       （截断取整会使每个周期边缘错位半格，显示上表现为锯齿与不对齐）。 */
    float Cf = WAVE_C1K * powf(r.sig.freq / 1000.0f, WAVE_ALPHA);  /* 幂律压缩；α=1 时退化为线性时基 */
    if (Cf < 1.0f) Cf = 1.0f;                                    /* 低频：至少铺满 1 个周期 */
    if (Cf > (float)WAVE_MAX_CYC) Cf = (float)WAVE_MAX_CYC;
    Capture_WaveEnv = (Cf >= (float)WAVE_ENV_BIN) ? 1u : 0u;     /* 密到每周期约 8 像素以下时改画包络 */
    for (int j = 0; j < WAVE_FOLD_PTS; j++) {
        float pos = (float)j * Cf / (float)WAVE_FOLD_PTS;         /* 单位：周期（允许小数） */
        pos -= floorf(pos);                                       /* 折算到 0..1 相位 */
        float xf = pos * (float)WAVE_FOLD_PTS;                    /* 0..128 的浮点 bin 位置 */
        int   b  = (int)xf;  if (b >= WAVE_FOLD_PTS) b = WAVE_FOLD_PTS - 1;
        int   b1 = (b + 1) % WAVE_FOLD_PTS;
        float fr = xf - (float)b;
        Capture_WaveBuf[j] = (uint16_t)((float)period_buf[b] * (1.0f - fr)
                                      + (float)period_buf[b1] * fr + 0.5f);
        /* 本列覆盖 cnt 个 bin，取其 min/max 作为上下包络。
           逐点抽稀在每周期约 8 像素以下时会直接漏掉峰谷（幅度偏小、方波变斜），
           包络方式则保证峰与谷一定出现在屏上。 */
        int cnt = (Cf >= 1.0f) ? (int)(Cf + 1.5f) : 1;
        uint16_t mn = 0xFFFF, mx = 0;
        for (int k = 0; k < cnt; k++) {
            uint16_t v = period_buf[(b + k) % WAVE_FOLD_PTS];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        Capture_WaveLo[j] = mn;
        Capture_WaveHi[j] = mx;
    }
    Capture_WaveLen = WAVE_FOLD_PTS;              /* 显示点数：波形缓冲区有效长度 */
    r.disp_start = 0; r.disp_len = WAVE_FOLD_PTS;
    return r;
}
