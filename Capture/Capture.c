/*=============================================================
 * Capture.c —— 采集调度层实现
 *   职责：自动量程（粗测→选档→精测）、首次有效数据门控（丢弃过渡帧）与帧管理。
 *   依赖方向保持单向：本文件同时包含 ADC.h 与 Measure.h，是整个工程中唯一同时
 *   引用这两个模块的位置；ADC 与 Measure 之间没有相互引用。
 *============================================================*/
#include "Capture.h"
#include "ADC.h"
#include <stddef.h>     /* NULL：auto_range_settle 超时时返回空指针 */
#include <math.h>       /* floorf/sqrtf/fabsf：相干折叠与统计运算 */

/* 单帧最长等待时间（ms）。最低采样率 20kHz 时 1024 点约需 51.2ms；该取值在预留余量的同时避免长时间阻塞。 */
#define CAP_TMO_MS  100u

/* 查询按键是否存在未读事件（定义在 Key.c）。此处只声明 extern 而不包含 Key.h：Capture 层无需了解
   GPIO 与消抖细节，PC 端单元测试也仍可只用桩函数完成链接。多帧采集在帧与帧之间检查该事件：一旦
   存在待处理按键，立即用已经采集到的帧返回结果，使按键按下到界面响应的最坏延迟由整次测量
   （DC 约 82ms，低频 AC 与 Wave 约 200~410ms）缩短为一帧的时间。 */
extern uint8_t Key_AnyPending(void);

/* 三档量程（采样率均可被 3.6MHz 整除，故不引入系统误差）。每档覆盖频段按
 * “一个周期内的采样点数不少于约 20、且窗口内的周期数不少于约 4” 划分（N 固定为 1024）：
 *   R_LOW ≈ 80Hz–1kHz，R_MID ≈ 0.8–6kHz，R_HIGH ≈ 1.6–20kHz。 */
static const AdRange R_LOW  = {  20000 };
static const AdRange R_MID  = { 200000 };
static const AdRange R_HIGH = { 400000 };

/* 量程锁定（用于提高刷新率）：记录 ADC 当前实际设置的量程档。锁定后连续的 AC 与 Wave 测量直接复用该档，
   不必每次重新执行 auto_range 的三步流程（丢弃过渡帧、粗测、换挡后再次丢弃），即省去 3 帧采集；
   低频档单帧约 51ms，单次测量可节省约 60–110ms。
   仅在以下情况重新执行完整 auto_range：① 首次测量；② 上一帧测得的频率已超出当前档的覆盖范围（pick_range
   判定应换到另一档）；③ 被 Capture_Unlock、Capture_DC 或模式切换强制复位。复用量程表示没有换挡，因此不存在
   过渡帧，也就不必丢弃。指针为 NULL 表示未锁定或需要重新扫描。 */
static const AdRange *g_lock = NULL;

/* Wave 折叠因多帧频率不一致而被作废的连续次数（判定条件见 Capture_Wave 内的注释）。
   连续作废达到 3 次后接受当前折叠结果，以避免信号缓慢漂移时画面长期保持不变。 */
static uint8_t wave_abort = 0;

/* Wave 纵轴 V/div 的当前档位号（0xFF 表示尚未定档，下次调用时重新选择）。分档切换带有回差，见 Capture_WaveRows。
   在模式切换、进入或退出标定等信号可能发生变化的场合，由 Capture_Unlock 复位。 */
static uint8_t wdiv_hold = 0xFF;
#define WDIV_OVER  1.02f   /* 升档余量：允许 2%（不超过 1px）的溢出，使临界抖动不引起分档切换 */
#define WDIV_BACK  0.70f   /* 降档回差：信号幅度明显变小（占用格数不足 70%）时才切换到下一低档 */

/* 采集一帧：重启 DMA 并等待缓冲区写满。返回 1 表示成功，返回 0 表示超时 */
static uint8_t grab(void)
{
    AD_Start();
    return AD_WaitFrame(CAP_TMO_MS);
}

/* 依据粗测得到的频率与窗口内周期数选择量程档 */
static const AdRange* pick_range(float f, uint16_t cnt)
{
    if (cnt < 2)     return &R_LOW;    // 窗口内不足 2 个周期，说明频率偏低，应降低采样率以拉长观测窗口
    if (f < 1000.0f) return &R_LOW;    // 单个周期内的采样点数过少，应降低采样率
    if (f < 6000.0f) return &R_MID;
    return &R_HIGH;                    // 高频段：提高采样率以保证每周期点数并避免混叠
}

/* 说明：波形显示不采用整帧裁剪方式，而由下方 Capture_Wave 的相干折叠生成单周期数据
   （Capture_WaveBuf）供应用层直接绘制。 */

void Capture_Init(void)
{
    AD_Init();
    AD_SetRange(&R_MID);
    g_lock = NULL;
}

/* 显式解锁接口：在信号或测量用途可能发生变化的场合（模式切换、进入或退出标定）调用，强制下次测量重新执行完整 auto_range。 */
void Capture_Unlock(void) { g_lock = NULL; wdiv_hold = 0xFF; }   /* 同时复位纵轴档位，使 V/div 在下一次调用时重新自动选择 */

/* 显式门控入口：模式或量程被外部改动后，丢弃一帧过渡数据 */
void Capture_Restart(void)
{
    grab();                            // 丢弃切换过程中产生的过渡帧
}

#define DC_AVG_FRAMES 16                 // 直流多帧平均：噪声按 √N 下降。16 帧（约 80ms）使 ADC 端抖动由约 1mV 降至约 0.7mV，折算到输入端（×3.85）由约 4mV 降至约 2.7mV

CaptureResult Capture_DC(void)
{
    CaptureResult r = { .sig = {0}, .fs_used = R_MID.fs_hz,
                        .disp_start = 0, .disp_len = ADC_BUF_LEN };
    AD_SetRange(&R_MID);
    g_lock = NULL;                         // DC 测量将 ADC 设回中档，使 AC 与 Wave 的量程锁定失效
    grab();                              // 丢弃换挡后产生的过渡帧

    float acc = 0.0f;
    uint16_t done = 0;
    for (uint16_t k = 0; k < DC_AVG_FRAMES; k++) {
        if (done && Key_AnyPending()) break;      /* 存在待处理按键时，立即用已采集到的帧返回结果 */
        if (!grab()) break;
        float mean; uint16_t vmax, vmin;
        Measure_Stats(ADC_Buf, ADC_BUF_LEN, &mean, &vmax, &vmin);
        acc += mean; done++;
    }
    if (done == 0) { r.sig.valid = 0; return r; }

    float mavg = acc / (float)done;
    r.raw_mean  = mavg;                        /* 供标定采集锚点使用：与显示采用同一多帧均值估计 */
    r.sig.vdc   = Measure_MeanToVolt(mavg);
    r.sig.valid = (uint8_t)((mavg > 8.0f) && (mavg < 4087.0f));   // 用均值判断饱和，使判定不受首样本毛刺影响
    return r;
}

/* 自动量程并丢弃换挡过渡帧：返回所选量程（超时返回 NULL）；返回时缓冲区内已是一帧换挡后的稳定数据 */
static const AdRange* auto_range_settle(void)
{
    AD_SetRange(&R_MID);
    grab();                              // 丢弃换挡过程中产生的过渡帧
    if (!grab()) return NULL;            // 粗测帧：用于估计频率并选择量程

    float mean; uint16_t vmax, vmin, cnt = 0;
    Measure_Stats(ADC_Buf, ADC_BUF_LEN, &mean, &vmax, &vmin);
    float f0 = Measure_Freq(ADC_Buf, ADC_BUF_LEN, (float)R_MID.fs_hz, mean, &cnt);

    const AdRange *sel = pick_range(f0, cnt);
    AD_SetRange(sel);
    grab();                              // 丢弃换挡后产生的过渡帧
    return sel;
}

#define AC_AVG_FULL  4                  // 交流稳态多帧平均：整数周期 RMS 在单帧内已无泄漏误差，多帧平均只抑制噪声；
                                        // 噪声进入 Vpp 为二阶小量（σ²/2V²），帧数由 8 减到 4 的精度损失小于 0.01%（Vpp≥0.3V 时），
                                        // 低频档（R_LOW 单帧 51ms）的刷新时间由 410ms 缩短为 205ms。
#define AC_AVG_FAST  2                  // 信号未稳定时的快速档：结果在稳定判定成立前不对外显示，减少帧数只为缩短响应时间
#define WAVE_AVG_FULL  8               // 波形稳态折叠帧数：Vpp 取真实峰峰值，噪声进入峰宽为一阶量，故保持 8 帧不再缩减
#define WAVE_AVG_FAST  4               // 波形未稳定时的快速档：保持 4 帧；2 帧的折叠样本过少，平顶抖动与梳状毛刺会显现

CaptureResult Capture_AC(uint8_t fast)
{
    CaptureResult r = { .sig = {0}, .fs_used = R_MID.fs_hz,
                        .disp_start = 0, .disp_len = ADC_BUF_LEN };

    const AdRange *sel;
    if (g_lock) sel = g_lock;                 // 已锁定量程：直接复用，跳过 auto_range 的 3 帧丢弃与粗测
    else        sel = auto_range_settle();     // 首次、解锁或跨频段：执行完整自动量程（内含换挡过渡帧丢弃）
    if (!sel) { g_lock = NULL; r.sig.valid = 0; return r; }
    g_lock = sel;
    r.fs_used = sel->fs_hz;

    /* 连续采集 frames 帧，逐帧调用 Measure_AC，并对有效帧的 vdc/vpp/freq/rms 求平均 */
    uint16_t frames = fast ? AC_AVG_FAST : AC_AVG_FULL;
    float sv = 0, sp = 0, sf = 0, sr = 0;
    uint16_t done = 0;
    for (uint16_t k = 0; k < frames; k++) {
        if (done && Key_AnyPending()) break;      /* 存在待处理按键时立即返回结果，不必跑满 4 帧 */
        if (!grab()) break;
        Measure_Deimpulse(ADC_Buf, ADC_BUF_LEN);     /* 剔除孤立毛刺与过冲：RMS 对单点大幅偏差呈平方敏感 */
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

    /* 频段自检：若测得频率已超出当前量程档的覆盖范围，则解除锁定，下次测量执行完整 auto_range 换挡（含过渡帧丢弃）。
       本帧可能在旧量程档下测得，结果略非最优但通常仍然有效；下一帧即被校正。 */
    { const AdRange *need = pick_range(r.sig.freq, 999u); if (need != g_lock) g_lock = NULL; }
    return r;
}

/* ===== 相干折叠：多帧按相位叠加为单个周期，任意波形的 Vpp 计算与绘制共用同一结果 =====
   噪声随机、信号周期重复，因此同相位样本平均后噪声按 √(帧数×每帧周期数) 下降，
   方波、三角波、正弦波等任意波形的形状都能完整保留。 */
/* WAVE_FOLD_PTS 定义在 Capture.h 中：应用层绘制坐标系时需要据此确定数组尺寸。 */
/* ===== 波形疏密显示规律：由 WAVE_C1K 与 WAVE_ALPHA 两个参数决定 =====
   赛题第三项（波形显示）针对 1k~10kHz 的任意波形，要求至少显示一个周期，且无明显畸变或突变。
   单屏显示的周期数按幂律压缩：Cf = WAVE_C1K × (f / 1kHz)^WAVE_ALPHA
     · WAVE_ALPHA = 1 为线性时基（固定时基的示波器行为）：10kHz 需显示 16 个周期，即 8px/周期，
       方波的相邻周期会重叠成实心区域，容易被判定为畸变；
     · WAVE_ALPHA = 0.6 为默认取值：1kHz 显示 1.2 个周期、5kHz 显示 3.6 个周期、10kHz 显示 5.9 个
       周期（约 21px/周期），疏密仍随频率单调增长，但不会过密。
   减小 WAVE_C1K 或 WAVE_ALPHA 可使显示更疏；取 WAVE_ALPHA = 1 可接近示波器的线性时基效果。 */
#define WAVE_C1K      1.2f    /* 1kHz 时单屏显示的周期数；取值大于 1 才能满足"至少显示一个周期"的要求 */
#define WAVE_ALPHA    0.6f    /* 疏密随频率增长的指数：0 表示周期数恒定，1 表示线性时基 */
#define WAVE_MAX_CYC  24      /* 周期数的安全上限：频率异常时 powf 的结果可能过大 */
#define WAVE_ENV_BIN  16      /* 每列覆盖的相位桶数达到该值（相当于不超过 8px/周期）时，自动改为绘制 min/max 包络竖线；
                                 赛题规定的 1k~10kHz 范围内不会触发，仅作为超高频时的保护措施 */
#define WAVE_TRIM_BINS 0     /* Vpp 统计：折叠周期两端各截去的相位桶数。取 0 表示使用真实 max/min——孤立尖峰已
                                 在时域由 Measure_Deimpulse 剔除，无需再按幅值截峰（截峰会使三角波的幅值低估百分之几） */
uint16_t Capture_WaveLo[WAVE_FOLD_PTS];   /* 每列覆盖区间的下包络（码值） */
uint16_t Capture_WaveHi[WAVE_FOLD_PTS];   /* 每列覆盖区间的上包络（码值） */
uint8_t  Capture_WaveEnv = 0;             /* 1 表示改用包络（竖线）绘制；0 表示以折线绘制 Capture_WaveBuf */
float    Capture_WaveCyc = 0.0f;          /* 单屏实际显示的周期数，用于计算 t/div */
uint16_t Capture_WaveBuf[WAVE_FOLD_PTS];   /* 折叠后的单周期数据（码值），供应用层绘制波形 */
uint16_t Capture_WaveLen = 0;
static float f_sum[WAVE_FOLD_PTS], f_wt[WAVE_FOLD_PTS], f_ss[WAVE_FOLD_PTS];

/* 将一帧数据按相位以线性插值累加进折叠缓冲；first 为该帧首个过零点的位置，P 为每个周期的采样点数 */
static void fold_frame(const uint16_t *buf, uint16_t n, float first, float P)
{
    if (P <= 1.0f) return;
    for (uint16_t i = 1; i < n; i++) {
        float ph = (float)(i - first) / P;
        ph -= floorf(ph);                       /* 折算到 0..1 区间 */
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
    if (g_lock) sel = g_lock;                 // 复用已锁定的量程档，跳过 auto_range 的过渡帧丢弃与粗测
    else        sel = auto_range_settle();
    if (!sel) { g_lock = NULL; r.disp_len = ADC_BUF_LEN; r.sig.valid = 0; return r; }
    g_lock = sel;
    r.fs_used = sel->fs_hz;
    float fs_hz = (float)sel->fs_hz;

    for (int p = 0; p < WAVE_FOLD_PTS; p++) { f_sum[p] = 0.0f; f_wt[p] = 0.0f; f_ss[p] = 0.0f; }

    uint16_t wframes = fast ? WAVE_AVG_FAST : WAVE_AVG_FULL;
    float f_acc = 0.0f; uint16_t done = 0;
    /* 多帧一致性检测：一次折叠需要连续采集多帧并按相位叠加。若期间信号频率发生变化，前后帧的周期长度不
       同，折叠得到的单周期会被叠加模糊，不满足赛题对"无明显畸变或突变"的要求。因此逐帧比较频率，偏差
       门限取 5%（远高于噪声引起的抖动，也远小于旋钮一步的改变量）；判定不一致时本次折叠作废：不写入显示
       缓冲，屏幕保持上一次的波形，由下一次轮询重新折叠。
       只检查频率而不检查幅度：两种幅度混合仅使平顶呈两级电平，不构成突变；而幅度类抖动（取决于边沿振铃
       是否被采到）容易造成误判，使屏幕长期停留在"测量中"提示。孤立振铃尖峰已由 Measure_Deimpulse 在时域
       剔除。连续作废 3 次后接受当前结果，以避免信号持续变化时画面长期保持不变。 */
    float f_ref = 0.0f; uint8_t dirty = 0;
    for (uint16_t k = 0; k < wframes; k++) {
        if (done && Key_AnyPending()) break;      /* 存在待处理按键时提前结束折叠帧的采集 */
        if (!grab()) break;
        /* 剔除孤立尖峰：边沿振铃的真实宽度（数十至数百 ns）远小于采样间隔（200kS/s 对应 5µs），在数据中
           表现为单个样本的孤立跳变，正是 5 点中位数判据的处理对象；该处理不削减正弦波与三角波的真实峰值
           （实测偏差为 +0.0mV），也不改变单调边沿。 */
        Measure_Deimpulse(ADC_Buf, ADC_BUF_LEN);
        float mean; uint16_t vmax, vmin, cnt = 0;
        Measure_Stats(ADC_Buf, ADC_BUF_LEN, &mean, &vmax, &vmin);
        float first = 0.0f, P = 0.0f;
        float fq = Measure_FreqInfo(ADC_Buf, ADC_BUF_LEN, fs_hz, mean, &cnt, &first, &P);
        if (cnt < 2 || fq <= 0.0f || P <= 1.0f) continue;
        uint16_t rhi = 0, rlo = 0;
        Measure_RobustSpan(ADC_Buf, ADC_BUF_LEN, &rhi, &rlo);
        float sp = (float)(rhi - rlo);              /* 仅作为参考信息，不参与多帧一致性判定 */
        (void)sp;
        if (done == 0) { f_ref = fq; }
        else if (fabsf(fq - f_ref) > 0.05f * f_ref) { dirty = 1; break; }   /* 只检查频率：不同周期的混合才是突变的来源 */
        fold_frame(ADC_Buf, ADC_BUF_LEN, first, P);
        f_acc += fq; done++;
    }
    if (done == 0) { g_lock = NULL; r.disp_start = 0; r.disp_len = ADC_BUF_LEN; r.sig.valid = 0; return r; }
    if (dirty && wave_abort < 3) { wave_abort++; r.disp_start = 0; r.disp_len = WAVE_FOLD_PTS; r.sig.valid = 0; return r; }
    wave_abort = 0;

    /* 归一化得到单周期数据 period_buf[]。
       空相位桶不能用整帧均值填充：方波与矩形波的均值恰好位于两条平顶的正中，绘制后会出现从平顶
       垂向中位的孤立竖线，并同时抬高 max/min（进而抬高 Vpp）；权重过小的相位桶（仅落入一两个样本）
       情况相同，该样本本身可能就是毛刺或边沿的中间值。判据为：权重不小于平均权重 0.5 倍的相位桶才
       视为可靠；其余（空桶或欠采样桶）用左右最近的可靠相位桶做环形线性插值填补，使平顶保持水平、边沿保持陡峭。 */
    static uint16_t period_buf[WAVE_FOLD_PTS];
    float wtot = 0.0f;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) wtot += f_wt[p];
    float wok = 0.5f * wtot / (float)WAVE_FOLD_PTS;      /* 可靠相位桶的权重下限 */

    uint8_t good[WAVE_FOLD_PTS];
    int ngood = 0;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) {
        good[p] = (f_wt[p] >= wok && f_wt[p] > 0.0f) ? 1u : 0u;
        if (good[p]) { period_buf[p] = (uint16_t)(f_sum[p] / f_wt[p] + 0.5f); ngood++; }
    }
    if (ngood == 0) { r.disp_start = 0; r.disp_len = ADC_BUF_LEN; r.sig.valid = 0; return r; }

    for (int p = 0; p < WAVE_FOLD_PTS; p++) {            /* 不可靠相位桶：用左右最近的可靠相位桶做环形插值 */
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

    /* 峰峰值取"两端各截去 WAVE_TRIM_BINS 个相位桶"之后的带宽。边沿振铃只占少数相位桶，却会决定 max/min，
       且被采到的比例随每帧的采样相位而变，造成读数逐次抖动（实测存在过冲时，152mVpp 的极差为 94mV、
       2Vpp 的极差为 962mV），稳定判定因此无法成立，屏幕长期显示"测量中"。截峰后等价于取两条平顶电平：
       方波与脉冲的结果稳定且随输入单调。截去 4 个相位桶的依据：过冲在折叠周期中的宽度约为 2~3 个相位桶，
       只截去 2 桶仍落在过冲区间内（实测偏差在 +5% 与 +42% 之间波动）。代价随波形不同：正弦约 -2%、三角
       约 -7%（波形模式以形状与单调性为判据，不据此判定精度）。该处理只影响 Vpp 数值，绘制出的波形
       仍保留真实峰值与振铃。全变差（TV/ROF）、谐波重构与 σ 去偏等替代方法的精度收益有限，而计算开销
       明显（TV 需 200 次迭代并涉及排序），因此未采用。 */
    uint16_t top[WAVE_TRIM_BINS + 1], bot[WAVE_TRIM_BINS + 1];
    for (int j = 0; j <= WAVE_TRIM_BINS; j++) { top[j] = 0; bot[j] = 0xFFFF; }
    for (int p = 0; p < WAVE_FOLD_PTS; p++) {
        uint16_t v = period_buf[p];
        for (int j = 0; j <= WAVE_TRIM_BINS; j++)
            if (v > top[j]) { for (int m = WAVE_TRIM_BINS; m > j; m--) top[m] = top[m-1]; top[j] = v; break; }
        for (int j = 0; j <= WAVE_TRIM_BINS; j++)
            if (v < bot[j]) { for (int m = WAVE_TRIM_BINS; m > j; m--) bot[m] = bot[m-1]; bot[j] = v; break; }
    }
    uint16_t mx = top[WAVE_TRIM_BINS], mn = bot[WAVE_TRIM_BINS];
    r.sig.vpp   = Measure_RawToVolt(mx) - Measure_RawToVolt(mn);

    float fsq = 0.0f;
    for (int p = 0; p < WAVE_FOLD_PTS; p++) { float d = (float)period_buf[p] - fmean; fsq += d * d; }
    float frms = sqrtf(fsq / (float)WAVE_FOLD_PTS);
    r.sig.vdc   = Measure_MeanToVolt(fmean);
    r.sig.freq  = f_acc / (float)done;
    float hr = fmean + frms; if (hr > 4095.0f) hr = 4095.0f;
    r.sig.rms   = fabsf(Measure_MeanToVolt(hr) - r.sig.vdc);
    r.sig.valid = 1;

    /* 频段自检（与 Capture_AC 相同）：测得频率超出当前量程档时解除锁定，下次测量执行完整 auto_range 换挡 */
    { const AdRange *need = pick_range(r.sig.freq, 999u); if (need != g_lock) g_lock = NULL; }

    /* 显示：将单周期数据按频率平铺为 C 个周期（固定时间窗），使疏密随频率变化，避免呈现与频率无关的静态波形。
       分数周期配合线性插值取相位桶：C 不再四舍五入为整数，因此疏密随频率连续单调变化（不发生逐格跳变），
       相邻列也不会重复或跳过相位桶（截断取整会使每个周期的边缘错位半格，呈现毛刺与不对齐）。 */
    float Cf = WAVE_C1K * powf(r.sig.freq / 1000.0f, WAVE_ALPHA);  /* 幂律压缩；WAVE_ALPHA=1 时退化为线性时基 */
    if (Cf < 1.0f) Cf = 1.0f;                                    /* 低频时至少显示 1 个周期（赛题第三项要求） */
    if (Cf > (float)WAVE_MAX_CYC) Cf = (float)WAVE_MAX_CYC;
    Capture_WaveEnv = (Cf >= (float)WAVE_ENV_BIN) ? 1u : 0u;     /* 疏密达到每周期不超过 8px 时改为包络模式 */
    Capture_WaveCyc = Cf;
    for (int j = 0; j < WAVE_FOLD_PTS; j++) {
        float pos = (float)j * Cf / (float)WAVE_FOLD_PTS;         /* 单位为周期，允许取小数 */
        pos -= floorf(pos);                                       /* 折算到 0..1 相位 */
        float xf = pos * (float)WAVE_FOLD_PTS;                    /* 0..128 范围内的浮点相位桶位置 */
        int   b  = (int)xf;  if (b >= WAVE_FOLD_PTS) b = WAVE_FOLD_PTS - 1;
        int   b1 = (b + 1) % WAVE_FOLD_PTS;
        float fr = xf - (float)b;
        Capture_WaveBuf[j] = (uint16_t)((float)period_buf[b] * (1.0f - fr)
                                      + (float)period_buf[b1] * fr + 0.5f);
        /* 本列覆盖 cnt 个相位桶：取其 min/max 作为上下包络。逐点抽稀在每周期不超过 8px 时会直接
           漏掉峰谷（幅度偏小、方波边沿变斜），而包络能保证峰与谷都显示在屏上。 */
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
    Capture_WaveLen = WAVE_FOLD_PTS;              /* 应用层按此长度绘制疏密随频率变化的波形 */
    r.disp_start = 0; r.disp_len = WAVE_FOLD_PTS;
    return r;
}

/* ================= 波形坐标系：码值 → 行号 的映射 =================
   该函数为纯计算，不访问显示寄存器，因此在 PC 端单元测试中可用同一函数验算几何关系（伸缩、
   平移与裁剪）。放在本层的原因是它属于采集数据的单位换算：折叠缓冲存放的是包含直流偏置的 ADC 码，
   要换算成按 Vpp 伸缩、按偏置平移的屏上位置，必须经过标定链，而 Capture 是唯一同时持有码值与
   Measure 换算能力的层次。像素几何只体现在 WAVE_BOX_* 几个常量中。 */

static const uint16_t wdiv_mv[] = { 5, 10, 20, 50, 100, 200, 500, 1000, 2000 };
static char *const wdiv_txt[]   = { "5mV", "10mV", "20mV", "50mV", "100mV", "200mV", "500mV", "1V", "2V" };
static char wdiv_none[]         = "--";
#define WDIV_N ((uint8_t)(sizeof(wdiv_mv) / sizeof(wdiv_mv[0])))

static int16_t row_of(float mv, int16_t y0v, float pxpmv)
{
    float yy = (float)y0v - mv * pxpmv;
    return (int16_t)(yy >= 0.0f ? yy + 0.5f : yy - 0.5f);
}
static int16_t clip_row(int16_t y)
{
    const int16_t a = (int16_t)(WAVE_BOX_Y + 1), b = (int16_t)(WAVE_BOX_Y + WAVE_BOX_H - 2);
    if (y < a) return a;
    if (y > b) return b;
    return y;
}

/* 填写 ylo/yhi（每列下沿与上沿的行号）并返回列数；同时输出 V/div 档位号与 0V 行号，供应用层标注。 */
uint16_t Capture_WaveRows(uint8_t *ylo, uint8_t *yhi, uint16_t maxn, uint8_t *out_kdiv, int16_t *out_y0v)
{
    uint16_t n = Capture_WaveLen, c_lo, c_hi, i;
    if (n == 0) return 0;
    if (n > maxn) n = maxn;

    /* 1) 本屏需要显示的码值上下限：包络模式取每列的 lo/hi，折线模式取单点 */
    c_lo = 0xFFFF; c_hi = 0;
    for (i = 0; i < n; i++) {
        uint16_t lo = Capture_WaveEnv ? Capture_WaveLo[i] : Capture_WaveBuf[i];
        uint16_t hi = Capture_WaveEnv ? Capture_WaveHi[i] : Capture_WaveBuf[i];
        if (lo < c_lo) c_lo = lo;
        if (hi > c_hi) c_hi = hi;
    }
    if (c_hi <= c_lo) c_hi = (uint16_t)(c_lo + 4);          /* 波形为水平线等退化情形的保护 */

    /* 2) 码值 → 真实输入电压（mV）：只在两个端点执行完整标定链，中间用线性插值。
          插值偏差远小于 1 格，端点准确，因此屏幕上按坐标格计数乘以 V/div 的结果与第 1 行显示的 Vpp 数值一致。 */
    float v_lo = Measure_MeanToVolt((float)c_lo) * 1000.0f;
    float v_hi = Measure_MeanToVolt((float)c_hi) * 1000.0f;
    if (v_hi < v_lo) { float t = v_hi; v_hi = v_lo; v_lo = t; }
    float ck = (v_hi - v_lo) / (float)(c_hi - c_lo);        /* 单位：mV/码 */

    /* 3) V/div 分档选择（带回差）。显示容量为 WAVE_DIV_V × vdiv，即整个波形框能够显示的电压范围：
          · 升档是几何硬限制——只有装不下才升档，并保留 2%（约 1px）的溢出余量，避免临界抖动引起分档切换；
          · 降档要求幅度明显变小（在低一档中占用格数不超过 70%）才切换到下一低档。
          若没有回差，Vpp=4V 恰好等于 8×500mV，测量值抖动 ±几 mV 就会在 500mV/div 与 1V/div 之间来回切换。 */
    float span = v_hi - v_lo;
    uint8_t kfit = 0;
    while (kfit + 1 < WDIV_N && span > (float)WAVE_DIV_V * (float)wdiv_mv[kfit]) kfit++;
    uint8_t k = (wdiv_hold < WDIV_N) ? wdiv_hold : kfit;
    if (span > (float)WAVE_DIV_V * (float)wdiv_mv[k] * WDIV_OVER) k = kfit;                  /* 装不下时直接切换到能够容纳的档位 */
    else {                                                                                   /* 幅度明显变小时一次切换到留有余量的最低档 */
        uint8_t j = k;
        while (j > 0 && span <= WDIV_BACK * (float)WAVE_DIV_V * (float)wdiv_mv[j - 1]) j--;
        k = j;
    }
    wdiv_hold = k;
    float pxpmv = (float)WAVE_PXDIV / (float)wdiv_mv[k];

    /* 4) 0V 默认固定在正中格线上；只有波形会被裁剪时才整体平移，平移量取刚好不裁剪的值。
          这样小幅偏置变化表现为迹线相对 0V 线上下移动；偏置大到 0V 超出波形框时由边缘箭头指示。 */
    /* 描线区域比波形框内缩 1px：否则在刚好不裁剪时平顶会与框线重叠，方波上沿被框线遮挡 */
    const int16_t yA = (int16_t)(WAVE_BOX_Y + 1), yB = (int16_t)(WAVE_BOX_Y + WAVE_BOX_H - 2);
    int16_t y0v = (int16_t)(WAVE_BOX_Y + WAVE_BOX_H / 2);      /* 默认 0V 落在正中格线，上下各 4 格 */
    int16_t yt = row_of(v_hi, y0v, pxpmv), yb = row_of(v_lo, y0v, pxpmv);
    if (yt < yA)      y0v = (int16_t)(y0v + (yA - yt));
    else if (yb > yB) y0v = (int16_t)(y0v - (yb - yB));

    /* 5) 逐列电压换算为行号 */
    for (i = 0; i < n; i++) {
        uint16_t lo = Capture_WaveEnv ? Capture_WaveLo[i] : Capture_WaveBuf[i];
        uint16_t hi = Capture_WaveEnv ? Capture_WaveHi[i] : Capture_WaveBuf[i];
        yhi[i] = (uint8_t)clip_row(row_of(v_lo + ((float)hi - (float)c_lo) * ck, y0v, pxpmv));
        ylo[i] = (uint8_t)clip_row(row_of(v_lo + ((float)lo - (float)c_lo) * ck, y0v, pxpmv));
    }
    if (out_kdiv) *out_kdiv = k;
    if (out_y0v)  *out_y0v  = y0v;
    return n;
}

uint16_t Capture_WaveDivMv(uint8_t kdiv)   { return (kdiv < WDIV_N) ? wdiv_mv[kdiv] : 0; }
char    *Capture_WaveDivText(uint8_t kdiv) { return (kdiv < WDIV_N) ? wdiv_txt[kdiv] : wdiv_none; }
