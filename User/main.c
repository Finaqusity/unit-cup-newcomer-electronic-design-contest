#include "stm32f10x.h"                  // Device header
#include "stm32f10x_flash.h"            // FLASH_Unlock/ErasePage/ProgramHalfWord/Lock：标定锚点掉电持久化
#include "Timer.h"                      // 1ms 系统时基(TIM2)与 Millis()
#include "OLED.h"
#include "CN_Str.h"                     // 中文取模字串宏(CN_xxx)，供 OLED_ShowChinese 使用
#include "Key.h"
#include "ADC.h"                        // ADC_BUF_LEN：单帧样本数，用于初始化显示缓冲长度
#include "Capture.h"                    // 采集调度层：自动量程、过渡帧剔除、首次显示数据门控
#include "DcEstim.h"                    // 直流采样估计器，具体算法由 DC_ESTIM 宏选定
            
/* ==================== 周期信号测量装置 · 主程序 ====================

   架构（依赖方向单向：main → Capture → {ADC, Measure}）：
     - main 只负责分发按键事件、按需触发测量、渲染显示，不含采集与算法实现。
     - 主循环顺序：读取按键事件 → 按需测量 → 限帧刷屏（只绘制缓存结果）。
     - 测量节奏与刷屏节奏解耦：单次测量忙等约 0.1~0.2s，只在切换模式、
       按键动作或到达轮询间隔时执行；刷屏仅读取缓存，不触发采集。
     - 轮询间隔依据显示是否稳定自适应切换（见 MEASURE_MS 与 FAST_MS）。
     - 切换模式时立即把缓存标记为无效并重新测量，保证显示的第一帧数据来自新信号。

   显示模式（KEY1 短按循环切换：DC → AC → 波形 → DC）：
     - DC   ：显示直流分量 Vdc（mV）
     - AC   ：显示 Vpp、频率、RMS
     - WAVE ：绘制波形，并同时显示 Vpp 与频率

   按键：KEY2 短按在 DC 模式切换显示位数，其余模式为手动重测；
         KEY1 长按复位到 DC 模式（CAL 模式下为清空锚点）。

   屏幕接口由 OLED.c 顶部宏 OLED_USE_ONBOARD 选择：
   0 = 外接屏（PB10/PB11），1 = 板载屏（PB8/PB9）。
   ================================================================= */

#define MODE_DC   0
#define MODE_AC   1
#define MODE_WAVE 2
#define MODE_CNT  3

#define REDRAW_MS   80u     // 刷屏节流周期：按此间隔重绘缓存，限制 OLED 刷新次数
#define MEASURE_MS  700u    // 稳态自动重测间隔：读数保持更新，又不占用过多测量时间
#define FAST_MS     120u    // 显示未稳定或标定时的加速轮询间隔，用于尽快恢复有效读数
#define DC_TARE_MAX_MV 50   // DC 零点标定门限：|当前 Vdc| 小于此值才允许归零；取值需大于零点漂移量级、小于最小待测真实信号

static uint8_t       mode        = MODE_DC;
static uint8_t       needMeasure = 1;      // 上电后先执行一次测量
static CaptureResult cr        = { .sig = {0}, .fs_used = 0, .disp_start = 0, .disp_len = ADC_BUF_LEN };
static uint32_t      lastMeasure = 0;
static uint32_t      lastDraw    = 0;

/* 直流零点标定（TARE）：零点偏置取决于每次上电后的热状态，仅保存在 RAM，掉电后失效 */
static float         dc_offset    = 0.0f;   /* 当前零点偏置(V)，显示时从 Vdc 中扣除 */
static char         *tareMsg      = "";     /* 按键反馈文字 */
static uint32_t      tareMsgUntil = 0;      /* 反馈显示的截止时刻(ms) */

/* 重标定（CAL）模式：KEY2 长按进入/退出；进入后 KEY2 短按在整伏输入点采集一个锚点，
   目标值就近吸附到栅格；KEY1 长按清空全部锚点。
   约束条件：标定范围之外按线性外推（不设上限截断）、就近吸附须落在栅格容差内、
   单调性检查拒绝倒序锚点。 */
static uint8_t       calMode      = 0;
static char         *calMsg       = "";
static uint32_t      calMsgUntil  = 0;
static uint8_t       calDirty     = 0;      /* 本次进入 CAL 后锚点是否被改动(采锚或清空)：仅在退出时据此决定是否写 Flash，以减少擦写 */

static uint8_t       dispStable   = 1;      /* 最近若干次测量是否一致；否则判定输入正在变化，显示置无效 */

static uint8_t       wave_have    = 0;      /* 是否已成功折叠出至少一帧波形；为真时波形区持续显示上一帧有效波形 */

static uint8_t       dcDecimal    = 0;      /* DC 显示格式：0 = 整数 mV，1 = 一位小数(0.1mV)，由 KEY2 短按切换 */

/* ==================== 标定锚点的 Flash 掉电持久化 ====================
   器件 STM32F103C8T6：64KB Flash，中密度，页大小 1KB，共 64 页。
   地址选择：本固件代码占用约 20.6KB（结束于 0x08005288 附近），末页 0x0800FC00
   未被代码使用，单独存放标定数据可保证擦写只影响该页、不触碰固件；
   更换存放地址前应先查 .map 文件确认目标页仍未被代码占用。
   读取：开机调用 CalFlash_Load()，依次校验 magic、版本号、锚点数取值范围和校验和，
   任一项不符即判定无有效记录（空白 Flash 读回全 0xFF），此时直接使用固件内置基表；
   校验通过才把锚点恢复进 Measure 模块的 RAM 表。
   写入：仅在退出 CAL 且本次锚点确有改动（calDirty）时执行一次 CalFlash_Save()，
   擦除整页后按半字写入当前锚点。限制写入次数有两方面原因：Flash 擦写寿命约 1 万次；
   且擦除与编程期间从 Flash 取指会停顿 CPU（页擦除为数十毫秒量级），
   期间测量与刷屏节奏都会延后。
   存储内容只保留 (实测 Vadc, 目标 Vin) 两个数组：基表编在固件内不需存储，
   表前修正量由 Measure 内部的 g0_vadc 反算得到。
   擦除粒度为 1 页，编程粒度为半字(16bit)。
   ================================================================== */
#define CALFLASH_ADDR   0x0800FC00u         /* 末页起始地址(64KB 器件) */
#define CALFLASH_MAGIC  0xCA1BCE01u
#define CALFLASH_VER    1u
#define CALFLASH_CAP    24u                  /* 与 CAL_ANCH_MAX 一致 */

typedef struct {
    uint32_t magic;
    uint32_t ver;
    uint8_t  count;                           /* 有效锚点数 0..CAP */
    uint8_t  pad0[3];
    uint16_t chk;                             /* 数据区校验和，由 calflash_calc 计算 */
    uint16_t pad1;
    float    u[CALFLASH_CAP];                 /* 实测 Vadc(mV) */
    float    tgt[CALFLASH_CAP];               /* 目标 Vin(mV) */
} CalFlash_t;

/* 校验和：对 magic/ver/count 以及前 count 个 u[] 元素与前 count 个 tgt[] 元素的
   32 位字求和，再折叠为 16 位。u[] 与 tgt[] 是两个各自 24 元素的独立数组（非交错存放），
   需分别按索引累加。 */
static uint16_t calflash_calc(const CalFlash_t *p)
{
    uint32_t s = p->magic + p->ver + p->count;
    const uint32_t *u = (const uint32_t *)&p->u[0];
    const uint32_t *t = (const uint32_t *)&p->tgt[0];
    for (uint8_t i = 0; i < p->count; i++) { s += u[i]; s += t[i]; }
    return (uint16_t)((s & 0xFFFFu) ^ (s >> 16));
}

/* 写入：把 Measure 当前生效的锚点序列化后存入末页。 */
static void CalFlash_Save(void)
{
    CalFlash_t d;
    d.magic = CALFLASH_MAGIC;
    d.ver   = CALFLASH_VER;
    uint8_t n = Measure_CalCount(); if (n > CALFLASH_CAP) n = CALFLASH_CAP;
    d.count = n;
    d.pad0[0] = d.pad0[1] = d.pad0[2] = 0;
    d.pad1 = 0;
    for (uint8_t i = 0; i < n; i++) Measure_CalGet(i, &d.u[i], &d.tgt[i]);
    for (uint8_t i = n; i < CALFLASH_CAP; i++) { d.u[i] = 0.0f; d.tgt[i] = 0.0f; }
    d.chk = calflash_calc(&d);

    FLASH_Unlock();
    FLASH_ErasePage(CALFLASH_ADDR);
    const uint16_t *w = (const uint16_t *)&d;
    uint32_t addr = CALFLASH_ADDR;
    for (uint32_t i = 0; i < sizeof(CalFlash_t) / 2u; i++) {
        if (FLASH_ProgramHalfWord(addr, w[i]) != FLASH_COMPLETE) break;
        addr += 2u;
    }
    FLASH_Lock();
}

/* 读取：开机调用；magic/版本/计数范围/校验和全部通过才恢复锚点，
   并返回成功恢复的锚点数；返回 0 表示无有效记录或数据损坏，使用内置基表。 */
static uint8_t CalFlash_Load(void)
{
    const CalFlash_t *p = (const CalFlash_t *)CALFLASH_ADDR;
    if (p->magic != CALFLASH_MAGIC || p->ver != CALFLASH_VER) return 0;
    if (p->count == 0u || p->count > CALFLASH_CAP) return 0;
    if (calflash_calc(p) != p->chk) return 0;
    return Measure_CalRestore(p->u, p->tgt, p->count);
}

/* 电压(V) 换算为毫伏整数并四舍五入；返回值为无符号类型，仅适用于非负输入 */
static uint32_t To_mV(float v)
{
    int32_t x = (int32_t)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    return (uint32_t)x;
}

/* 显示稳定门：对最近 3 次有效测量取中位数，剔除输入调整过程中的单帧离群值 */
#define HIST_N 3
static float h_vdc[HIST_N], h_vpp[HIST_N], h_freq[HIST_N], h_rms[HIST_N];
static uint8_t hCnt = 0;
static Signal_t shown = {0};

static void hist_push(float v, float *h) { h[2] = h[1]; h[1] = h[0]; h[0] = v; }
static float hist_med(const float *h, uint8_t n)
{
    float t[HIST_N]; for (uint8_t i = 0; i < n; i++) t[i] = h[i];
    for (uint8_t i = 1; i < n; i++) { float k = t[i]; int j = i - 1; while (j >= 0 && t[j] > k) { t[j+1] = t[j]; j--; } t[j+1] = k; }
    return t[n / 2];
}
static float hist_spread(const float *h, uint8_t n)   /* 最近 n 次的极差 */
{
    float lo = h[0], hi = h[0];
    for (uint8_t i = 1; i < n; i++) { if (h[i] < lo) lo = h[i]; if (h[i] > hi) hi = h[i]; }
    return hi - lo;
}

/* 直流显示选值策略：最近两次测量已足够接近时视为新电平已稳定，直接取最新值，
   以消除中位数因历史加权产生的滞后，使真实阶跃无需等待多帧即可显示；
   两次差值较大时退回中位数，保留剔除单帧毛刺的能力。 */
static float hist_pick(const float *h, uint8_t n, float tol)
{
    if (n >= 2) {
        float d = h[0] - h[1]; if (d < 0) d = -d;
        if (d < tol) return h[0];
    }
    return hist_med(h, n);
}

/* 执行一次采集与处理，结果写入 cr，并更新经稳定处理后的显示值 shown */
static void DoMeasure(void)
{
    uint8_t fast = (uint8_t)(!dispStable);   // 显示未稳定（输入变化或刚切换模式）时，AC/WAVE 采用减少帧数的快速档位以提高响应速度；已稳定时使用全帧以保证精度
    if (calMode)                cr = Capture_DC();          // 标定模式：按直流方式取帧
    else if (mode == MODE_DC)   cr = Capture_DC();          // 直流：均值换算
    else if (mode == MODE_WAVE) cr = Capture_Wave(fast);    // 任意波：相干折叠 + 指标
    else                        cr = Capture_AC(fast);      // 交流：Vpp/F/Rms

    if (cr.sig.valid) {
        hist_push(cr.sig.vdc,  h_vdc);
        hist_push(cr.sig.vpp,  h_vpp);
        hist_push(cr.sig.freq, h_freq);
        hist_push(cr.sig.rms,  h_rms);
        if (hCnt < HIST_N) hCnt++;
        /* 显示路由：MODE_DC 使用 DcEstim 时域估计器（兼顾稳定与跟随）；
           CAL 模式直接采用 16 帧平均值，不经过历史窗口，读数当场稳定，便于判断电压；
           AC 与波形模式的 vdc 不显示，沿用 hist_pick。 */
        if (mode == MODE_DC)       shown.vdc = DcEstim_Feed(cr.sig.vdc);
        else if (calMode)          shown.vdc = cr.sig.vdc;
        else                       shown.vdc = hist_pick(h_vdc, hCnt, 0.015f);
        /* AC/波形：vpp/freq/rms 使用 hist_pick 而非固定中位数——连续两次差值在测量噪声内
           即取最新值，使新电平立即生效；仅当两者差值明显偏大（疑似异常帧）时退回中位数剔除。
           相比固定中位数少受一个历史值影响，输入变化后约提前一次刷新周期稳定。
           容差取在噪声底之上、孤立异常帧偏差之下，并按幅值比例放宽。 */
        shown.vpp  = hist_pick(h_vpp,  hCnt, 0.010f + 0.02f * (h_vpp[0]  < 0 ? -h_vpp[0]  : h_vpp[0]));
        shown.freq = hist_pick(h_freq, hCnt, 5.0f   + 0.01f * (h_freq[0] < 0 ? -h_freq[0] : h_freq[0]));
        shown.rms  = hist_pick(h_rms,  hCnt, 0.010f + 0.02f * (h_rms[0]  < 0 ? -h_rms[0]  : h_rms[0]));
        shown.valid = 1;
        if (mode == MODE_WAVE) wave_have = 1;      /* 折叠成功（未因中途改变输入而作废），显示缓冲中已是一帧有效波形 */

        /* 稳定性判据：最近若干次测量彼此偏差过大，判定输入正在变化，显示置无效。
           AC 模式只考察最近 2 次，使输入变化后能提前一个测量周期结束无效提示；
           DC 与波形模式维持完整 3 次窗口——波形模式依赖多帧折叠，放宽判据可能显示
           尚未平均干净的波形。 */
        if (hCnt >= 2) {
            uint8_t w = (mode == MODE_AC) ? 2u : hCnt;
            float ad = shown.vdc < 0 ? -shown.vdc : shown.vdc;
            dispStable = (hist_spread(h_vdc,  w) < 0.04f + 0.03f * ad)
                      && (hist_spread(h_vpp,  w) < 0.04f + 0.03f * shown.vpp)
                      && (hist_spread(h_freq, w) < 20.0f + 0.02f * shown.freq);
        } else {
            dispStable = 1;
        }
    } else {
        shown.valid = 0;
        dispStable  = 0;      /* 本次无有效帧或折叠作废：下一轮按 FAST_MS 立即重测，无需等待 MEASURE_MS */
    }
    lastMeasure = Millis();
    needMeasure = 0;
}

/* 请求重测：置测量标志并清空缓存与历史窗口，保证切换后首次显示不受旧数据影响 */
static void MarkDirty(void)
{
    needMeasure   = 1;
    cr.sig.valid  = 0;
    shown.valid   = 0;
    hCnt          = 0;
    DcEstim_Reset();                     /* 切换模式与进入/退出 CAL：清空估计器历史，使首次测量即时且不被旧数据污染 */
    Capture_Unlock();                    /* 切换模式与进入/退出 CAL：信号类型可能改变，下次测量重新执行自动量程 */
}

/* ==================== 各模式渲染（只读缓存，不触发采集） ====================
   排版约定：屏幕每行 16 列。数值字段从固定列开始，按该字段的最大位数占位，
   单位标签放在数字最高位之后的列，两者占用的列区间不重叠，否则后写入的内容
   会覆盖先写入的字符。数值无效或尚未稳定时，整个数值字段显示测量中提示。
   ======================================================================== */
static void Render_DC(void)
{
    OLED_ShowString(1, 1, "DC Vdc=");                            /* 前缀占第 1~7 列 */
    if (shown.valid && dispStable) {
        float val = shown.vdc - dc_offset;                        /* 电压(V)，扣除零点偏置 */
        if (dcDecimal) {                                          /* 一位小数格式(0.1mV) */
            float    tv  = val * 10000.0f;                        /* 换算到 0.1mV 单位 */
            int32_t  tm  = (int32_t)(tv + (tv >= 0.0f ? 0.5f : -0.5f));
            uint8_t  neg = (tm < 0);
            uint32_t a   = neg ? (uint32_t)(-tm) : (uint32_t)tm;
            OLED_ShowChar (1, 8,  neg ? '-' : '+');
            OLED_ShowNum  (1, 9,  a / 10u, 4);                    /* 整数部分 mV，4 位可覆盖 0~5V */
            OLED_ShowChar (1, 13, '.');
            OLED_ShowNum  (1, 14, a % 10u, 1);                    /* 小数第 1 位(0.1mV) */
            OLED_ShowString(1, 15, "mV");
        } else {                                                  /* 整数 mV 格式：符号 1 列 + 5 位数字 */
            OLED_ShowSignedNum(1, 8, (int32_t)To_mV(val), 5);
            OLED_ShowString(1, 14, "mV");
        }
    } else {
        OLED_ShowString(1, 8, "Measuring");                        /* 无效或输入未稳定：整个数值字段显示测量中提示 */
    }

    if (Millis() < tareMsgUntil) {                              /* 零点标定反馈，超时后恢复采样率显示 */
        OLED_ShowString(3, 1, "TARE ");
        OLED_ShowString(3, 6, tareMsg);
    } else {
        OLED_ShowChinese(0, 32, CN_SAMPLE);                     // “采样”：当前采样率档位
        OLED_ShowNum(3, 5, cr.fs_used / 1000u, 3);
        OLED_ShowString(3, 9, "kHz");
    }

    /* 第 4 行显示当前直流估计器的算法短名（由 DcEstim_Name() 给出，随 DC_ESTIM 配置变化），
       用于核对实际生效的去抖策略。 */
    OLED_ShowString(4, 1, "est:");
    OLED_ShowString(4, 5, DcEstim_Name());
}

static void Render_AC(void)
{
    /* 数值未稳定时整个数值字段显示测量中提示，此时测量与判稳流程照常运行 */
    uint8_t ok = shown.valid && dispStable;
    OLED_ShowString(2, 1, "Vpp=");
    if (ok) { OLED_ShowNum(2, 6, To_mV(shown.vpp), 4); OLED_ShowString(2, 11, "mV"); }
    else    { OLED_ShowString(2, 6, "Measuring"); }

    OLED_ShowString(3, 1, "F  =");
    if (ok) { OLED_ShowNum(3, 6, (uint32_t)shown.freq, 5); OLED_ShowString(3, 12, "Hz"); }
    else    { OLED_ShowString(3, 6, "Measuring"); }

    OLED_ShowString(4, 1, "Rms=");
    if (ok) { OLED_ShowNum(4, 6, To_mV(shown.rms), 4); OLED_ShowString(4, 11, "mV"); }
    else    { OLED_ShowString(4, 6, "Measuring"); }
}

static void Render_Wave(void)
{
    /* 顶部两行为 Vpp 与频率，与波形同屏显示；数值未稳定时整个数值字段显示测量中提示 */
    uint8_t ok = shown.valid && dispStable;
    OLED_ShowString(1, 1, "Vpp=");
    if (ok) { OLED_ShowNum(1, 6, To_mV(shown.vpp), 4); OLED_ShowString(1, 11, "mV"); }
    else    { OLED_ShowString(1, 6, "Measuring"); }

    OLED_ShowString(2, 1, "F  =");
    if (ok) { OLED_ShowNum(2, 6, (uint32_t)shown.freq, 5); OLED_ShowString(2, 12, "Hz"); }
    else    { OLED_ShowString(2, 6, "Measuring"); }

    /* 下部为波形区，不受稳定门限制。Capture_Wave 会在折叠中途输入被改变时作废该次折叠、
       不写显示缓冲，因此这里绘制的始终是上一次成功折叠的波形：改变幅度或频率时画面短暂停留，
       随后更新为新形状，既不会画出多帧叠加的失真波形，也不会在波形与提示文字之间反复切换。 */
    OLED_DrawFrame(0, 33, 128, 31, 1);
    if (wave_have) {
        if (Capture_WaveEnv) OLED_DrawWaveEnv(Capture_WaveLo, Capture_WaveHi, Capture_WaveLen, 1, 34, 126, 29);
        else                 OLED_DrawWave (Capture_WaveBuf, Capture_WaveLen, 1, 34, 126, 29);
    } else {
        OLED_ShowString(4, 3, "no signal");        /* 仅在从未成功折叠出波形时显示 */
    }
}

/* CAL 界面：第 1 行锚点数量，第 2 行当前直流读数，第 3~4 行操作提示与上一次采锚结果 */
static void Render_CAL(void)
{
    OLED_ShowString(1, 1, "CAL  n=");
    OLED_ShowNum(1, 8, Measure_CalCount(), 2);

    OLED_ShowString(2, 1, "Vdc=");
    if (shown.valid) OLED_ShowSignedNum(2, 6, (int32_t)To_mV(shown.vdc), 6);
    else             OLED_ShowString(2, 6, "  ----");
    OLED_ShowString(2, 13, "mV");

    OLED_ShowString(3, 1, "set V, K2=add");

    if (Millis() < calMsgUntil) {
        OLED_ShowString(4, 1, "last: ");
        OLED_ShowString(4, 7, calMsg);
    } else {
        OLED_ShowString(4, 1, "K2long=exit");
    }
}

static void Render(void)
{
    OLED_Clear();
    if      (calMode)         Render_CAL();
    else if (mode == MODE_DC)   Render_DC();
    else if (mode == MODE_AC)   Render_AC();
    else                        Render_Wave();
    OLED_Flush();
}

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    OLED_Init();
    Timer_Init();       // 先启动 1ms 时基（Key 扫描与 Millis 依赖它）
    Key_Init();         // 配置按键 GPIO(PB3/PB4)，并把 Key_Scan 挂到 TIM2 回调
    Capture_Init();     // 初始化 ADC(TIM3+DMA)，默认中间量程档
    CalFlash_Load();    // 上电恢复上次写入 Flash 的标定锚点；无有效记录时使用内置基表

    while (1)
    {
        /* 1) 读取按键事件（消抖与长按判定在 Key 模块的定时扫描中完成，此处非阻塞取标志） */
        if (Key_Click(1) && !calMode) { mode = (uint8_t)((mode + 1) % MODE_CNT); MarkDirty(); }

        if (Key_LongPress(2)) {
            calMode = (uint8_t)(!calMode);
            if (calMode) {
                dc_offset = 0.0f;                             /* 进入 CAL：清除 TARE 偏置，避免与锚点修正叠加 */
                calDirty  = 0;                                /* 进入 CAL：复位改动标志，退出时据此决定是否写 Flash */
            } else if (calDirty) {
                CalFlash_Save(); calDirty = 0;                /* 退出 CAL 且本次锚点有改动才写入末页持久化，否则不产生擦写 */
            }
            MarkDirty();
        }                                                   // 进入/退出重标定

        if (Key_Click(2)) {
            if (calMode) {
                DoMeasure();                                   /* 取最新直流帧：cr.raw_mean 为 16 帧平均后的原始码值 */
                float mcode = cr.raw_mean;                     /* 与显示使用同一估计，避免锚点落在单帧上造成偏差 */
                float vadc_mv = mcode * 3300.0f / 4095.0f;     /* 设备端 Vadc(mV)，即 PA0 实测电压 */
                float v = Measure_MeanToVolt(mcode);           /* 修正后读数；标定范围外线性外推，不设上限截断 */
                int32_t K = (int32_t)(v * 2.0f + (v >= 0 ? 0.5f : -0.5f));   /* 就近吸附到 0.5V 栅格 */
                float Kv = (float)K * 0.5f;
                /* 收锚条件为读数有效且显示已稳定，不再设置“与栅格距离”门限：0.5V 栅格下任意读数
                   与最近格点的距离不超过 0.25V，而高段系统误差正是标定要修正的对象，距离门限会把
                   这类误差排除在外。可靠性由显示稳定判据与 Measure_AddCal 的单调性检查共同保证。 */
                if (shown.valid && dispStable) {
                    if (Measure_AddCal(vadc_mv, Kv * 1000.0f, 500.0f)) {
                        calMsg = "ADD ";
                        calDirty = 1;                            /* 锚点已改动：退出 CAL 时写入 Flash */
                        shown.vdc = Measure_MeanToVolt(mcode); /* 立即回算，本次按键后即可看到修正后的 Vdc */
                    } else {
                        calMsg = "MONO";                       /* 目标值与已有锚点顺序矛盾，该锚点被拒绝 */
                    }
                } else {
                    calMsg = "WAIT";                           /* 读数未稳定或无信号，等待稳定后再采锚 */
                }
                calMsgUntil = Millis() + 700u;
            } else if (mode == MODE_DC) {
                dcDecimal = (uint8_t)(!dcDecimal);               /* DC 模式短按：在整数 mV 与一位小数(0.1mV)之间切换 */
            } else {
                MarkDirty();                                   /* 其余模式：保留手动重测功能 */
            }
        }
        if (Key_LongPress(1)) {                                // CAL 模式下清空锚点；其他模式复位到 DC
            if (calMode) {
                Measure_ClearCal();
                CalFlash_Save();                 /* 立即把空表写回 Flash，避免复位或断电后 CalFlash_Load 重新载入旧锚点 */
                calDirty = 0;                    /* 已落盘，退出 CAL 时无需再次写入 */
                calMsg = "CLEAR"; calMsgUntil = Millis() + 700u;
            }
            else { mode = MODE_DC; MarkDirty(); }
        }

        /* 2) 按需测量：显示未稳定、估计器仍在确认候选台阶，或处于 CAL 模式（需要即时读数）时使用
           FAST_MS 加速轮询，其余情况按 MEASURE_MS 的常规间隔自动重测 */
        uint32_t now = Millis();
        uint32_t iv  = (dispStable && !calMode && !DcEstim_Pending()) ? MEASURE_MS : FAST_MS;
        if (needMeasure || now - lastMeasure >= iv) DoMeasure();

        /* 3) 限帧刷屏：按 REDRAW_MS 间隔重绘，只绘制缓存结果 */
        if (now - lastDraw >= REDRAW_MS) { lastDraw = Millis(); Render(); }
    }
}
