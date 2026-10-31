#include "stm32f10x.h"                  // Device header
#include "stm32f10x_flash.h"            // FLASH_Unlock/ErasePage/ProgramHalfWord/Lock：用于标定锚点的掉电持久化
#include "Timer.h"                      // 1ms 系统时基（TIM2）与 Millis()
#include "OLED.h"
#include "CN_Str.h"                     // 中文转义宏（CN_xxx），供 OLED_ShowChinese 使用
#include "Key.h"
#include "ADC.h"                        // ADC_Buf：波形模式将采集缓冲直接交给 OLED_DrawWave 绘制
#include "Capture.h"                    // 采集调度层：自动量程管理与首次数据有效门控
#include "DcEstim.h"                    // 直流采样估计器：算法由 DC_ESTIM 宏选定
            
/* ============================================================
   周期信号测量装置 · 主程序
   ------------------------------------------------------------
   架构（依赖关系单向：main → Capture → {ADC, Measure}）：
     · main 仅负责结果分发、界面渲染与按键事件读取，不含采集与算法逻辑
     · 测量节奏与刷新节奏相互独立：DoMeasure 为忙等流程，耗时约 0.1~0.2s，
       故仅在切换模式、按键操作或定时间隔到达时触发；界面刷新只绘制缓存内容
     · 切换模式时立即把缓存标记为无效并重新测量，以保证首次显示即为本次的有效结果

   模式（KEY1 短按循环切换：DC → AC → 波形 → DC）：
     · DC   : 显示 Vdc（mV）
     · AC   : 显示 Vpp、频率、RMS
     · WAVE : 绘制波形，并同时显示 Vpp 与频率
   KEY2 短按：手动重测当前模式    KEY1 长按：复位至 DC 模式

   显示屏接口由 OLED.c 顶部的 OLED_USE_ONBOARD 选择（0 为外接屏 PB10/11，1 为板载屏 PB8/9）。
   ============================================================ */

#define MODE_DC   0
#define MODE_AC   1
#define MODE_WAVE 2
#define MODE_CNT  3

#define REDRAW_MS   80u     // 刷新节流：每 80ms 重绘一次缓存内容
#define MEASURE_MS  700u    // 自动重测间隔：使读数持续更新，同时避免测量过于频繁引起卡顿
#define FAST_MS     120u    // 判定为输入正在切换时采用的快轮询间隔，以尽快恢复有效读数
#define DC_TARE_MAX_MV 50   // 直流零点标定门限：仅当 |当前 Vdc| 小于此值才允许归零（大于最大零点漂移约 10mV，且小于最小真实信号）

static uint8_t       mode        = MODE_DC;
static uint8_t       needMeasure = 1;      // 上电后第一次循环即执行测量
static CaptureResult cr        = { .sig = {0}, .fs_used = 0, .disp_start = 0, .disp_len = ADC_BUF_LEN };
static uint32_t      lastMeasure = 0;
static uint32_t      lastDraw    = 0;

/* 直流零点标定（TARE）：零点漂移取决于每次上电后的热状态，故仅存放于 RAM，掉电即失效 */
static float         dc_offset    = 0.0f;   /* 当前零点偏置(V)，显示时从 Vdc 中扣除 */
static char         *tareMsg      = "";     /* 按键操作反馈文字 */
static uint32_t      tareMsgUntil = 0;      /* 反馈显示的截止时刻(ms) */

/* 现场重标定（CAL）模式：KEY2 长按进入或退出；KEY2 短按在当前稳定输入处采集一个锚点，
   目标值就近吸附到 0.1V 栅格；KEY1 长按清空全部锚点。
   安全护栏：超出标定范围时按线性外推而不钳位封顶；吸附结果须落在给定容差内；
   同一栅格点的重复采集就地更新，目标值倒序的锚点由单调守卫拒绝。 */
static uint8_t       calMode      = 0;
static char         *calMsg       = "";
static uint32_t      calMsgUntil  = 0;
static uint8_t       calDirty     = 0;      /* 本次进入 CAL 后锚点是否被修改（AddCal 或 Clear）：仅在退出时写入 Flash，以减少擦写次数 */

static uint8_t       dispStable   = 1;      /* 最近几次测量结果是否一致；不一致则判定输入正在切换，显示为无效 */

static uint8_t       wave_have    = 0;      /* 是否已成功折叠出至少一帧波形；成立后持续显示最近一次有效波形，不再显示状态提示文字 */

static uint8_t       dcDecimal    = 0;      /* DC 显示分辨率切换：0 为整数 mV，1 为一位小数（0.1mV），由 KEY2 短按切换 */

/* ============================================================
   标定锚点的 Flash 掉电持久化
   ------------------------------------------------------------
   STM32F103C8T6 的 64KB Flash 分为 64 页 × 1KB（中密度）。当前固件代码量约 20.6KB
   （结束地址约 0x08005288），末页 0x0800FC00 未被代码占用，预留用于存放 CAL 锚点。
   · 开机调用 CalFlash_Load()：校验魔数、版本与校验和，通过后把锚点恢复进 Measure（纯 RAM）。
   · 退出 CAL 且本次修改过锚点（calDirty）时调用 CalFlash_Save()：擦除末页后按半字逐次写入当前锚点。
   · 擦除粒度为 1 页、编程单位为半字（16bit）；F103 的擦写寿命不低于 1 万次，而只有标定改动
     才触发一次写入，因此余量充足。
   仅需存放（实测 Vadc, 目标 Vin）两个数组：表前修正所需的 an_v 由 Measure 内部按 g0_vadc 反算，
   基准表本身编在固件里、不写入 Flash。首次上电时 Flash 全为 0xFF，魔数校验不通过，
   系统退回纯基准表（0 个锚点）。
   槽位 = 2 块 × 24 = 48。分两块存放是为了在不改动单块尺寸与布局的前提下把容量翻倍：
   第 1 块沿用既有记录的位置与尺寸，既有固件写入的锚点仍可原样读回；第 2 块置于偏移 256B 处、
   以独立魔数标识，未写入时自然被忽略。两块各自独立校验，其中一块损坏只丢失该块数据。
   记录格式（字段布局、块数量、单块槽位数或地址）发生变化时必须提升 CALFLASH_VER：
   装载时版本不符即整块忽略，否则旧记录会被按新布局误读成错误的锚点。
   风险：锚点位于末页，而擦除以整页为单位，任何整片 Flash 擦除（例如重新烧录时的全片擦除）
   都会连同该页一并清除，锚点随之丢失，需要重新标定；修改存放地址前应确认新页未被代码占用。 */
#define CALFLASH_ADDR   0x0800FC00u         /* 末页起始地址（64KB 容量芯片） */
#define CALFLASH_MAGIC  0xCA1BCE01u
#define CALFLASH_MAGIC2 0xCA1BCE02u         /* 第 2 块的魔数：用于与第 1 块区分，同时可识别该块尚未写入 */
#define CALFLASH_VER    1u
#define CALFLASH_CAP    24u                 /* 单块槽位数（保持结构体尺寸不变，以兼容已有记录） */
#define CALFLASH_B2     (CALFLASH_ADDR + 0x100u)   /* 第 2 块地址：块长 sizeof(CalFlash_t)=208B，故置于 256B 偏移处 */
#define CALFLASH_TOTAL  (CALFLASH_CAP * 2u) /* 合计 48，与 Measure 的 CAL_ANCH_MAX 一致 */

typedef struct {
    uint32_t magic;
    uint32_t ver;
    uint8_t  count;                           /* 有效锚点数，取值 0..CAP */
    uint8_t  pad0[3];
    uint16_t chk;                             /* 数据区校验和（由 calflash_calc 计算） */
    uint16_t pad1;
    float    u[CALFLASH_CAP];                 /* 实测 Vadc(mV) */
    float    tgt[CALFLASH_CAP];               /* 目标 Vin(mV) */
} CalFlash_t;

/* 校验和：对 magic、ver、count 以及前 count 个 u[] 与前 count 个 tgt[] 的 32 位字求和，折叠到 16 位。
   u[] 与 tgt[] 是两个各含 24 个元素的独立数组（并非交错存放），必须按索引分别遍历，
   不能从 &u[0] 起连续读取。 */
static uint16_t calflash_calc(const CalFlash_t *p)
{
    uint32_t s = p->magic + p->ver + p->count;
    const uint32_t *u = (const uint32_t *)&p->u[0];
    const uint32_t *t = (const uint32_t *)&p->tgt[0];
    for (uint8_t i = 0; i < p->count; i++) { s += u[i]; s += t[i]; }
    return (uint16_t)((s & 0xFFFFu) ^ (s >> 16));
}

/* 写：把 Measure 的当前锚点分两块存入末页（共 CALFLASH_TOTAL 个槽）。
   单块的尺寸与布局与既有版本完全一致，因此既有固件写入的单块记录仍可正常读回
   （第 2 块未写入时内容为 0xFF，魔数不符即被忽略）。
   锚点全部清空时同样需要写入：第 1 块以 count=0 落盘，避免下次上电又载入旧锚点。 */
static CalFlash_t cf_blk[2];                     /* 声明为静态存储（2×208B）：主栈仅 1KB，不宜容纳此量级的局部对象 */

static void CalFlash_Save(void)
{
    uint8_t n = Measure_CalCount();
    if (n > CALFLASH_TOTAL) n = CALFLASH_TOTAL;

    for (uint8_t b = 0; b < 2; b++) {
        CalFlash_t *d = &cf_blk[b];
        uint8_t base = (uint8_t)(b * CALFLASH_CAP);
        uint8_t rest = (uint8_t)(n > base ? n - base : 0);
        uint8_t cnt  = (uint8_t)(rest > CALFLASH_CAP ? CALFLASH_CAP : rest);

        d->magic = (b == 0) ? CALFLASH_MAGIC : CALFLASH_MAGIC2;
        d->ver   = CALFLASH_VER;
        d->count = cnt;
        d->pad0[0] = d->pad0[1] = d->pad0[2] = 0;
        d->pad1 = 0;
        for (uint8_t i = 0; i < cnt; i++) Measure_CalGet((uint8_t)(base + i), &d->u[i], &d->tgt[i]);
        for (uint8_t i = cnt; i < CALFLASH_CAP; i++) { d->u[i] = 0.0f; d->tgt[i] = 0.0f; }
        d->chk = calflash_calc(d);

        const uint16_t *w = (const uint16_t *)d;
        uint32_t addr = (b == 0) ? CALFLASH_ADDR : CALFLASH_B2;
        if (b == 0) { FLASH_Unlock(); FLASH_ErasePage(CALFLASH_ADDR); }
        for (uint32_t i = 0; i < sizeof(CalFlash_t) / 2u; i++) {
            if (FLASH_ProgramHalfWord(addr, w[i]) != FLASH_COMPLETE) break;
            addr += 2u;
        }
    }
    FLASH_Lock();
}

/* 读：上电时调用；两块各自独立校验，其中一块损坏仅丢弃该块，保留另一块的可用锚点，不使整组失效。 */
static float cf_mu[CALFLASH_TOTAL], cf_mt[CALFLASH_TOTAL];   /* 两块的合并暂存区，一次性交回 Measure */

static uint8_t CalFlash_Load(void)
{
    const uint32_t blk_addr[2]  = { CALFLASH_ADDR, CALFLASH_B2 };
    const uint32_t blk_magic[2] = { CALFLASH_MAGIC, CALFLASH_MAGIC2 };
    uint8_t tot = 0;

    for (uint8_t b = 0; b < 2; b++) {
        const CalFlash_t *p = (const CalFlash_t *)blk_addr[b];
        if (p->magic != blk_magic[b] || p->ver != CALFLASH_VER) continue;
        if (p->count == 0u || p->count > CALFLASH_CAP) continue;
        if (calflash_calc(p) != p->chk) continue;
        for (uint8_t i = 0; i < p->count && tot < CALFLASH_TOTAL; i++) {
            cf_mu[tot] = p->u[i]; cf_mt[tot] = p->tgt[i]; tot++;
        }
    }
    if (tot == 0u) return 0;
    return Measure_CalRestore(cf_mu, cf_mt, tot);
}

/* 电压（V）转换为毫伏整数，四舍五入；返回值取无符号，故仅适用于非负量 */
static uint32_t To_mV(float v)
{
    int32_t x = (int32_t)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
    return (uint32_t)x;
}

/* 显示稳定门：对最近 3 次有效测量取中位数，用于剔除输入正在调整期间的单帧离群值 */
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
static float hist_spread(const float *h, uint8_t n)   /* 最近 n 次测量的极差 */
{
    float lo = h[0], hi = h[0];
    for (uint8_t i = 1; i < n; i++) { if (h[i] < lo) lo = h[i]; if (h[i] > hi) hi = h[i]; }
    return hi - lo;
}

/* 直流稳定即锁定：最近两次取值已足够接近时视为落定的新值，直接采用最新值，以消除中位数
   对旧电平的记忆滞后，使 10mV 量级的真实台阶一次即可跟随；否则退回中位数，保留剔除单帧毛刺的能力。 */
static float hist_pick(const float *h, uint8_t n, float tol)
{
    if (n >= 2) {
        float d = h[0] - h[1]; if (d < 0) d = -d;
        if (d < tol) return h[0];
    }
    return hist_med(h, n);
}

/* 执行一次采集与处理，把结果写入 cr，并用稳定后的统计值更新 shown */
static void DoMeasure(void)
{
    uint8_t fast = (uint8_t)(!dispStable);   // 上一周期未稳定（输入改变或刚切换模式）时，AC 与波形模式采用快档并减少帧数以提高跟随速度；已稳定时采用全帧以保持精度
    if (calMode)                cr = Capture_DC();          // 标定模式：按直流通道取帧
    else if (mode == MODE_DC)   cr = Capture_DC();          // 直流：由均值换算电压
    else if (mode == MODE_WAVE) cr = Capture_Wave(fast);    // 波形：相干折叠并计算各项指标
    else                        cr = Capture_AC(fast);      // 交流：Vpp、频率、Rms

    if (cr.sig.valid) {
        hist_push(cr.sig.vdc,  h_vdc);
        hist_push(cr.sig.vpp,  h_vpp);
        hist_push(cr.sig.freq, h_freq);
        hist_push(cr.sig.rms,  h_rms);
        if (hCnt < HIST_N) hCnt++;
        /* 显示路由：MODE_DC 使用 DcEstim 做时域估计，兼顾读数稳定与跟随；CAL 模式直接取 16 帧均值，
           不引入历史记忆，因此当场即稳定且不增加延迟，便于判断当前输入电压；AC 与波形模式不显示 vdc，沿用 hist_pick。 */
        if (mode == MODE_DC)       shown.vdc = DcEstim_Feed(cr.sig.vdc);
        else if (calMode)          shown.vdc = cr.sig.vdc;
        else                       shown.vdc = hist_pick(h_vdc, hCnt, 0.015f);
        /* AC 与波形模式：vpp/freq/rms 由 hist_med 改为 hist_pick——连续两次取值足够接近（处于测量噪声
           范围内）时直接采用最新值，立即跟随新电平；仅当彼此差异较大（疑似异常帧）时才退回中位数剔除。
           相比固定中位数少保留一个旧值，输入改变后约少一次刷新周期即可落定，跟随更快；
           容差取在噪声地板之上、孤立异常帧偏差之下。 */
        shown.vpp  = hist_pick(h_vpp,  hCnt, 0.010f + 0.02f * (h_vpp[0]  < 0 ? -h_vpp[0]  : h_vpp[0]));
        shown.freq = hist_pick(h_freq, hCnt, 5.0f   + 0.01f * (h_freq[0] < 0 ? -h_freq[0] : h_freq[0]));
        shown.rms  = hist_pick(h_rms,  hCnt, 0.010f + 0.02f * (h_rms[0]  < 0 ? -h_rms[0]  : h_rms[0]));
        shown.valid = 1;
        if (mode == MODE_WAVE) wave_have = 1;      /* 折叠成功且结果未因多帧混合而作废，显示缓冲区已是可信的一帧波形 */

        /* 稳定性判据：最近若干次测量彼此差异过大时判定为输入正在切换，界面显示为无效。
           AC 模式仅比较最近 2 次，使输入改变后可少等一个周期即结束无效显示；
           DC 与波形模式维持整窗 3 次——波形折叠本身依赖多帧平均，若判据同步放宽，
           会更早显示尚未平均干净的波形，因此这两种模式保留较严格的判据。 */
        if (hCnt >= 2) {
            uint8_t w = (mode == MODE_AC) ? 2u : hCnt;
            float ad = shown.vdc < 0 ? -shown.vdc : shown.vdc;
            /* Vpp 容差按模式区分：AC 模式按正弦 RMS 求值，读数本身稳定，取 3%；波形模式取真实峰峰值，
               边沿振铃是否被采集到随采样相位而变，极差可达读数的 60% 以上，若沿用同一门限将长期判定为
               未稳定，界面会持续显示测量中。波形模式以波形形状与单调变化作为判稳依据，故此处放宽 Vpp 门限；
               频率与直流门限保持严格，因为频率不稳定才真正意味着测量尚未收敛。 */
            float vt = (mode == MODE_WAVE) ? (0.05f + 0.35f * shown.vpp)
                                           : (0.04f + 0.03f * shown.vpp);
            dispStable = (hist_spread(h_vdc,  w) < 0.04f + 0.03f * ad)
                      && (hist_spread(h_vpp,  w) < vt)
                      && (hist_spread(h_freq, w) < 20.0f + 0.02f * shown.freq);
        } else {
            dispStable = 1;
        }
    } else {
        shown.valid = 0;
        dispStable  = 0;      /* 本次无有效帧，或折叠结果因多帧混合而作废：下一循环按 120ms 快轮询立即重测，无需等待 700ms 常规间隔 */
    }
    lastMeasure = Millis();
    needMeasure = 0;
}

/* 触发重测：置为待测状态并清空历史数据与缓存有效性，使切换模式后的第一次结果即时产生且不受旧数据影响 */
static void MarkDirty(void)
{
    needMeasure   = 1;
    cr.sig.valid  = 0;
    shown.valid   = 0;
    hCnt          = 0;
    DcEstim_Reset();                     /* 切换模式或进出 CAL：清空估计器历史，保证第一次输出即时且不受旧数据影响 */
    Capture_Unlock();                    /* 更换模式或进出 CAL：输入信号与用途可能改变，强制下一次重新执行自动量程 */
}

/* ---------- 各模式渲染（仅读取缓存，不触发采集） ---------- */
static void Render_DC(void)
{
    OLED_ShowString(1, 1, "DC Vdc=");                            /* 前缀占 7 列（1-7） */
    if (shown.valid && dispStable) {
        float val = shown.vdc - dc_offset;                        /* 电压(V)，已扣除零点偏置 */
        if (dcDecimal) {                                          /* 一位小数显示（0.1mV） */
            float    tv  = val * 10000.0f;                        /* 以十分之一 mV 为单位取整 */
            int32_t  tm  = (int32_t)(tv + (tv >= 0.0f ? 0.5f : -0.5f));
            uint8_t  neg = (tm < 0);
            uint32_t a   = neg ? (uint32_t)(-tm) : (uint32_t)tm;
            OLED_ShowChar (1, 8,  neg ? '-' : '+');
            OLED_ShowNum  (1, 9,  a / 10u, 4);                    /* 整数 mV，±9999 的量程足以覆盖 0~5V */
            OLED_ShowChar (1, 13, '.');
            OLED_ShowNum  (1, 14, a % 10u, 1);                    /* 小数第 1 位（0.1mV） */
            OLED_ShowString(1, 15, "mV");
        } else {                                                  /* 整数 mV 显示：符号位 + 5 位数字 */
            OLED_ShowSignedNum(1, 8, (int32_t)To_mV(val), 5);
            OLED_ShowString(1, 14, "mV");
        }
    } else {
        OLED_ShowString(1, 8, "Measuring");                        /* 无效或正在切换时整字段给出测量提示，不显示 ---- 占位符 */
    }

    if (Millis() < tareMsgUntil) {                              /* 零点标定的操作反馈 */
        OLED_ShowString(3, 1, "TARE ");
        OLED_ShowString(3, 6, tareMsg);
    } else {
        OLED_ShowChinese(0, 32, CN_SAMPLE);                     // “采样”字样：标明其后为当前采样率档位
        OLED_ShowNum(3, 5, cr.fs_used / 1000u, 3);
        OLED_ShowString(3, 9, "kHz");
    }

    /* 第 4 行：显示当前直流估计器的算法名称（ADAPT、CONF2 等），便于确认读数所采用的稳定策略 */
    OLED_ShowString(4, 1, "est:");
    OLED_ShowString(4, 5, DcEstim_Name());
}

static void Render_AC(void)
{
    /* 未落定时整字段显示 Measuring：仅替换占位符的显示形式，不改变算法与判稳逻辑 */
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

/* 波形页：坐标系与自动 V/div 显示，绘制方案由 WAVE_GRID 单一开关选择，修改后需重新编译。
   WAVE_GRID=1：标注文字改用 5×7 半高字库。8×16 全高字库两行即占满 y=0..31，波形区只剩 29 行，
                无法同时容纳坐标系；半高字库把 Vpp/F 与 V/div、t/div 压缩到 y=0..15，
                使 y=16..63 全部留给波形迹线。迹线按真实输入电压定位：Vpp 变化时纵向伸缩，
                偏置变化时相对 0V 基准线上下平移；V/div 按 1-2-5 序列自动分档，当 0V 基准线被
                偏置移出显示框时，整体平移至恰好不产生截断的位置，并以箭头指示偏移方向。
   WAVE_GRID=0：保留原有画法，两行 8×16 数字占满 y=0..31，波形铺满剩余 29 行，无坐标系。
   注意：电压换算经过标定链；FRONT_END_BYPASS=1 时标定被旁路，格线标注的是 PA0 电压而非输入电压，
        该配置仅用于调试。 */
#define WAVE_GRID 1

#if WAVE_GRID
static uint8_t wyLo[WAVE_FOLD_PTS], wyHi[WAVE_FOLD_PTS];   /* 每列的下沿与上沿行号，由 Capture_WaveRows 计算 */
#endif

static void Render_Wave(void)
{
    uint8_t ok = shown.valid && dispStable;

#if WAVE_GRID
    /* 职责划分：整页布局位于 OLED_WavePage（显示层），码值到电压再到行号的映射位于 Capture_WaveRows（数据层）。
       两层均不依赖具体硬件，可在 PC 端仿真运行，便于在烧写前核对整屏的显示效果。 */
    uint16_t n = 0; int16_t y0v = (int16_t)(WAVE_BOX_Y + WAVE_BOX_H / 2); uint8_t k = 0;
    float tdiv = 0.0f;
    if (wave_have) {
        n = Capture_WaveRows(wyLo, wyHi, WAVE_FOLD_PTS, &k, &y0v);
        if (shown.freq > 0.0f && Capture_WaveCyc > 0.0f)
            tdiv = Capture_WaveCyc * (1000.0f / shown.freq) / (float)WAVE_DIV_H;   /* ms/格 */
    }
    OLED_WavePage(ok ? To_mV(shown.vpp) : 0u, ok ? (uint32_t)shown.freq : 0u,
                  (int32_t)(shown.vdc * 1000.0f + (shown.vdc >= 0.0f ? 0.5f : -0.5f)),
                  ok, (uint8_t)(n != 0), wyLo, wyHi, n,
                  Capture_WaveDivMv(k), tdiv, y0v, WAVE_BOX_Y, WAVE_BOX_H);
#else
    OLED_ShowString(1, 1, "Vpp=");
    if (ok) { OLED_ShowNum(1, 6, To_mV(shown.vpp), 4); OLED_ShowString(1, 11, "mV"); }
    else    { OLED_ShowString(1, 6, "Measuring"); }

    OLED_ShowString(2, 1, "F  =");
    if (ok) { OLED_ShowNum(2, 6, (uint32_t)shown.freq, 5); OLED_ShowString(2, 12, "Hz"); }
    else    { OLED_ShowString(2, 6, "Measuring"); }

    /* 波形区不受稳定门封锁：多帧混合的折叠结果已在 Capture_Wave 中作废且不写入显示缓冲区，
       因此此处绘制的始终是最近一次有效的波形。改变幅度或频率时画面先保持一帧、随即切换到新形状，
       既不会绘制多帧叠加的失真波形，也不会出现闪烁。 */
    OLED_DrawFrame(0, 33, 128, 31, 1);
    if (wave_have) {
        if (Capture_WaveEnv) OLED_DrawWaveEnv(Capture_WaveLo, Capture_WaveHi, Capture_WaveLen, 1, 34, 126, 29);
        else                 OLED_DrawWave (Capture_WaveBuf, Capture_WaveLen, 1, 34, 126, 29);
    } else {
        OLED_ShowString(4, 3, "no signal");
    }
#endif
}

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
    Timer_Init();       // 先启动 1ms 系统时基（Key 扫描与 Millis 均依赖它）
    Key_Init();         // 配置 GPIO（PB3/PB4），并把 Key_Scan 注册为 TIM2 回调
    Capture_Init();     // 初始化 ADC（TIM3 + DMA），默认置于中间量程档
    CalFlash_Load();    // 上电时恢复上次写入 Flash 的标定锚点；无有效记录则退回纯基准表

    while (1)
    {
        /* 1) 非阻塞读取按键事件 */
        if (Key_Click(1) && !calMode) { mode = (uint8_t)((mode + 1) % MODE_CNT); MarkDirty(); }

        if (Key_LongPress(2)) {
            calMode = (uint8_t)(!calMode);
            if (calMode) {
                dc_offset = 0.0f;                             /* 进入 CAL：清除 TARE 偏置，避免其与标定锚点叠加形成双重修正 */
                calDirty  = 0;                                /* 进入 CAL：复位改动标志，退出时据此判断是否需要写入 Flash */
            } else if (calDirty) {
                CalFlash_Save(); calDirty = 0;                /* 退出 CAL 且本次修改过锚点：写入末页持久化；未修改则不写，以减少 Flash 擦写 */
            }
            MarkDirty();
        }                                                   // 进入或退出 CAL 模式

        if (Key_Click(2)) {
            if (calMode) {
                DoMeasure();                                   /* 取最新直流帧：cr.raw_mean 为 16 帧均值码 */
                float mcode = cr.raw_mean;                     /* 与显示采用同一估计值，避免锚点取自单帧造成错位 */
                float vadc_mv = mcode * 3300.0f / 4095.0f;     /* 设备 Vadc(mV)，即 PA0 的实测电压 */
                float v = Measure_MeanToVolt(mcode);           /* 修正后读数：超出范围时线性外推，高段不再被钳位封顶 */
                int32_t K = (int32_t)(v * 10.0f + (v >= 0 ? 0.5f : -0.5f));   /* 目标值就近吸附到 0.1V 栅格 */
                float Kv = (float)K * 0.1f;
                /* 收锚条件：只要求读数有效且显示已稳定，不再要求读数与栅格点的偏离小于 0.2V。
                   原条件会把高段的系统性误差（例如输入 5V 时读得 4.77V，与 5.0V 相差 0.23V）排除在外，
                   而这类误差正是 CAL 需要修正的对象。吸附栅格由 0.5V 细化为 0.1V：0.5V 栅格的就近吸附
                   最多引入 0.25V 偏差，与标定精度同量级；改为 0.1V 后读数与最近格点的偏差不超过 0.05V，
                   吸附本身不再构成误差来源。代价是要求信号源的设定值为 0.1V 的整数倍：设定 1.15V 会被
                   吸附到 1.1V，写入的即是一个错误的锚点。锚点倒序的风险由单调守卫与 dispStable 共同约束。 */
                if (shown.valid && dispStable) {
                    if (Measure_AddCal(vadc_mv, Kv * 1000.0f, 100.0f)) {
                        calMsg = "ADD ";
                        calDirty = 1;                            /* 锚点已被修改：退出 CAL 时写入 Flash */
                        shown.vdc = Measure_MeanToVolt(mcode); /* 立即回算，本次按键即可看到修正后的 Vdc */
                    } else {
                        calMsg = "MONO";                       /* 目标值与已有锚点构成倒序，被单调守卫拒绝（如标定时顺序颠倒） */
                    }
                } else {
                    calMsg = "WAIT";                           /* 读数未稳定或无信号：等待读数落定后再按键（通常输入仍在调整中） */
                }
                calMsgUntil = Millis() + 700u;
            } else if (mode == MODE_DC) {
                dcDecimal = (uint8_t)(!dcDecimal);               /* DC 模式短按：在整数 mV 与一位小数（0.1mV）之间切换 */
            } else {
                MarkDirty();                                   /* 非 DC 模式：短按保留为手动重测 */
            }
        }
        if (Key_LongPress(1)) {                                // CAL 模式下清空全部锚点；其余模式下复位到 DC 模式
            if (calMode) {
                Measure_ClearCal();
                CalFlash_Save();                 /* 立即把空表写回 Flash：防止复位或断电后 CalFlash_Load 又载入旧锚点 */
                calDirty = 0;                    /* 已经落盘，退出 CAL 时无需再次写入 */
                calMsg = "CLEAR"; calMsgUntil = Millis() + 700u;
            }
            else { mode = MODE_DC; MarkDirty(); }
        }

        /* 2) 满足条件即测量：读数未稳定、估计器处于确认状态，或处于 CAL 模式（需要即时反映输入电压）时
              采用快轮询间隔，其余情况恢复常速间隔 */
        uint32_t now = Millis();
        uint32_t iv  = (dispStable && !calMode && !DcEstim_Pending()) ? MEASURE_MS : FAST_MS;
        if (needMeasure || now - lastMeasure >= iv) DoMeasure();

        /* 3) 按帧率上限刷新界面，仅绘制缓存内容 */
        if (now - lastDraw >= REDRAW_MS) { lastDraw = Millis(); Render(); }
    }
}
