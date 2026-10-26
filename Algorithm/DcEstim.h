#ifndef __DCESTIM_H
#define __DCESTIM_H

#include <stdint.h>

/* ===== DcEstim —— 直流采样估计器（可切换算法层，纯算法，不访问硬件） =====
 * 作用对象：Capture_DC 完成 16 帧块平均后得到的直流电压估计（单位 V）的时间序列。
 * 本层决定如何把逐帧测量合成为屏上稳定且准确的读数，与标定基表相互正交：
 * 估计器只改变读数的时序统计特性，不改变码值到电压的换算关系。
 *
 * 放在这一层的原因：读数抖动来源于估计量的方差。平滑标定基表会引入约 20mV 的折点偏置，
 * 直接损害精度；时域鲁棒估计则能在不牺牲精度的前提下抑制坏帧抖动并加快跟随阶跃。
 *
 * 用法：修改 DC_ESTIM 的取值并重新编译即可切换算法，直流界面会显示当前算法短名。
 *   DC_EST_HISTLOCK 0 = 稳定即锁：近两次差值足够小时取最新值，否则取窗口中位数
 *   DC_EST_MEDIAN   1 = 窗口中位数：抑制单帧毛刺，但记忆滞后，小台阶会被吞掉
 *   DC_EST_BLOCK    2 = 窗口算术均值：最平滑，但不抑制毛刺
 *   DC_EST_TRIM     3 = 截尾均值：去掉最高与最低各 K 个后再平均，兼顾抑制毛刺与平滑
 *   DC_EST_EMA      4 = 指数滑动均值：运算量小、跟随快，但毛刺会进入记忆
 *   DC_EST_HAMPEL   5 = 中位数+MAD 野值门限：超限样本钳回中位数后再做 EMA
 *   DC_EST_CONF     6 = 确认门限（confirm-gate）：超阈且连续 CONFIRM 帧同向才跳变，
 *                       孤立帧丢弃；真阶跃约 2~3 帧跟上，稳态由 EMA 压低抖动
 *   DC_EST_CONF2 与 DC_EST_ADAPT 的说明见下方宏定义处。
 * PC 端数值仿真的对比结果：BLOCK/EMA 遇孤立坏帧时峰峰抖动达 350~570mV；
 * TRIM/HAMPEL/CONF 可将毛刺压到约 2.6mV，稳态峰峰抖动约 5mV，且不损失精度。
 * ====================================================================== */

#define DC_EST_HISTLOCK 0
#define DC_EST_MEDIAN   1
#define DC_EST_BLOCK    2
#define DC_EST_TRIM     3
#define DC_EST_EMA      4
#define DC_EST_HAMPEL   5
#define DC_EST_CONF     6
#define DC_EST_CONF2    7   /* CONF 的分档改进版：偏差大于 HUGE 时立即采纳并清空记忆；介于 JUMP 与 HUGE 之间仍需连续同向确认以抑制孤立坏帧；小于 JUMP 时走 EMA */
#define DC_EST_ADAPT    8   /* 持续性自适应：以连续同向偏离而非幅度作判据，故能跟随 20mV 量级的小台阶。大于 HUGE 时 1 帧采纳；大于 SHIFT 时累计同向帧数，达到 CONFIRM 帧判为真台阶并整值跟随，未确认前保持不变；带内偏差用慢系数平滑 */

/* ---- 默认算法：DC_ESTIM=8（ADAPT，持续性自适应）。
   选择依据：在 20~50mV 的中等台阶下，CONF2 依靠 EMA 逐步逼近新电平，PC 端仿真给出 6~10s 的
   收敛时间；ADAPT 在确认台阶后直接采纳新值，并由 main 依据 DcEstim_Pending 临时改用快轮询，
   同一台阶的收敛时间缩短到约 1.9s，稳态抖动与抗坏帧能力不劣于其他档。
   也可在编译命令行用 /DDC_ESTIM=n 覆盖（PC 端逐个扫描算法即使用该方法），无需修改本文件。 ---- */
#ifndef DC_ESTIM
#define DC_ESTIM        8
#endif

/* ---- 可调参数（除 DC_EST_WIN、DC_EST_TRIM_K、DC_EST_CONFIRM 为个数外，单位均为电压 V）。
       均可用 /DDC_EST_xxx=.. 在编译命令行覆盖，便于参数扫描。 ---- */
#ifndef DC_EST_WIN
#define DC_EST_WIN      5      /* 窗口内的测量个数（每个测量本身已含 16 帧平均） */
#endif
#ifndef DC_EST_TRIM_K
#define DC_EST_TRIM_K   1      /* 截尾均值在高低两端各去掉的个数（需 WIN > 2*K） */
#endif
#ifndef DC_EST_ALPHA
#define DC_EST_ALPHA    0.15f  /* EMA/HAMPEL/CONF 的稳态低通系数（0~1，越小越平稳但跟随越慢）；建议范围 0.10~0.18 */
#endif
#ifndef DC_EST_TOL
#define DC_EST_TOL      0.015f /* HISTLOCK 的稳定判据：近两次差值小于此值时直接取最新值 */
#endif
#ifndef DC_EST_JUMP
#define DC_EST_JUMP     0.06f  /* CONF 的跳变门限：|新值-当前估计|大于此值时按可能的真实跳变处理，否则视为噪声 */
#endif
#ifndef DC_EST_CONFIRM
#define DC_EST_CONFIRM  3      /* 同向大偏差连续达到该帧数才确认为真跳变并跟随 */
#endif
#ifndef DC_EST_HUGE
#define DC_EST_HUGE     0.4f   /* 巨大跳变门限(V)：超过此值判定为输入确实改变（远超噪声幅度），立即采纳并清空记忆 */
#endif
#ifndef DC_EST_MADK
#define DC_EST_MADK     3.0f   /* HAMPEL 野值判据：偏离中位数超过 K×MAD 即视为野值 */
#endif
/* ---- ADAPT（持续性自适应）参数 ---- */
#ifndef DC_EST_SHIFT
#define DC_EST_SHIFT    0.010f /* 噪声地板(V)：偏差大于此值才算有效偏离。DC 经 16 帧平均后 σ 约 3mV，此处取约 3σ */
#endif
#ifndef DC_EST_A_FAST
#define DC_EST_A_FAST   0.60f  /* 判定为真台阶后的快响应系数（当前 ADAPT 实现为整值采纳，未引用该系数） */
#endif
#ifndef DC_EST_A_SLOW
#define DC_EST_A_SLOW   0.15f  /* 带内噪声的慢响应系数，使低位读数保持稳定 */
#endif

/* 每获得一个有效测量值就调用一次，返回当前应显示的估计值(V)；内部按 DC_ESTIM 选择算法。 */
float DcEstim_Feed(float v_meas);

/* 切换模式或进出现场标定(CAL)时调用：清空历史，保证切换后的第一次读数立即生效且不受旧数据影响。 */
void  DcEstim_Reset(void);

/* 是否处于"正在确认一个候选真实台阶"的挂起态（CONF/CONF2/ADAPT 使用）：
   返回 1 表示估计器尚未落到新电平，main 应临时改用快轮询间隔(FAST_MS)尽快采集后续帧，
   使跳变在约 CONFIRM×120ms 内完成，而不被 700ms 的正常重测间隔拖慢。其余算法恒返回 0。 */
uint8_t DcEstim_Pending(void);

/* 当前算法的短名，供界面角标显示，用于确认正在运行的算法。返回 char* 以匹配 OLED_ShowString 的形参类型（ARMCC v5）。 */
char* DcEstim_Name(void);

#endif /* __DCESTIM_H */
