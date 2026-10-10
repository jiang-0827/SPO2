/*********************************************************************************************************
:* 模 块 名 称：spo2_pulse.h
:* 摘        要：逐点流式脉率检测（峰定位 + 中位数RR → bpm + 6点中值滤波）
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：对应原型：SPO2DynamicPrj/CalcPulseRate.m
:*                     [~,index] = findpeaks(dataIn,'minpeakdistance',250,'minpeakheight',100);
:*                     pr = int16(30000/median(diff(index)));
:*                     gPRBuf 6 点滑入 → sort → 取中间两个求平均 → 越界置 -100
:*               对应参考：同工程 Alg/ecg_heart.c（心电那套 500Hz 在线峰值检测）
:*
:*               【与 MATLAB 原型的一处刻意偏离，必须知道】
:*                 MATLAB 把 findpeaks 直接作用在**原始 24bit AD 码值**上（含 ~2000 码直流），
:*                 minpeakheight=100 是绝对门限，在那个量纲下几乎等于"无门限"，真正起作用的
:*                 只有 minpeakdistance。C 侧把峰值检测搬到了**滤波后的交流域**（信号更干净、
:*                 与心电移植保持一致的设计），量纲随之改变，absolute 100 不再有意义，故改用
:*                 "自适应门限 + 不应期"：
:*                     thr = winMin + alpha*(winMax - winMin)     (alpha = SPO2_PR_THR_ALPHA)
:*                     不应期 = SPO2_PR_MIN_DIST = 250 样本 = 0.5s
:*                 不应期与 minpeakdistance 语义完全等价（都限定了最大可测脉率 120bpm）；
:*                 门限部分由 SPO2_PR_ABS_MIN / SPO2_PR_THR_ALPHA 标定，需按实采波形回填。
:*                 后端的"median(RR) → pr"与"gPRBuf 6 点中值 → 越界置 -100"两条链路严格照搬，
:*                 输出数值行为与 MATLAB 一致。
:*
:*               算法（全部流式，无 malloc，无 math.h）：
:*                 1) 滑窗极值：最近 SPO2_PR_WIN_LEN(2048) 个样本按 SPO2_PR_BLK_LEN(64)
:*                    分块追踪极值，块提交时刷新窗 max/min —— 4 秒自适应窗；
:*                 2) 自适应门限：thr = min + alpha*(max-min)；
:*                 3) 峰检测状态机：样本首次过阈 → 跟踪候选峰（记录峰值与其"最后创新高"
:*                    位置）；峰后连续 SPO2_PR_CONFIRM_FALL 个样本回落 → 确认峰；
:*                    确认后进入 SPO2_PR_MIN_DIST(250 样本=0.5s) 不应期，忽略重搏波/噪声；
:*                 4) RR 入环形缓冲（最近 8 个）取中位数 → pr_raw = 30000/medRR（四舍五入）；
:*                 5) pr_raw 滑入 6 点缓冲取中间两个平均 → 越界 [20,120] 置 SPO2_INVALID_DATA。
:*
:*               用法（在滤波链之后逐样本调用）：
:*                   static spo2_pulse_t g_pr;                 // 约 320 字节 RAM
:*                   spo2_pulse_init(&g_pr);                   // 上电初始化一次
:*                   int16_t pr = spo2_pulse_step(&g_pr, y);   // y = 滤波后样本; -100 = 无效
:* 注        意：边界与保护：
:*                 - 窗峰峰值 < SPO2_PR_ABS_MIN → 判"信号丢失"：清 RR 历史、输出无效值；
:*                 - 上电前半个窗（2s）不输出脉率，避免未标定的窗极值误检；
:*                 - 不应期从"峰位置"起算（扣除确认延迟），避免高频脉率周期性丢峰。
*********************************************************************************************************/
#ifndef SPO2_PULSE_H
#define SPO2_PULSE_H

#include <stdint.h>
#include "spo2_config.h"

/* 滑窗分块极值的块数 = 2048/64 = 32 块 */
#define SPO2_PR_BLK_NUM         (SPO2_PR_WIN_LEN / SPO2_PR_BLK_LEN)
/* RR 合法性上限（样本） = 60*Fs / 脉率下限 */
#define SPO2_PR_RR_MAX          (uint32_t)(60.0f * SPO2_FS / (float)SPO2_PR_MIN)

typedef struct {
    /* ---- 滑窗分块极值（自适应门限） ---- */
    uint16_t blkPos;                            /* 当前块内样本计数 */
    float    blkMax;                            /* 当前块最大值 */
    float    blkMin;                            /* 当前块最小值 */
    float    blkMaxArr[SPO2_PR_BLK_NUM];        /* 块最大值环形缓冲 */
    float    blkMinArr[SPO2_PR_BLK_NUM];        /* 块最小值环形缓冲 */
    uint8_t  blkPtr;                            /* 环形块写指针 */
    uint8_t  blkValid;                          /* 已提交的有效块数(<=32) */
    float    winMax;                            /* 当前窗最大值(提交时刷新) */
    float    winMin;                            /* 当前窗最小值(提交时刷新) */
    uint8_t  sigValid;                          /* 窗峰峰值是否达标(信号存在) */

    /* ---- 峰检测状态机 ---- */
    uint8_t  peakActive;                        /* 是否在跟踪候选峰 */
    float    peakVal;                           /* 候选峰幅值 */
    uint32_t peakPos;                           /* 候选峰位置(最后创新高处) */
    uint16_t fallCnt;                           /* 峰后回落计数 */
    uint16_t refractory;                        /* 不应期倒计时(样本) */

    /* ---- RR 与脉率 ---- */
    uint32_t sampleIdx;                         /* 累计样本序号 */
    uint32_t lastPeakIdx;                       /* 上一个确认峰位置 */
    uint8_t  hasLastPeak;                       /* 是否已有参考峰 */
    uint16_t rrnArr[SPO2_PR_MAX_RR];            /* 最近 RR 环形缓冲(样本数) */
    uint8_t  rrCnt;                             /* 有效 RR 个数 */
    uint8_t  rrPtr;                             /* RR 环形写指针 */
    uint16_t medRR;                             /* R-R 中位数(诊断用) */
    uint16_t peakCount;                         /* 累计确认峰数(诊断用) */

    int16_t  prBuf[SPO2_PR_BUF_LEN];            /* gPRBuf：6 点脉率中值缓冲 */
    uint8_t  prPtr;                             /* 环形写指针 */
    int16_t  prRaw;                             /* 本次 median(RR) 得到的原始脉率 */
    int16_t  pr;                                /* 最新脉率；SPO2_INVALID_DATA = 无效 */
} spo2_pulse_t;

void    spo2_pulse_init(spo2_pulse_t *h);
int16_t spo2_pulse_step(spo2_pulse_t *h, float x);
void    spo2_pulse_reset(spo2_pulse_t *h);   /* 对应 ResetAdj：只复位脉率缓冲 */

#endif /* SPO2_PULSE_H */
