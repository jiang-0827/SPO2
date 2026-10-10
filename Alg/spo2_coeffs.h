/*********************************************************************************************************
:* 模 块 名 称：spo2_coeffs.h
:* 摘        要：滤波系数与血氧查表常量（由 gen_spo2_coeffs.m 自动生成，勿手改！）
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：生成来源：SPO2DynamicPrj/FilterPulseWave.m（butter(5, 20/(500/2))）
:*                     + SPO2DynamicPrj/CalcSPO2.m（RR_TABLE）
:*               生成时间：2026-09-24
:*
:*               自检记录：
:*                 HP    二阶 Butterworth 高通 fc=0.5Hz @Fs=500 → 0.5Hz 处 -3.0dB
:*                 LP    五阶 Butterworth 低通 fc=20Hz  @Fs=500 → 20Hz 处 -3.0dB，
:*                     40Hz 处 -30.8dB，50Hz 处 -41.0dB
:*                 NOTCH 50Hz 陷波 beta=0.96（MATLAB 侧默认不启用）
:*                 DF2T 递推 vs scipy.signal.lfilter 逐样本误差 < 3e-9（double 域）
:*                 系数已按 a(1) 归一化；被关闭的级为直通节 [1 0 0]/[1 0 0]
:* 注        意：MATLAB 侧 FilterPulseWave.m 每次都现场调 butter() 重新设计，C 侧把
:*               "阶数 5 + 截止 20Hz" 固化成这里的常量，滤波结果逐样本等价（同一个
:*               传递函数 + 同一种转置直接II型递推，差异只有 single 舍入）。
*********************************************************************************************************/
#ifndef SPO2_COEFFS_H
#define SPO2_COEFFS_H

/* ---- 各级阶数（= 系数数组长度 - 1）---- */
#define SPO2_HP_ORDER      2
#define SPO2_LP_ORDER      5
#define SPO2_NOTCH_ORDER   2
#define SPO2_IIR_MAX_ORDER 5      /* 状态数组按最高阶分配 */

/* ---- 0.5Hz 二阶 Butterworth 高通（去基线漂移）---- */
extern const float SPO2_HP_B[3];
extern const float SPO2_HP_A[3];

/* ---- 50Hz 陷波（beta = 0.96）---- */
extern const float SPO2_NOTCH_B[3];
extern const float SPO2_NOTCH_A[3];

/* ---- 20Hz 五阶 Butterworth 低通（对应 MATLAB 的主滤波）---- */
extern const float SPO2_LP_B[6];
extern const float SPO2_LP_A[6];

/* ---- 血氧饱和度 R 值 → SpO2 的查表（对应 CalcSPO2.m 的 RR_TABLE）----
 * 目录索引从 1 起算：while(R >= TABLE(idx)) idx++;
 * 结果 spo2 = 101 - idx（100 时按 99 上报）。 */
extern const unsigned short SPO2_RR_TABLE[11];

#endif /* SPO2_COEFFS_H */
