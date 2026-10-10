/*********************************************************************************************************
:* 模 块 名 称：spo2_light.h
:* 摘        要：光强自动调节 + 手指脱落判决状态机
:* 当前版本：1.0.0
:* 作        者：Jaziel.Jiang(COPYRIGHT 2018 - 2026 Jaziel.Jiang. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：对应原型：SPO2DynamicPrj/RoughAdj.m（粗调）
:*                     SPO2DynamicPrj/FineAdj.m（细调）
:*                     SPO2DynamicPrj/JudgeReAdj.m（是否需要重新调光）
:*                     SPO2DynamicPrj/ResetAdj.m（复位调光参数）
:*                     SPO2DynamicPrj/CheckRoughAdj.m（粗调完成判据）
:*                     SPO2DynamicPrj/RealAnalyzeData.m 里 gFingerOnFlag 的维护
:*
:*               【架构差异：SendAdjCmd 去哪了】
:*                 MATLAB 里 RoughAdj/FineAdj 每改一次光强就调 SendAdjCmd(ir,red) 把 DA 值
:*                 下发给"从机"（血氧模块）。本工程里 MCU 就是那个"从机"——它自己测光、
:*                 自己调、自己输出 DAC —— 所以 SendAdjCmd 被"把当前 DA 值交给 SPO2FE 写
:*                 DAC"取代：本模块只维护 DA 值，由上层（spo2_alg → Main）每拍读出并写 DAC。
:*
:*               【调用节拍（与 MATLAB 一一对应）】
:*                 RealDraw 定时器 Period = 0.1s → spo2_light_rough_step() 每
:*                     SPO2_ADJ_PERIOD_SMP(50) 个样本调一次
:*                 "粗调完成后每满一屏"           → spo2_light_request_readj() +
:*                     spo2_light_judge_step()
:*
:*               【复位联动】
:*                 MATLAB 的 ResetAdj() 还会把 gRValBuff 复位成 6 个 560、gPRBuf 复位成 6 个
:*                 75。C 侧把这两个缓冲放在 spo2_calc / spo2_pulse 里，所以本模块只用
:*                 resetFlag 通知上层"刚刚 ResetAdj 了"，由上层去复位那两个缓冲。
:*
:*               用法：
:*                   static spo2_light_t g_light;                       // 约 40 字节
:*                   spo2_light_init(&g_light);
:*                   spo2_light_set_sample(&g_light, irAd, redAd);      // 每样本
:*                   // 每 SPO2_ADJ_PERIOD_SMP 个样本：
:*                   spo2_light_rough_step(&g_light);
:*                   if(g_light.resetFlag) { spo2_calc_reset(&g_calc); spo2_pulse_reset(&g_pr);
:*                     g_light.resetFlag = 0; }
:*                   // 满一屏：
:*                   spo2_light_set_screen_avg(&g_light, irAvg, redAvg);
:*                   if(spo2_light_is_rough_done(&g_light)) {
:*                     spo2_light_request_readj(&g_light);
:*                     spo2_light_judge_step(&g_light);
:*                   }
*********************************************************************************************************/
#ifndef SPO2_LIGHT_H
#define SPO2_LIGHT_H

#include <stdint.h>
#include "spo2_config.h"

typedef struct {
    /* ---- 手指/探头状态（对应 gFirDetFinger / gFingerOnFlag 与两个防抖计数器）---- */
    uint8_t  firDetFinger;          /* 重新调光后是否第一次检测到手指 */
    uint8_t  fingerOn;              /* 1=连接 0=脱落（对应 gFingerOnFlag） */
    uint16_t astableFingerCnt;      /* 手指不稳定(脱落)计数器 */
    uint16_t stableFingerCnt;       /* 手指稳定(接入)计数器 */

    /* ---- 探头脱落判别（本工程新增，验收项"自动判别手指脱落与探头脱落"）----
     * 探头脱落 = 光电管完全收不到 LED 的光（归一后码值趴在满量程一侧），
     * 与"探头在、LED 亮、只是没夹手指"（码值趴在 0 一侧）是电学上可分的两种极端。*/
    uint8_t  probeOn;               /* 1=探头已连接 0=探头脱落 */
    uint8_t  offState;              /* EnumSPO2OffState: NONE / FINGER / PROBE */
    uint16_t probeOffCnt;           /* 探头脱落防抖计数 */

    /* ---- 粗调状态（对应 gRoughAdjIRFlag / gRoughAdjRedFlag 及条件计数器）---- */
    uint8_t  roughAdjIrFlag;
    uint8_t  roughAdjRedFlag;
    uint16_t roughAdjIrCondCnt;
    uint16_t roughAdjRedCondCnt;
    uint16_t adjIrDelayCnt;         /* 粗调节拍分频计数器 */
    uint16_t adjRedDelayCnt;

    /* ---- 重新调光判据（对应 gIROutRngCnt / gRedOutRngCnt）---- */
    uint16_t irOutRngCnt;
    uint16_t redOutRngCnt;

    /* ---- 光强 DA 值（对应 gIRIntDAVal / gRedIntDAVal）---- */
    uint16_t irIntDa;
    uint16_t redIntDa;

    /* ---- 最近码值与每屏均值（对应 gIRADVal/gRedADVal 与 gIRADAvg/gRedADAvg）---- */
    uint16_t irAd;
    uint16_t redAd;
    uint16_t irAdAvg;
    uint16_t redAdAvg;

    /* ---- 触发标志 ---- */
    uint8_t  initAvgValid;          /* 是否已经收到过一屏均值 */
    uint8_t  reAdjReq;              /* 对应 gAdjFlag：满一屏且粗调完成时置 1 */
    uint8_t  resetFlag;             /* 本模块刚执行过 ResetAdj，上层需复位 R/PR 缓冲 */
    uint8_t  manualMode;            /* 1=光强由上位机命令接管，本模块不再自动调 */
} spo2_light_t;

void    spo2_light_init(spo2_light_t *h);
void    spo2_light_reset(spo2_light_t *h);                       /* ResetAdj */
void    spo2_light_set_sample(spo2_light_t *h, uint16_t irAd, uint16_t redAd);
void    spo2_light_set_screen_avg(spo2_light_t *h, uint16_t irAvg, uint16_t redAvg);
void    spo2_light_rough_step(spo2_light_t *h);                  /* RoughAdj + 脱落判别 */
void    spo2_light_request_readj(spo2_light_t *h);               /* gAdjFlag = 1 */
void    spo2_light_judge_step(spo2_light_t *h);                  /* JudgeReAdj + FineAdj */
uint8_t spo2_light_is_rough_done(const spo2_light_t *h);         /* CheckRoughAdj */
uint8_t spo2_light_is_finger_on(const spo2_light_t *h);
uint8_t spo2_light_is_probe_on(const spo2_light_t *h);           /* 1=探头已连接 */
uint8_t spo2_light_get_off_state(const spo2_light_t *h);         /* EnumSPO2OffState */
void    spo2_light_get_da(const spo2_light_t *h, uint16_t *pIrDa, uint16_t *pRedDa);
void    spo2_light_apply_manual(spo2_light_t *h, uint16_t irDa, uint16_t redDa); /* 上位机接管光强 */

#endif /* SPO2_LIGHT_H */
