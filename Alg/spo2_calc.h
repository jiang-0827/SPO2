/*********************************************************************************************************
:* 模 块 名 称：spo2_calc.h
:* 摘        要：屏幕窗统计 + R 值（比率法）+ 血氧饱和度查表
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：对应原型：SPO2DynamicPrj/RealAnalyzeData.m（收满一屏时的处理）
:*                     + SPO2DynamicPrj/CalcSPO2.m
:*
:*                 irMax = max(IR屏)  irMin = min(IR屏)   irADRng = irMax - irMin
:*                 redMax = max(Red屏) redMin = min(Red屏) redADRng = redMax - redMin
:*                 gIRADAvg  = (irMax  + irMin )/2        gRedADAvg = (redMax + redMin)/2
:*                 if irADRng > 0:
:*                     gRValBuff 滑入 R = redADRng * 1000 / irADRng
:*                 rVal = (sort(gRValBuff)[2] + sort(gRValBuff)[3]) / 2   // 0基下标：中间两个
:*                 index = 1; while rVal >= RR_TABLE[index-1] && index < 11: index++
:*                 spo2 = 101 - index;  spo2 == 100 → 99
:*
:*               两点说明（都是刻意的，不是漏做）：
:*                 1) 为什么用"原始码值的峰峰值"而不用滤波后的交流幅值：
:*                    MATLAB 就是这么干的（CalcSPO2 直接吃 gDataAxes1/gDataAxes2）。峰峰值
:*                    对直流不敏感，所以滤波与否在数值上只差"高频噪声抬高的那一点点"，
:*                    而原始码值不受 IIR 群延迟/相位非线性影响 —— 保真优先。
:*                 2) 为什么 gIRADAvg/gRedADAvg 要单独提出来：
:*                    它们是光强自动调节（RoughAdj/FineAdj/JudgeReAdj）的输入，单位是
:*                    "码值"，必须与 spo2_config.h 里的中线阈值同量纲。
:*
:*               用法（逐样本 push，满屏时返回 1 再调 run）：
:*                   static spo2_calc_t g_calc;                     // 屏缓冲约 8KB
:*                   spo2_calc_init(&g_calc);
:*                   if(spo2_calc_push(&g_calc, irAd, redAd))
:*                   {
:*                     spo2_calc_screen_run(&g_calc);             // 每 4.096s 一次
:*                   }
*********************************************************************************************************/
#ifndef SPO2_CALC_H
#define SPO2_CALC_H

#include <stdint.h>
#include "spo2_config.h"
#include "spo2_coeffs.h"

typedef struct {
    /* ---- 一屏原始码值（对应 gDataAxes1 / gDataAxes2）---- */
    uint16_t irBuf[SPO2_SCREEN_LEN];
    uint16_t redBuf[SPO2_SCREEN_LEN];
    uint16_t pos;                       /* 屏幕写指针(0 基) */
    uint8_t  screenFull;                /* 已累计满一屏 */
    uint32_t sampleCnt;                 /* 累计样本数 */
    uint16_t screenCnt;                 /* 已完成屏数(诊断用) */

    /* ---- 每屏统计（对应 gIRADAvg / gRedADAvg 与两个 ADRng）---- */
    uint16_t irMax;
    uint16_t irMin;
    uint16_t redMax;
    uint16_t redMin;
    uint16_t irRng;
    uint16_t redRng;
    uint16_t irAvg;
    uint16_t redAvg;

    /* ---- 灌注指数 PI（新增，对应验收项"灌注值数 3%"）----
     * PI×100：3.00% → 300。整数存储，避免 %f（MicroLIB 不支持）。*/
    uint16_t piIrX100;                  /* 红外通道灌注指数×100 */
    uint16_t piRedX100;                 /* 红光通道灌注指数×100 */

    /* ---- R 值与血氧（对应 gRValBuff / spo2）---- */
    float    rBuf[SPO2_R_BUF_LEN];
    uint8_t  rPtr;
    float    rVal;                      /* 中值+标定后的 R 值(可能带小数, 与 MATLAB 一致) */
    float    rRaw;                      /* 未做标定的 R 值中值(诊断用) */
    uint16_t rIndex;                    /* 查表索引 1..11(诊断用) */
    int16_t  spo2;                      /* 最新血氧；SPO2_INVALID_DATA = 无效 */

    /* ---- R 值现场标定（R_used = R_raw * gain/1000 + offset）---- */
    int16_t  rCalGainPm;                /* 千分比增益，默认 SPO2_R_CAL_GAIN_DEF */
    int16_t  rCalOffset;                /* 绝对偏置，默认 SPO2_R_CAL_OFFSET_DEF */
} spo2_calc_t;

void    spo2_calc_init(spo2_calc_t *c);
void    spo2_calc_reset(spo2_calc_t *c);   /* 对应 ResetAdj：R 值缓冲复位为 SPO2_R_SEED */
uint8_t spo2_calc_push(spo2_calc_t *c, uint16_t irAd, uint16_t redAd);
/* allowR = CheckRoughAdj()：MATLAB 侧只有粗调完成后才调 CalcSPO2，
 * 所以 R 值缓冲里只会攒"已标定"的屏；C 侧保持同一门控，避免标定期的
 * 垃圾幅度污染 6 点中值（屏统计量无论如何都要算，光强调节等着用） */
void    spo2_calc_screen_run(spo2_calc_t *c, uint8_t allowR);
void    spo2_calc_set_invalid(spo2_calc_t *c);   /* 手指脱落/未标定时把结果置为无效值 */

/* R 值现场标定：把模拟器的读数和标称值对齐（验收"血氧误差 ±2%"的唯一调节手段）。
 * gainPm 为千分比增益(500~1500)，offset 为绝对偏置(-300~300)；
 * 返回 1 = 参数合法已生效，0 = 越界被拒。*/
uint8_t spo2_calc_set_r_cal(spo2_calc_t *c, int16_t gainPm, int16_t offset);
void    spo2_calc_get_r_cal(const spo2_calc_t *c, int16_t *pGainPm, int16_t *pOffset);

#endif /* SPO2_CALC_H */
