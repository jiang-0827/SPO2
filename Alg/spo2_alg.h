/*********************************************************************************************************
:* 模 块 名 称：spo2_alg.h
:* 摘        要：血氧算法顶层聚合（对外的唯一入口）
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：把 MATLAB 侧散落在 RealAnalyzeData.m / RealDraw 定时器里的调用顺序
:*               固化成一个"每样本一步"的状态机：
:*
:*                 每样本 :
:*                   ① spo2_filt_step()          滤波（扣直流→0.5Hz高通→[50Hz陷波]→20Hz低通）
:*                   ② spo2_pulse_step()         逐点脉率检测
:*                   ③ spo2_calc_push()          写入屏幕缓冲（IR/RED 各 2048 点）
:*                   ④ spo2_light_set_sample()   更新最近码值供光强调节用
:*                 每 SPO2_ADJ_PERIOD_SMP 个样本（= 0.1s，对应 MATLAB RealDraw 的 0.1s）:
:*                   ⑤ spo2_light_rough_step()   粗调（RoughAdj）
:*                   ⑥ spo2_light_judge_step()   重调判据 + 细调（JudgeReAdj + FineAdj）
:*                 每满一屏（2048 样本 = 4.096s，对应 RealAnalyzeData 的"接收满一屏"）:
:*                   ⑦ spo2_calc_screen_run()    屏统计 + R 值 + 血氧查表
:*                   ⑧ spo2_light_set_screen_avg() 把 gIRADAvg/gRedADAvg 交给光强调节
:*                   ⑨ (已移除) 原 spo2_fft_run() 功率谱；结果帧的 fundHz 字段恒为 0
:*
:*               输出语义（与 MATLAB 上位机一致）：
:*                 spo2 / pulseRate == SPO2_INVALID_DATA(-100) ？ 上位机显示 "---"
:*                 判无效的条件：手指脱落（gFingerOnFlag==0）或粗调未完成（CheckRoughAdj()==0）
:*
:*               用法：
:*                   static spo2_alg_t g_spo2Alg;              // 必须 static/全局
:*                   spo2_alg_init(&g_spo2Alg);                // 上电初始化一次
:*                   spo2_alg_step(&g_spo2Alg, irAd, redAd);   // 每 2ms 调一次
:*                   r = spo2_alg_get_result(&g_spo2Alg);
:*                   // r->spo2 / r->pulseRate / r->irDa / r->redDa / r->irFilt ...
:* 注        意：【RAM 提醒】spo2_alg_t 里含 8KB 屏缓冲（FFT 工作区已于 2026-10-10 移除），**必须**声明成
:*               static/全局（.bss），不要放在栈上（默认栈只有 1KB，会直接 HardFault）。
*********************************************************************************************************/
#ifndef SPO2_ALG_H
#define SPO2_ALG_H

#include <stdint.h>
#include "spo2_config.h"
#include "spo2_filter.h"
#include "spo2_pulse.h"
#include "spo2_calc.h"
#include "spo2_light.h"

typedef struct {
    uint8_t  fingerOn;          /* 1=手指连接 0=脱落 */
    uint8_t  probeOn;           /* 1=探头连接 0=探头脱落（新增，验收项"判别两种脱落"）*/
    uint8_t  offState;          /* EnumSPO2OffState: NONE/FINGER/PROBE（新增）*/
    uint8_t  adjDone;           /* 1=粗调完成（CheckRoughAdj） */
    uint8_t  screenDone;        /* 1=本样本刚好写满一屏（用于触发上报/打点） */
    uint16_t irAd;              /* 最新红外码值（已按 SPO2_FE_INVERT 归一） */
    uint16_t redAd;             /* 最新红光码值 */
    uint16_t irDa;              /* 当前红外光强 DA（上层据此写 DAC） */
    uint16_t redDa;             /* 当前红光光强 DA */
    float    irFilt;            /* 最新滤波后的红外脉搏波（交流域，送主机画波形） */
    int16_t  pulseRate;         /* 脉率 bpm；SPO2_INVALID_DATA = 无效 */
    int16_t  spo2;              /* 血氧饱和度 %；SPO2_INVALID_DATA = 无效 */

    uint16_t piIrX100;          /* 红外灌注指数×100（新增，验收项"灌注值数 3%"→300）*/
    uint16_t piRedX100;         /* 红光灌注指数×100（新增）*/
    uint8_t  piValid;           /* 1=PI 达标（确实有脉搏波），0=整屏无脉动 */

    int16_t  rCalGainPm;        /* 当前 R 值标定增益(千分比，见 CMD_SPO2_R_CAL)。
                                 * 原为 uint8_t：gain=1636 会被截成 116，使 0x84/0x8B
                                 * 的 calGain 回显失真，标定时容易误判"参数没生效" */

    uint16_t irMax;             /* 本屏红外最大码值 */
    uint16_t irMin;             /* 本屏红外最小码值 */
    uint16_t redMax;            /* 本屏红光最大码值 */
    uint16_t redMin;            /* 本屏红光最小码值 */
    uint16_t irRng;             /* 红外峰峰值（irADRng） */
    uint16_t redRng;            /* 红光峰峰值（redADRng） */
    uint16_t irAvg;             /* 红外屏均值（gIRADAvg） */
    uint16_t redAvg;            /* 红光屏均值（gRedADAvg） */
    float    rVal;              /* R 值（经标定后的值，用于查表） */
    float    rRaw;              /* 未标定的 R 值（诊断/标定用，新增） */
    uint16_t rIndex;            /* R 值对应的查表索引 1..11（诊断） */
    uint16_t prRaw;             /* 未做 6 点中值前的脉率（诊断） */
    uint16_t medRR;             /* R-R 中位数（诊断） */
    uint16_t peakCnt;           /* 累计确认峰数（诊断） */
    float    fundHz;            /* 功率谱带内主频 Hz（≈脉率/60，交叉检查用） */
    uint16_t adjTickCnt;        /* 已执行的光强调节节拍数（诊断） */
} spo2_result_t;

typedef struct {
    spo2_filt_t   filt;     /* 滤波链 */
    spo2_pulse_t  pulse;    /* 脉率检测 */
    spo2_calc_t   calc;     /* 屏幕窗 + R 值 + 血氧（约 8KB） */
    spo2_light_t  light;    /* 光强调节 + 脱落判决 */
    spo2_result_t out;      /* 最新结果快照 */
    uint16_t      adjDiv;   /* 光强周期计数的分频因子 */
    uint8_t       screenPending;  /* 1=屏已满，屏级重算(屏统计+R+PI)待主循环空闲时执行 */
    uint32_t      screenLostCnt;  /* 重算未及时取走又被下一屏覆写而丢掉的屏数(应为 0) */
    uint8_t       piBadCnt; /* PI 连续不达标的屏数（手指脱落二级判据的防抖）*/
} spo2_alg_t;

void                  spo2_alg_init(spo2_alg_t *a);
void                  spo2_alg_step(spo2_alg_t *a, uint16_t irAd, uint16_t redAd);
void                  spo2_alg_reset(spo2_alg_t *a);   /* 全算法复位（对应 ResetAdj + 清屏） */
const spo2_result_t  *spo2_alg_get_result(const spo2_alg_t *a);
/* 上位机下发调光命令时调用：交出光强控制权（下一次复位交还自动） */
void                  spo2_alg_set_manual_light(spo2_alg_t *a, uint16_t irDa, uint16_t redDa);
/* R 值现场标定（验收"血氧误差 ±2%"的调节手段）；返回 1-成功 0-参数越界 */
uint8_t               spo2_alg_set_r_cal(spo2_alg_t *a, int16_t gainPm, int16_t offset);
void                  spo2_alg_get_r_cal(const spo2_alg_t *a, int16_t *pGainPm, int16_t *pOffset);
/* 读取屏缓冲里的第 idx 对原始码值（验收要求的"至少 1000 点真实 AD 数据"导出用） */
uint8_t               spo2_alg_get_screen_point(const spo2_alg_t *a, uint16_t idx,
                                                uint16_t *pIrAd, uint16_t *pRedAd);

/* ---- 屏级重算挪出 2ms 采样拍(2026-10-09 新增)----
 *   spo2_alg_step()        满屏时只置 pending 标志，不再在拍内跑屏统计；
 *   spo2_alg_screen_run()  由主循环空闲时取走执行(见 app/Main/Main.c 的 ProcSPO2HeavyTask)。
 * 理由：屏统计(2048 点)在 168MHz 下要数百 us 级，留在拍内会把"IR 采完到 RED 采完"
 * 之间拉开好几毫秒，两个通道不再成对同时刻，R 值与 PI 的口径直接被污染。 */
uint8_t               spo2_alg_screen_pending(const spo2_alg_t *a);
void                  spo2_alg_screen_run(spo2_alg_t *a);
uint32_t              spo2_alg_get_screen_lost_cnt(const spo2_alg_t *a);
#endif /* SPO2_ALG_H */
