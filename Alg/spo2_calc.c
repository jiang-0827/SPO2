/*********************************************************************************************************
:* 模 块 名 称：spo2_calc.c
:* 摘        要：屏幕窗统计 + R 值（比率法）+ 血氧饱和度查表
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：算法说明与使用方式见 spo2_calc.h 头注释；数值链路严格照搬 CalcSPO2.m。
*********************************************************************************************************/

#include "spo2_calc.h"

/*********************************************************************************************************
:*                                             内部函数声明
*********************************************************************************************************/
static void  CalcScreenStatic(spo2_calc_t *c);   /* 计算一屏的 max/min/rng/avg + PI */
static float MedianRBuf(const spo2_calc_t *c);   /* gRValBuff 取中间两个的平均 */
static void  UpdateSPO2(spo2_calc_t *c, float rRaw);  /* R 值标定 + 查表得血氧 */
static uint16_t CalcPiX100(uint16_t rng, uint16_t avg); /* 灌注指数(×100，整数) */

/*********************************************************************************************************
* 函数名称：spo2_calc_init
* 功能功能：初始化屏幕窗与 R 值缓冲
* 输入参数：c-计算器
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：gRValBuff 按 MATLAB InitData() 的做法填入 SPO2_R_SEED(560)，
*           保证上电初期的数值行为与 MATLAB 完全一致
*********************************************************************************************************/
void spo2_calc_init(spo2_calc_t *c)
{
    uint16_t i;

    for(i = 0; i < SPO2_SCREEN_LEN; i++)
    {
        c->irBuf[i]  = 0;
        c->redBuf[i] = 0;
    }

    c->pos        = 0;
    c->screenFull = 0;
    c->sampleCnt  = 0;
    c->screenCnt  = 0;

    c->irMax  = 0;
    c->irMin  = (uint16_t)SPO2_ADC_FULL;
    c->redMax = 0;
    c->redMin = (uint16_t)SPO2_ADC_FULL;
    c->irRng  = 0;
    c->redRng = 0;
    c->irAvg  = 0;
    c->redAvg = 0;

    c->piIrX100  = 0;
    c->piRedX100 = 0;

    c->rPtr   = 0;
    c->rVal   = (float)SPO2_R_SEED;
    c->rRaw   = (float)SPO2_R_SEED;
    c->rIndex = 1;
    c->spo2   = (int16_t)SPO2_INVALID_DATA;

    /* R 值标定参数默认直通（gain=1000, offset=0），与 MATLAB 数值行为完全一致 */
    c->rCalGainPm = (int16_t)SPO2_R_CAL_GAIN_DEF;
    c->rCalOffset = (int16_t)SPO2_R_CAL_OFFSET_DEF;

    for(i = 0; i < SPO2_R_BUF_LEN; i++)
    {
        c->rBuf[i] = (float)SPO2_R_SEED;   /* 复刻 InitData() 的 gRValBuff = 6 个 560 */
    }
}

/*********************************************************************************************************
* 函数名称：spo2_calc_reset
* 功能功能：复位 R 值缓冲与统计量(对应 MATLAB ResetAdj())
* 输入参数：c-计算器
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：只复位"标定相关"的状态；屏幕缓冲不清，避免波形显示出现断点
*********************************************************************************************************/
void spo2_calc_reset(spo2_calc_t *c)
{
    uint16_t i;

    c->rPtr   = 0;
    c->rVal   = (float)SPO2_R_SEED;
    c->rRaw   = (float)SPO2_R_SEED;
    c->rIndex = 1;
    c->spo2   = (int16_t)SPO2_INVALID_DATA;

    for(i = 0; i < SPO2_R_BUF_LEN; i++)
    {
        c->rBuf[i] = (float)SPO2_R_SEED;
    }
}

/*********************************************************************************************************
* 函数名称：spo2_calc_set_invalid
* 功能功能：把血氧结果置为无效值(手指脱落 / 粗调未完成时调用)
* 输入参数：c-计算器
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：对应上位机"血氧显示 ---"的行为
*********************************************************************************************************/
void spo2_calc_set_invalid(spo2_calc_t *c)
{
    c->spo2 = (int16_t)SPO2_INVALID_DATA;
}

/*********************************************************************************************************
* 函数名称：spo2_calc_push
* 功能功能：把一个 IR/RED 采样点写入屏幕缓冲
* 输入参数：c-计算器; irAd-红外光码值; redAd-红光码值
* 输出参数：void
* 返 回 值：1-刚好写满一屏(调用者应随即调用 spo2_calc_screen_run); 0-未满
* 生成日期：2026年09月24日
* 注    意：对应 RealAnalyzeData.m 里"接收满一屏数据"的判据
*********************************************************************************************************/
uint8_t spo2_calc_push(spo2_calc_t *c, uint16_t irAd, uint16_t redAd)
{
    uint8_t full = 0;

    c->irBuf[c->pos]  = irAd;
    c->redBuf[c->pos] = redAd;

    c->pos++;
    c->sampleCnt++;

    if(c->pos >= (uint16_t)SPO2_SCREEN_LEN)
    {
        c->pos        = 0;
        c->screenFull = 1;
        c->screenCnt++;
        full          = 1;
    }

    return full;
}

/*********************************************************************************************************
* 函数名称：CalcScreenStatic
* 功能功能：计算一屏原始码值的 max/min/峰峰值/中点
* 输入参数：c-计算器
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：对应 RealAnalyzeData.m 里的 irMax/irMin/redMax/redMin 与
*           gIRADAvg = (irMax+irMin)/2 这一组量
*********************************************************************************************************/
static void CalcScreenStatic(spo2_calc_t *c)
{
    uint16_t i;
    uint16_t v;
    uint16_t irMax = 0;
    uint16_t irMin = (uint16_t)SPO2_ADC_FULL;
    uint16_t rdMax = 0;
    uint16_t rdMin = (uint16_t)SPO2_ADC_FULL;

    for(i = 0; i < (uint16_t)SPO2_SCREEN_LEN; i++)
    {
        v = c->irBuf[i];
        if(v > irMax) { irMax = v; }
        if(v < irMin) { irMin = v; }

        v = c->redBuf[i];
        if(v > rdMax) { rdMax = v; }
        if(v < rdMin) { rdMin = v; }
    }

    c->irMax  = irMax;
    c->irMin  = irMin;
    c->redMax = rdMax;
    c->redMin = rdMin;

    c->irRng  = (uint16_t)(irMax - irMin);
    c->redRng = (uint16_t)(rdMax - rdMin);

    /* MATLAB: (irMax + irMin)/2 —— 整数除法在 MATLAB 里是浮点除法(可能带 .5)，
     * 这里的量只用于"与中线比较"，四舍五入即可，故 +1 再整除 */
    c->irAvg  = (uint16_t)(((uint32_t)irMax + (uint32_t)irMin + 1u) / 2u);
    c->redAvg = (uint16_t)(((uint32_t)rdMax + (uint32_t)rdMin + 1u) / 2u);

    /* 灌注指数：直流用同一个"屏中点"，与光强调节的口径完全一致（都以 irAvg 为准） */
    c->piIrX100  = CalcPiX100(c->irRng,  c->irAvg);
    c->piRedX100 = CalcPiX100(c->redRng, c->redAvg);
}

/*********************************************************************************************************
:* 函数名称：CalcPiX100
:* 功能功能：灌注指数 PI = 交流幅度 / 直流 × 100%，结果放大 100 倍存成整数
:* 输入参数：rng-屏峰峰值(码值); avg-屏中点(码值, 即直流)
:* 输出参数：void
:* 返 回 值：PI×100（3.00% → 300），已按 SPO2_PI_MAX_X100 封顶
:* 生成日期：2026年09月24日
:* 注    意：推导：PI = (峰峰值/2)/直流 × 100 = 峰峰值 × 50 / 直流；
:*           再乘 SPO2_PI_SCALE(100) → PI×100 = 峰峰值 × 5000 / 直流。
:*           12bit 下峰峰值 ≤ 4095，4095×5000 = 2.05e7 < 2^32，uint32 不会溢出。
:*           交流幅度取"峰峰值的一半"而不是"均方根×sqrt(2)"：前者对单周期
:*           完整波形才是严格的正弦幅值，但本工程的屏窗是整数个心跳、
:*           且 MATLAB 侧的 R 值也用峰峰值口径，两者保持一致便于交叉核对。
*********************************************************************************************************/
static uint16_t CalcPiX100(uint16_t rng, uint16_t avg)
{
    uint32_t v;

    if(avg == 0u)
    {
        return 0u;                  /* 全暗/全饱和：直流为 0，PI 无定义 */
    }

    v = (((uint32_t)rng * 50u) * (uint32_t)SPO2_PI_SCALE) / (uint32_t)avg;

    if(v > (uint32_t)SPO2_PI_MAX_X100)
    {
        v = (uint32_t)SPO2_PI_MAX_X100;   /* 封顶：非脉搏波(如方波干扰)会把 PI 撑到离谱 */
    }

    return (uint16_t)v;
}

/*********************************************************************************************************
* 函数名称：MedianRBuf
* 功能功能：gRValBuff 排序后取中间两个的平均（对应 rVal = (seq(3)+seq(4))/2）
* 输入参数：c-计算器
* 输出参数：void
* 返 回 值：R 值中位数
* 生成日期：2026年09月24日
* 注    意：MATLAB 侧 R 值本身是浮点(redADRng*1000/irADRng 不一定整除)，
*           故这里也用 float，保证与 RR_TABLE 的比较结果一致
*********************************************************************************************************/
static float MedianRBuf(const spo2_calc_t *c)
{
    float   tmp[SPO2_R_BUF_LEN];
    uint8_t i, j, n;
    float   key;

    n = (uint8_t)SPO2_R_BUF_LEN;
    for(i = 0; i < n; i++)
    {
        tmp[i] = c->rBuf[i];
    }

    for(i = 1; i < n; i++)
    {
        key = tmp[i];
        j   = i;
        while((j > 0u) && (tmp[j - 1u] > key))
        {
            tmp[j] = tmp[j - 1u];
            j--;
        }
        tmp[j] = key;
    }

    /* 6 个元素 → 排序后第 3、第 4 个(1 基)，即 0 基的 [2]、[3] */
    return (tmp[n / 2u - 1u] + tmp[n / 2u]) * 0.5f;
}

/*********************************************************************************************************
* 函数名称：UpdateSPO2
* 功能功能：R 值查表得到血氧饱和度
* 输入参数：c-计算器; rVal-R 值
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：逐字照抄 CalcSPO2.m:
*             index = 1; while rVal >= RR_TABLE(index) && index < 11: index++
*             spo2 = 101 - index;  if spo2 == 100: spo2 = 99
*********************************************************************************************************/
static void UpdateSPO2(spo2_calc_t *c, float rRaw)
{
    uint16_t index = 1u;
    int16_t  spo2;
    float    rVal;

    c->rRaw = rRaw;

    /* ---- R 值现场标定（验收"血氧误差 ±2%"的调节旋钮）----
     *   R_used = R_raw * gain/1000 + offset
     * 默认 gain=1000/offset=0 时是严格直通，数值与 MATLAB 逐位一致；
     * 标定时只动 gain/offset，不动 RR_TABLE，标定记录可复现。*/
    rVal = rRaw * ((float)c->rCalGainPm * 0.001f) + (float)c->rCalOffset;

    if(rVal < 0.0f)
    {
        rVal = 0.0f;
    }
    c->rVal = rVal;

    while((rVal >= (float)SPO2_RR_TABLE[index - 1u]) && (index < (uint16_t)SPO2_R_TABLE_LEN))
    {
        index++;
    }
    c->rIndex = index;

    spo2 = (int16_t)((int16_t)SPO2_R_INDEX_BASE - (int16_t)index);

    if(spo2 == 100)
    {
        spo2 = (int16_t)SPO2_SAT_MAX;   /* 与 MATLAB 一致：100 按 99 上报 */
    }
    c->spo2 = spo2;
}

/*********************************************************************************************************
* 函数名称：spo2_calc_screen_run
* 功能功能：满一屏时执行一次统计与血氧计算
* 输入参数：c-计算器; allowR-是否允许把本屏的 R 值滑入缓冲(= CheckRoughAdj())
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：对应 RealAnalyzeData.m 里 gXCntAxes1 > MAX_X_CNT 分支：
*           (1) 先算屏统计(gIRADAvg/gRedADAvg 供光强调节用) —— 这一步无条件做
*           (2) 再算 R 值并查表得血氧 —— MATLAB 把这一步放在
*               "if CheckRoughAdj() == 1" 里面，所以标定未完成时缓冲不攒数据。
*               C 侧用 allowR 保持同一门控。
*           另外 MATLAB 里 "if irADRng > 0" 才把新 R 值滑入缓冲；irADRng == 0
*           时缓冲不动、但仍用旧缓冲算出 rVal —— 这里保持同样的行为
*********************************************************************************************************/
void spo2_calc_screen_run(spo2_calc_t *c, uint8_t allowR)
{
    float   r;
    uint8_t rOk = 0u;

    CalcScreenStatic(c);        /* 屏统计量：光强调节(JudgeReAdj/FineAdj)要用，必须每屏更新 */

    if(allowR == 0u)
    {
        return;                 /* 未标定：只更新统计量，不碰 R 值缓冲与血氧 */
    }

    if(c->irRng > 0u)
    {
#if (SPO2_R_DC_NORM != 0u)
        /* 标准定义式 R = (redAC/redDC)/(irAC/irDC)
         * 分子分母同用峰峰值（各含一个 ×2，比值不变），直流用屏中点。
         * 只有把 RR_TABLE 一并重新整定过之后才允许编译到这一支。*/
        if((c->redAvg > 0u) && (c->irAvg > 0u))
        {
            r = ((float)c->redRng / (float)c->redAvg)
              / ((float)c->irRng  / (float)c->irAvg)
              * (float)SPO2_R_RATIO_SCALE;
            rOk = 1u;
        }
#else
        /* MATLAB 原式 R = redADRng * 1000 / irADRng（RR_TABLE 就是按它整定的）*/
        r   = ((float)c->redRng * (float)SPO2_R_RATIO_SCALE) / (float)c->irRng;
        rOk = 1u;
#endif
    }

    if(rOk != 0u)
    {
        c->rBuf[c->rPtr] = r;
        c->rPtr++;
        if(c->rPtr >= (uint8_t)SPO2_R_BUF_LEN)
        {
            c->rPtr = 0u;
        }
    }

    UpdateSPO2(c, MedianRBuf(c));
}

/*********************************************************************************************************
:* 函数名称：spo2_calc_set_r_cal / spo2_calc_get_r_cal
:* 功能功能：读写 R 值现场标定参数（验收"血氧误差 ±2%"的调节手段）
:* 输入参数：c-计算器; gainPm-千分比增益(500~1500); offset-绝对偏置(-300~300)
:* 输出参数：pGainPm, pOffset
:* 返 回 值：1-参数合法已生效; 0-越界被拒
:* 生成日期：2026年09月24日
:* 注    意：标定的是 R 值而不是血氧值，好处是"改一处、全曲线一起动"，且标定后
:*           每一次查表走的还是同一条 RR_TABLE，可复现、可与 MATLAB 对拍。
:*           标定完成后请用 CMD_SPO2_R_CAL 读回并存进实验报告的标定记录。
*********************************************************************************************************/
uint8_t spo2_calc_set_r_cal(spo2_calc_t *c, int16_t gainPm, int16_t offset)
{
    if((gainPm < (int16_t)SPO2_R_CAL_GAIN_MIN) || (gainPm > (int16_t)SPO2_R_CAL_GAIN_MAX))
    {
        return 0u;
    }
    if((offset < (int16_t)SPO2_R_CAL_OFF_MIN) || (offset > (int16_t)SPO2_R_CAL_OFF_MAX))
    {
        return 0u;
    }

    c->rCalGainPm = gainPm;
    c->rCalOffset = offset;

    return 1u;
}

void spo2_calc_get_r_cal(const spo2_calc_t *c, int16_t *pGainPm, int16_t *pOffset)
{
    if(pGainPm != 0) { *pGainPm = c->rCalGainPm; }
    if(pOffset != 0) { *pOffset = c->rCalOffset; }
}
