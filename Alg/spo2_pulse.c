/*********************************************************************************************************
:* 模 块 名 称：spo2_pulse.c
:* 摘        要：逐点流式脉率检测（峰定位 + 中位数RR → bpm + 6点中值滤波）
:* 当前版本：1.0.0
:* 作        者：Jaziel.Jiang(COPYRIGHT 2018 - 2026 Jaziel.Jiang. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：算法说明与使用方式见 spo2_pulse.h 头注释；数值后端链路严格照搬
:*               CalcPulseRate.m（median(RR) → 30000/medRR → 6 点中值 → 越界置无效）。
*********************************************************************************************************/

#include "spo2_pulse.h"

/*********************************************************************************************************
:*                                             内部函数声明
*********************************************************************************************************/
static void    SubmitBlock(spo2_pulse_t *h);            /* 块提交: 刷新窗极值与信号有效性 */
static float   CalcThreshold(const spo2_pulse_t *h);    /* 自适应门限 */
static void    PushRR(spo2_pulse_t *h, uint16_t rr);    /* 记录 RR 并更新脉率 */
static float   MedianRR(const spo2_pulse_t *h);         /* RR 环形缓冲中位数(可能带 .5) */
static int16_t GprBufMedian(const spo2_pulse_t *h);     /* gPRBuf 6 点中值(中间两个平均后四舍五入) */

/*********************************************************************************************************
* 函数名称：spo2_pulse_init
* 功能功能：初始化脉率检测器(全部状态清零)
* 输入参数：h-检测器
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：gPRBuf 按 MATLAB InitData() 的做法填入 SPO2_PR_SEED(75)，
*           这样上电初期输出与 MATLAB 完全一致(75 → 在合法范围内直接显示)
*********************************************************************************************************/
void spo2_pulse_init(spo2_pulse_t *h)
{
    uint16_t i;

    h->blkPos     = 0;
    h->blkMax     = -1.0e9f;   /* 哨兵: 首块从最小开始累积 */
    h->blkMin     =  1.0e9f;
    h->blkPtr     = 0;
    h->blkValid   = 0;
    h->winMax     = 0.0f;
    h->winMin     = 0.0f;
    h->sigValid   = 0;

    h->peakActive = 0;
    h->peakVal    = 0.0f;
    h->peakPos    = 0;
    h->fallCnt    = 0;
    h->refractory = 0;

    h->sampleIdx   = 0;
    h->lastPeakIdx = 0;
    h->hasLastPeak = 0;
    h->rrCnt       = 0;
    h->rrPtr       = 0;
    h->medRR       = 0;
    h->peakCount   = 0;

    h->prPtr  = 0;
    h->prRaw  = SPO2_PR_SEED;
    h->pr     = SPO2_PR_SEED;

    for(i = 0; i < SPO2_PR_BLK_NUM; i++)
    {
        h->blkMaxArr[i] = 0.0f;
        h->blkMinArr[i] = 0.0f;
    }
    for(i = 0; i < SPO2_PR_MAX_RR; i++)
    {
        h->rrnArr[i] = 0;
    }
    for(i = 0; i < SPO2_PR_BUF_LEN; i++)
    {
        h->prBuf[i] = SPO2_PR_SEED;   /* 复刻 InitData() 的 gPRBuf = 6 个 75 */
    }
}

/*********************************************************************************************************
* 函数名称：spo2_pulse_reset
* 功能功能：复位脉率缓冲(对应 MATLAB ResetAdj())
* 输入参数：h-检测器
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：对应 ResetAdj.m 里 gPRBuf 复位为 6 个 75 的动作；RR 历史一并清掉
*********************************************************************************************************/
void spo2_pulse_reset(spo2_pulse_t *h)
{
    uint16_t i;

    h->rrCnt       = 0;
    h->rrPtr       = 0;
    h->medRR       = 0;
    h->hasLastPeak = 0;
    h->peakActive  = 0;
    h->fallCnt     = 0;
    h->refractory  = 0;
    h->prRaw       = SPO2_PR_SEED;
    h->pr          = SPO2_PR_SEED;

    for(i = 0; i < SPO2_PR_MAX_RR; i++)
    {
        h->rrnArr[i] = 0;
    }
    for(i = 0; i < SPO2_PR_BUF_LEN; i++)
    {
        h->prBuf[i] = SPO2_PR_SEED;
    }
}

/*********************************************************************************************************
* 函数名称：SubmitBlock
* 功能功能：提交当前块极值, 刷新窗 max/min 与信号有效性判定
* 输入参数：h-检测器
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
*********************************************************************************************************/
static void SubmitBlock(spo2_pulse_t *h)
{
    uint8_t i, n;
    float   mn, mx;

    h->blkMaxArr[h->blkPtr] = h->blkMax;
    h->blkMinArr[h->blkPtr] = h->blkMin;
    h->blkPtr++;
    if(h->blkPtr >= SPO2_PR_BLK_NUM)
    {
        h->blkPtr = 0;
    }
    if(h->blkValid < SPO2_PR_BLK_NUM)
    {
        h->blkValid++;
    }

    h->blkPos = 0;
    h->blkMax = -1.0e9f;       /* 新块重新累积 */
    h->blkMin =  1.0e9f;

    /* 刷新窗极值 */
    n  = h->blkValid;
    mx = h->blkMaxArr[0];
    mn = h->blkMinArr[0];
    for(i = 1; i < n; i++)
    {
        if(h->blkMaxArr[i] > mx) { mx = h->blkMaxArr[i]; }
        if(h->blkMinArr[i] < mn) { mn = h->blkMinArr[i]; }
    }
    h->winMax = mx;
    h->winMin = mn;

    /* 信号有效性: 已积累半个窗以上, 且窗峰峰值达到 SPO2_PR_ABS_MIN */
    if((h->blkValid >= (SPO2_PR_BLK_NUM / 2u)) &&
       ((mx - mn) >= (float)SPO2_PR_ABS_MIN))
    {
        h->sigValid = 1;
    }
    else
    {
        h->sigValid = 0;
    }
}

/*********************************************************************************************************
* 函数名称：CalcThreshold
* 功能功能：自适应门限 thr = min + alpha*(max-min)
* 输入参数：h-检测器
* 输出参数：void
* 返 回 值：门限值
* 生成日期：2026年09月24日
*********************************************************************************************************/
static float CalcThreshold(const spo2_pulse_t *h)
{
    return h->winMin + SPO2_PR_THR_ALPHA * (h->winMax - h->winMin);
}

/*********************************************************************************************************
* 函数名称：MedianRR
* 功能功能：RR 环形缓冲取中位数(拷贝后插入排序, n<=8)
* 输入参数：h-检测器
* 输出参数：void
* 返 回 值：R-R 中位数(样本数; 偶数个时可能带 0.5)
* 生成日期：2026年09月24日
* 注    意：MATLAB 的 median() 对偶数个元素取中间两个的平均, 可能是 x.5,
*           这里同样用 float 返回, 保证 30000/medRR 的截断结果与 MATLAB 一致
*********************************************************************************************************/
static float MedianRR(const spo2_pulse_t *h)
{
    uint16_t tmp[SPO2_PR_MAX_RR];
    uint8_t  i, j, n;
    uint16_t key;

    n = h->rrCnt;
    for(i = 0; i < n; i++)
    {
        tmp[i] = h->rrnArr[i];   /* 环形指针不影响中位数(排序后无差异) */
    }

    for(i = 1; i < n; i++)       /* 插入排序 */
    {
        key = tmp[i];
        j   = i;
        while((j > 0) && (tmp[j - 1] > key))
        {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = key;
    }

    if((n & 1u) != 0u)
    {
        return (float)tmp[n / 2u];
    }
    return ((float)tmp[n / 2u - 1u] + (float)tmp[n / 2u]) * 0.5f;
}

/*********************************************************************************************************
* 函数名称：GprBufMedian
* 功能功能：gPRBuf(6 点脉率缓冲)取中值 → 对应 MATLAB
*           pulseRate = round((seq(3)+seq(4))/2)
* 输入参数：h-检测器
* 输出参数：void
* 返 回 值：中值脉率
* 生成日期：2026年09月24日
* 注    意：MATLAB 的 round() 是"四舍五入、.5 远离零", 对正数等价于 (a+b+1)/2 取整
*********************************************************************************************************/
static int16_t GprBufMedian(const spo2_pulse_t *h)
{
    int16_t tmp[SPO2_PR_BUF_LEN];
    uint8_t i, j, n;
    int16_t key;

    n = SPO2_PR_BUF_LEN;
    for(i = 0; i < n; i++)
    {
        tmp[i] = h->prBuf[i];
    }

    for(i = 1; i < n; i++)
    {
        key = tmp[i];
        j   = i;
        while((j > 0) && (tmp[j - 1] > key))
        {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = key;
    }

    /* 6 个元素 → 取排序后第 3、第 4 个(1 基)求平均 */
    return (int16_t)(((int32_t)tmp[n / 2u - 1u] + (int32_t)tmp[n / 2u] + 1) / 2);
}

/*********************************************************************************************************
* 函数名称：PushRR
* 功能功能：记录一个 R-R 间期, 达到输出条件后更新脉率
* 输入参数：h-检测器; rr-R-R 间期(样本数)
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：对应 MATLAB:
*             med = median(RR);  pr = int16(30000/med);
*             gPRBuf 滑入 pr → sort → 取中间两个平均 → 越界置 -100
*********************************************************************************************************/
static void PushRR(spo2_pulse_t *h, uint16_t rr)
{
    float   medRR;
    int16_t prRaw;
    int16_t prMed;

    h->rrnArr[h->rrPtr] = rr;
    h->rrPtr++;
    if(h->rrPtr >= SPO2_PR_MAX_RR)
    {
        h->rrPtr = 0;
    }
    if(h->rrCnt < SPO2_PR_MAX_RR)
    {
        h->rrCnt++;
    }

    if(h->rrCnt < SPO2_PR_MIN_RR_OUT)
    {
        return;                 /* 样本太少, 暂不输出 */
    }

    medRR = MedianRR(h);
    if(medRR <= 0.0f)
    {
        return;
    }
    h->medRR = (uint16_t)(medRR + 0.5f);    /* 仅供诊断显示 */

    /* pr = int16(30000/medianY)：30 * Fs(=500) = 30000，C 的强制转换与
     * MATLAB int16() 一样是"向零截断"，故数值逐位一致 */
    prRaw = (int16_t)((60.0f * SPO2_FS) / medRR);
    h->prRaw = prRaw;

    /* gPRBuf 滑入 */
    h->prBuf[h->prPtr] = prRaw;
    h->prPtr++;
    if(h->prPtr >= SPO2_PR_BUF_LEN)
    {
        h->prPtr = 0;
    }

    prMed = GprBufMedian(h);

    if((prMed < (int16_t)SPO2_PR_MIN) || (prMed > (int16_t)SPO2_PR_MAX))
    {
        h->pr = (int16_t)SPO2_INVALID_DATA;   /* 越界: 上报无效值 */
    }
    else
    {
        h->pr = prMed;
    }
}

/*********************************************************************************************************
* 函数名称：spo2_pulse_step
* 功能功能：逐样本脉率检测(500Hz 实时调用)
* 输入参数：h-检测器; x-滤波后的样本值(交流域, 单位与门限一致)
* 输出参数：void
* 返 回 值：当前脉率(bpm); SPO2_INVALID_DATA = 无效(信号丢失/样本不足/超范围)
* 生成日期：2026年09月24日
*********************************************************************************************************/
int16_t spo2_pulse_step(spo2_pulse_t *h, float x)
{
    uint32_t diff;

    h->sampleIdx++;

    /* ---- 1. 滑窗分块极值维护 ---- */
    if(x > h->blkMax) { h->blkMax = x; }
    if(x < h->blkMin) { h->blkMin = x; }
    h->blkPos++;
    if(h->blkPos >= (uint16_t)SPO2_PR_BLK_LEN)
    {
        SubmitBlock(h);
    }

    /* ---- 2. 不应期倒计时(重搏波/噪声一律忽略) ---- */
    if(h->refractory > 0u)
    {
        h->refractory--;
        return h->pr;
    }

    /* ---- 3. 信号丢失时清零历史, 防止旧 RR 污染 ---- */
    if(h->sigValid == 0u)
    {
        h->peakActive = 0;
        h->fallCnt    = 0;
        if(h->hasLastPeak != 0u)
        {
            h->hasLastPeak = 0;
            h->rrCnt       = 0;
            h->rrPtr       = 0;
        }
        return h->pr;
    }

    /* ---- 4. 峰检测状态机 ---- */
    if(h->peakActive == 0u)
    {
        /* 等待过阈样本, 开始跟踪候选峰 */
        if(x > CalcThreshold(h))
        {
            h->peakActive = 1u;
            h->peakVal    = x;
            h->peakPos    = h->sampleIdx;
            h->fallCnt    = 0;
        }
        return h->pr;
    }

    /* 跟踪中 */
    if(x >= h->peakVal)
    {
        h->peakVal = x;             /* 仍在攀升: 刷新峰值与位置 */
        h->peakPos = h->sampleIdx;
        h->fallCnt = 0;
        return h->pr;
    }

    h->fallCnt++;
    if(h->fallCnt < (uint16_t)SPO2_PR_CONFIRM_FALL)
    {
        return h->pr;               /* 尚未确认(给信号一点回落时间) */
    }

    /* ---- 5. 确认一个脉搏波主峰 ---- */
    /* 不应期从"峰位置"起算: 扣除本次确认已消耗的回落样本数, 否则高频脉率
     * (如120bpm, RR=250)会被超长不应期周期性遮挡 */
    h->refractory = ((uint16_t)SPO2_PR_MIN_DIST > h->fallCnt)
                    ? (uint16_t)((uint16_t)SPO2_PR_MIN_DIST - h->fallCnt)
                    : 0u;
    h->peakActive = 0u;
    h->fallCnt    = 0u;
    h->peakCount++;

    if(h->hasLastPeak != 0u)
    {
        diff = h->peakPos - h->lastPeakIdx;
        if((diff >= (uint32_t)SPO2_PR_MIN_DIST) && (diff <= SPO2_PR_RR_MAX))
        {
            PushRR(h, (uint16_t)diff);
        }
    }
    h->lastPeakIdx = h->peakPos;
    h->hasLastPeak = 1u;

    return h->pr;
}
