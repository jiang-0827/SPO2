/*********************************************************************************************************
:* 模 块 名 称：spo2_light.c
:* 摘        要：光强自动调节 + 手指脱落判决状态机
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：每个函数与 MATLAB 的同名 .m 文件逐行对应；与原型的两处刻意差异已在下面
:*               各自的注释里标出（都是为了 C 侧的安全/边界，不改变控制律）：
:*                 1) DA 值加了 [MIN, MAX] 钳位 —— MATLAB 的 FineAdj 做 ±1 时不钳位，落到
:*                    C 的 uint16 上会下溢成 65535，必须防；
:*                 2) SPO2_ADJ_EN = 0 时整套状态机短路（用于前端为交流耦合、无直流光强
:*                    信息时的降级运行）。
*********************************************************************************************************/

#include "spo2_light.h"

/*********************************************************************************************************
:*                                             内部函数声明
*********************************************************************************************************/
static void ResetAdj(spo2_light_t *h);
static uint16_t ClampIrDa(uint16_t da);
static uint16_t ClampRedDa(uint16_t da);
static uint8_t DetectProbeOff(spo2_light_t *h);   /* 探头脱落判别（新增）*/

/*********************************************************************************************************
:*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ClampIrDa / ClampRedDa
* 功能功能：把光强 DA 值钳到 [MIN, MAX] 区间
* 输入参数：da-待钳位的 DA 值
* 输出参数：void
* 返 回 值：钳位后的 DA 值
* 生成日期：2026年09月24日
* 注    意：对应"刻意差异 1"：MATLAB 的 FineAdj 不钳位，C 侧必须钳，
*           否则 uint16 会下溢
*********************************************************************************************************/
static uint16_t ClampIrDa(uint16_t da)
{
    if(da < (uint16_t)SPO2_MIN_IR_INT_DA_VAL) { return (uint16_t)SPO2_MIN_IR_INT_DA_VAL; }
    if(da > (uint16_t)SPO2_MAX_IR_INT_DA_VAL) { return (uint16_t)SPO2_MAX_IR_INT_DA_VAL; }
    return da;
}

static uint16_t ClampRedDa(uint16_t da)
{
    if(da < (uint16_t)SPO2_MIN_RED_INT_DA_VAL) { return (uint16_t)SPO2_MIN_RED_INT_DA_VAL; }
    if(da > (uint16_t)SPO2_MAX_RED_INT_DA_VAL) { return (uint16_t)SPO2_MAX_RED_INT_DA_VAL; }
    return da;
}

/*********************************************************************************************************
* 函数名称：ResetAdj
* 功能功能：复位调光参数，准备重新调光（逐行对应 ResetAdj.m）
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：MATLAB 里还顺带把 gRValBuff/gPRBuf 复位并 SendAdjCmd(MIN,MIN)；
*           C 侧前者由 resetFlag 通知上层，后者由"DA 值被置为 MIN"体现
*********************************************************************************************************/
static void ResetAdj(spo2_light_t *h)
{
    h->roughAdjIrFlag     = 0;
    h->roughAdjRedFlag    = 0;
    h->roughAdjIrCondCnt  = 0;
    h->roughAdjRedCondCnt = 0;
    h->firDetFinger       = 0;
    h->astableFingerCnt   = 0;
    h->stableFingerCnt    = 0;
    h->irOutRngCnt        = 0;
    h->redOutRngCnt       = 0;
    h->probeOffCnt        = 0;

    h->irIntDa  = (uint16_t)SPO2_MIN_IR_INT_DA_VAL;
    h->redIntDa = (uint16_t)SPO2_MIN_RED_INT_DA_VAL;

    h->manualMode = 0;  /* 交还自动调节（上位机接管后靠一次复位命令/重新插拔手指退出） */

    h->resetFlag = 1;   /* 通知上层：请复位 R 值缓冲 / 脉率缓冲 */
}

/*********************************************************************************************************
:* 函数名称：DetectProbeOff
:* 功能功能：探头脱落判别（验收项"自动判别手指脱落与探头脱落"的"探头脱落"一侧）
:* 输入参数：h-状态机
:* 输出参数：void
:* 返 回 值：1-已确认探头脱落（调用者应立即停止调光返回）; 0-未判出探头脱落
:* 生成日期：2026年09月24日
:* 注    意：判据是"两个通道的归一码值同时趴在满量程一侧"。
:*           归一后约定是"光越强 → 码值越小"（见 SPO2_FE_INVERT），所以：
:*             码值 ≈ 4095 → 光电管一点光都没收到 → 探头没插好 / 断线 / LED 开路
:*             码值 ≈ 0    → 光太强（LED 直射） → 探头在、只是没夹手指（属于手指脱落）
:*           两者是两个相反的极端，因此**不会互相误判**，这就是"能区分两种脱落"的全部依据。
:*           确认后把光强 DA 压到最小值：既不烧 LED，也避免"DA 到顶仍到不了中线"
:*           的粗调分支反复触发 ResetAdj 刷屏。
*********************************************************************************************************/
static uint8_t DetectProbeOff(spo2_light_t *h)
{
#if (SPO2_PROBE_OFF_EN == 0u)
    h->probeOn = 1u;
    return 0u;                  /* 关闭探头判别：退化为"只有一种脱落" */
#else
    if((h->irAd >= (uint16_t)SPO2_PROBE_OFF_CODE_MIN) &&
       (h->redAd >= (uint16_t)SPO2_PROBE_OFF_CODE_MIN))
    {
        if(h->probeOffCnt < 0xFFFFu)
        {
            h->probeOffCnt++;
        }

        if(h->probeOffCnt >= (uint16_t)SPO2_PROBE_OFF_CNT)
        {
            if(h->offState != (uint8_t)SPO2_OFF_PROBE)
            {
                ResetAdj(h);                        /* 状态切换：重新标定 + 通知上层复位缓冲 */
                h->offState = (uint8_t)SPO2_OFF_PROBE;
            }

            h->probeOn  = 0u;
            h->fingerOn = 0u;
            h->irIntDa  = (uint16_t)SPO2_MIN_IR_INT_DA_VAL;
            h->redIntDa = (uint16_t)SPO2_MIN_RED_INT_DA_VAL;

            return 1u;
        }

        return 0u;              /* 防抖期间：先维持原状态继续跑 */
    }

    /* 见到光了 → 撤销"无光"计数，恢复探头在线 */
    h->probeOffCnt = 0u;
    h->probeOn     = 1u;

    return 0u;
#endif
}

/*********************************************************************************************************
:*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：spo2_light_init
* 功能功能：初始化光强调节状态机(对应 InitData.m 里的相关初值)
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
*********************************************************************************************************/
void spo2_light_init(spo2_light_t *h)
{
    h->firDetFinger  = 0;
    h->fingerOn      = 0;

    h->astableFingerCnt = 0;
    h->stableFingerCnt  = 0;

    /* 上电默认"探头脱落"：还没收到任何光，先说探头没接好，
     * 等真的见到光路再升级为 NONE，避免开机瞬间误显示成"手指脱落" */
    h->probeOn          = 0;
    h->offState         = (uint8_t)SPO2_OFF_PROBE;
    h->probeOffCnt      = 0;

    h->roughAdjIrFlag   = 0;
    h->roughAdjRedFlag  = 0;
    h->roughAdjIrCondCnt  = 0;
    h->roughAdjRedCondCnt = 0;
    h->adjIrDelayCnt    = 0;
    h->adjRedDelayCnt   = 0;
    h->irOutRngCnt      = 0;
    h->redOutRngCnt     = 0;

    h->irIntDa  = (uint16_t)SPO2_MIN_IR_INT_DA_VAL;
    h->redIntDa = (uint16_t)SPO2_MIN_RED_INT_DA_VAL;

    h->irAd     = 0;
    h->redAd    = 0;
    h->irAdAvg  = 0;
    h->redAdAvg = 0;

    h->initAvgValid = 0;
    h->reAdjReq     = 0;
    h->resetFlag    = 0;
    h->manualMode   = 0;
}

/*********************************************************************************************************
* 函数名称：spo2_light_reset
* 功能功能：外部复位(等价于直接调用 MATLAB 的 ResetAdj())
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
*********************************************************************************************************/
void spo2_light_reset(spo2_light_t *h)
{
    ResetAdj(h);
}

/*********************************************************************************************************
* 函数名称：spo2_light_set_sample
* 功能功能：更新最近一次采样的 IR/RED 码值(对应 gIRADVal / gRedADVal)
* 输入参数：h-状态机; irAd-红外码值; redAd-红光码值
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：逐样本调用；RoughAdj 只读这两个值
*********************************************************************************************************/
void spo2_light_set_sample(spo2_light_t *h, uint16_t irAd, uint16_t redAd)
{
    h->irAd  = irAd;
    h->redAd = redAd;
}

/*********************************************************************************************************
* 函数名称：spo2_light_set_screen_avg
* 功能功能：更新一屏的均值(对应 gIRADAvg / gRedADAvg)
* 输入参数：h-状态机; irAvg-红外屏均值; redAvg-红光屏均值
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：在 spo2_calc_screen_run() 之后调用
*********************************************************************************************************/
void spo2_light_set_screen_avg(spo2_light_t *h, uint16_t irAvg, uint16_t redAvg)
{
    h->irAdAvg  = irAvg;
    h->redAdAvg = redAvg;

    h->initAvgValid = 1;
}

/*********************************************************************************************************
* 函数名称：spo2_light_rough_step
* 功能功能：粗调红外光与红光光强（逐行对应 RoughAdj.m）
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：调用节拍 = SPO2_ADJ_PERIOD_SMP 个样本一次（对应 RealDraw 的 0.1s）
*********************************************************************************************************/
void spo2_light_rough_step(spo2_light_t *h)
{
#if (SPO2_ADJ_EN == 0u)
    /* 自动调光关闭：不做脱落判决，直接认为"信号有效"，DA 保持初始值 */
    h->fingerOn = 1;
    h->probeOn  = 1;
    h->offState = (uint8_t)SPO2_OFF_NONE;
#else
    if(h->manualMode != 0u)
    {
        return;             /* 光强已被上位机命令接管，不再自动调 */
    }

    /* ---- 最高优先级：探头脱落（无光）—— 一旦确认就停止一切调光动作 ---- */
    if(DetectProbeOff(h) != 0u)
    {
        return;
    }

    if(h->firDetFinger == 1u)   /* 已经检测到手指接入 */
    {
        if((h->irAd < (uint16_t)SPO2_IR_OFF_MAX_VAL) ||
           (h->redAd < (uint16_t)SPO2_RED_OFF_MAX_VAL))
        {
            /* 疑似脱落：连续 MAX_CNT_ASTABLE_FINGER 次才确认。
             * 走到这里说明"光路是通的、码值趴在 0 一侧"（光过强），
             * 而不是"完全无光"，所以判为**手指脱落**而不是探头脱落。*/
            h->astableFingerCnt++;
            if(h->astableFingerCnt >= (uint16_t)SPO2_MAX_CNT_ASTABLE_FINGER)
            {
                ResetAdj(h);
                h->fingerOn = 0;
                h->offState = (uint8_t)SPO2_OFF_FINGER;
            }
        }
        else
        {
            uint16_t da;

            h->astableFingerCnt = 0;
            h->fingerOn         = 1;
            h->offState         = (uint8_t)SPO2_OFF_NONE;

            if(h->stableFingerCnt < (uint16_t)SPO2_MAX_CNT_STABLE_FINGER)
            {
                h->stableFingerCnt++;   /* 连续 N 次判接入才算真正接入 */
            }
            else
            {
                /* ---- 粗调红外光光强 ---- */
                if(h->roughAdjIrFlag == 0u)
                {
                    if(h->irAd < (uint16_t)SPO2_IR_CENTRAL_LINE)
                    {
                        /* 光够强，到位计数 */
                        h->roughAdjIrCondCnt++;
                        if(h->roughAdjIrCondCnt >= (uint16_t)SPO2_MAX_CNT_ROUGH_ADJ_IR_COND)
                        {
                            h->roughAdjIrFlag = 1;   /* 红外粗调完成 */
                        }
                    }
                    else
                    {
                        h->adjIrDelayCnt++;
                        if(h->adjIrDelayCnt >= (uint16_t)SPO2_MAX_CNT_ADJ_IR_DELAY)
                        {
                            h->adjIrDelayCnt = 0;
                            if(h->irIntDa < (uint16_t)SPO2_MAX_IR_INT_DA_VAL)
                            {
                                da = (uint16_t)(h->irIntDa + (uint16_t)SPO2_ROUGH_ADJ_IR_STEP);
                                h->irIntDa = ClampIrDa(da);   /* 原 MATLAB 是 SendAdjCmd(da, red) */
                            }
                            else
                            {
                                /* DA 已到顶仍到不了中线 → 复位重来 */
                                ResetAdj(h);
                            }
                            h->roughAdjIrCondCnt = 0;
                        }
                    }
                }

                /* ---- 粗调红光光强 ---- */
                if(h->roughAdjRedFlag == 0u)
                {
                    if(h->redAd < (uint16_t)SPO2_RED_CENTRAL_LINE)
                    {
                        h->roughAdjRedCondCnt++;
                        if(h->roughAdjRedCondCnt >= (uint16_t)SPO2_MAX_CNT_ROUGH_ADJ_RED_COND)
                        {
                            h->roughAdjRedFlag = 1;   /* 红光粗调完成 */
                        }
                    }
                    else
                    {
                        h->adjRedDelayCnt++;
                        if(h->adjRedDelayCnt >= (uint16_t)SPO2_MAX_CNT_ADJ_RED_DELAY)
                        {
                            h->adjRedDelayCnt = 0;
                            if(h->redIntDa < (uint16_t)SPO2_MAX_RED_INT_DA_VAL)
                            {
                                da = (uint16_t)(h->redIntDa + (uint16_t)SPO2_ROUGH_ADJ_RED_STEP);
                                h->redIntDa = ClampRedDa(da);
                            }
                            else
                            {
                                ResetAdj(h);
                            }
                            h->roughAdjRedCondCnt = 0;
                        }
                    }
                }
            }
        }
    }
    else    /* 还没检测到手指接入 */
    {
        if((h->irAd > (uint16_t)SPO2_IR_OFF_MAX_VAL) ||
           (h->redAd > (uint16_t)SPO2_RED_OFF_MAX_VAL))
        {
            h->stableFingerCnt = 0;
            h->firDetFinger    = 1;     /* 第一次检测到"光路通且未饱和" */
            h->offState        = (uint8_t)SPO2_OFF_NONE;
        }
        else
        {
            /* 码值趴在 0 一侧（光过强/饱和）：说明 LED 在发光、探头是插着的，
             * 只是没有手指挡光 → 明确判为**手指脱落**。
             * 注意"探头脱落"已经在上面 DetectProbeOff() 里被排除掉了
             * （那种情况码值趴在 4095 一侧），所以这里不会再误判。
             * 少了这一段，"探头插着但不夹手指"会残留成开机默认的 PROBE 状态。*/
            h->offState = (uint8_t)SPO2_OFF_FINGER;
        }

        /* 未接入期间把 DA 拉回起始值，准备重新调光 */
        h->irIntDa  = (uint16_t)SPO2_MIN_IR_INT_DA_VAL;
        h->redIntDa = (uint16_t)SPO2_MIN_RED_INT_DA_VAL;
    }
#endif  /* SPO2_ADJ_EN */
}

/*********************************************************************************************************
* 函数名称：spo2_light_request_readj
* 功能功能：满一屏且粗调完成时置位"需要执行重调/细调"标志（对应 RealAnalyzeData.m 的 gAdjFlag = 1）
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
*********************************************************************************************************/
void spo2_light_request_readj(spo2_light_t *h)
{
    h->reAdjReq = 1;
}

/*********************************************************************************************************
* 函数名称：spo2_light_judge_step
* 功能功能：判断是否需要重新调光并做细调（逐行对应 JudgeReAdj.m + FineAdj.m）
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：调用者需保证"粗调已完成"（RealDraw 里就是 CheckRoughAdj()==1 且 gAdjFlag==1）
*********************************************************************************************************/
void spo2_light_judge_step(spo2_light_t *h)
{
    if(h->reAdjReq == 0u)
    {
        return;             /* 对应 gAdjFlag == 0：什么都不做 */
    }
    h->reAdjReq = 0;

    if(h->manualMode != 0u)
    {
        return;
    }

/*********************************************************************************************************
:*                                              JudgeReAdj：AD 均值跑出区间就重新粗调
*********************************************************************************************************/
    if(h->roughAdjIrFlag == 1u)
    {
        if((h->irAdAvg > (uint16_t)SPO2_RE_ADJ_IR_TOP_LINE) ||
           (h->irAdAvg < (uint16_t)SPO2_RE_ADJ_IR_BOT_LINE))
        {
            h->irOutRngCnt++;
            if(h->irOutRngCnt >= (uint16_t)SPO2_MAX_CNT_OUT_RNG)
            {
                h->roughAdjIrFlag = 0;      /* 准备重新调光 */
                h->irOutRngCnt    = 0;
            }
        }
        else
        {
            h->irOutRngCnt = 0;
        }
    }
    else
    {
        ResetAdj(h);
        h->irOutRngCnt = 0;
    }

    if(h->roughAdjRedFlag == 1u)
    {
        if((h->redAdAvg > (uint16_t)SPO2_RE_ADJ_RED_TOP_LINE) ||
           (h->redAdAvg < (uint16_t)SPO2_RE_ADJ_RED_BOT_LINE))
        {
            h->redOutRngCnt++;
            if(h->redOutRngCnt >= (uint16_t)SPO2_MAX_CNT_OUT_RNG)
            {
                h->roughAdjRedFlag = 0;
                h->redOutRngCnt    = 0;
            }
        }
        else
        {
            h->redOutRngCnt = 0;
        }
    }
    else
    {
        ResetAdj(h);
        h->redOutRngCnt = 0;
    }

/*********************************************************************************************************
:*                                              FineAdj：粗调都完成后做 ±1 微调
*********************************************************************************************************/
    if((h->roughAdjIrFlag == 1u) && (h->roughAdjRedFlag == 1u))
    {
        if(h->irAdAvg > (uint16_t)(SPO2_IR_CENTRAL_LINE + SPO2_IR_FINE_ADJ_OFFSET))
        {
            h->irIntDa = ClampIrDa((uint16_t)(h->irIntDa + 1u));  /* 光过弱 → 增强 */
        }
        else if(h->irAdAvg < (uint16_t)(SPO2_IR_CENTRAL_LINE - SPO2_IR_FINE_ADJ_OFFSET))
        {
            h->irIntDa = ClampIrDa((uint16_t)(h->irIntDa - 1u));  /* 光过强 → 减弱 */
        }
        else
        {
            /* 光强适中：保持 */
        }

        if(h->redAdAvg > (uint16_t)(SPO2_RED_CENTRAL_LINE + SPO2_RED_FINE_ADJ_OFFSET))
        {
            h->redIntDa = ClampRedDa((uint16_t)(h->redIntDa + 1u));
        }
        else if(h->redAdAvg < (uint16_t)(SPO2_RED_CENTRAL_LINE - SPO2_RED_FINE_ADJ_OFFSET))
        {
            h->redIntDa = ClampRedDa((uint16_t)(h->redIntDa - 1u));
        }
        else
        {
            /* 光强适中：保持 */
        }
    }
}

/*********************************************************************************************************
* 函数名称：spo2_light_is_rough_done
* 功能功能：判断粗调是否完成（对应 CheckRoughAdj.m）
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：1-粗调完成; 0-未完成
* 生成日期：2026年09月24日
* 注    意：SPO2_ADJ_EN = 0 时恒返回 1（血氧/脉率照常计算）
*********************************************************************************************************/
uint8_t spo2_light_is_rough_done(const spo2_light_t *h)
{
#if (SPO2_ADJ_EN == 0u)
    (void)h;
    return 1u;
#else
    if((h->roughAdjIrFlag == 1u) && (h->roughAdjRedFlag == 1u))
    {
        return 1u;
    }
    return 0u;
#endif
}

/*********************************************************************************************************
* 函数名称：spo2_light_is_finger_on
* 功能功能：读取手指连接状态（对应 gFingerOnFlag）
* 输入参数：h-状态机
* 输出参数：void
* 返 回 值：1-连接; 0-脱落
* 生成日期：2026年09月24日
*********************************************************************************************************/
uint8_t spo2_light_is_finger_on(const spo2_light_t *h)
{
    return h->fingerOn;
}

/*********************************************************************************************************
:* 函数名称：spo2_light_is_probe_on
:* 功能功能：读取探头连接状态
:* 输入参数：h-状态机
:* 输出参数：void
:* 返 回 值：1-探头已连接; 0-探头脱落（无光，见 DetectProbeOff）
:* 生成日期：2026年09月24日
*********************************************************************************************************/
uint8_t spo2_light_is_probe_on(const spo2_light_t *h)
{
    return h->probeOn;
}

/*********************************************************************************************************
:* 函数名称：spo2_light_get_off_state
:* 功能功能：读取脱落状态（EnumSPO2OffState）
:* 输入参数：h-状态机
:* 输出参数：void
:* 返 回 值：SPO2_OFF_NONE / SPO2_OFF_FINGER / SPO2_OFF_PROBE
:* 生成日期：2026年09月24日
*********************************************************************************************************/
uint8_t spo2_light_get_off_state(const spo2_light_t *h)
{
    return h->offState;
}

/*********************************************************************************************************
* 函数名称：spo2_light_get_da
* 功能功能：读取当前光强 DA 值（取代 MATLAB 的 SendAdjCmd 下发动作）
* 输入参数：h-状态机; pIrDa-红外光强 DA(输出); pRedDa-红光光强 DA(输出)
* 输出参数：pIrDa, pRedDa
* 返 回 值：void
* 生成日期：2026年09月24日
*********************************************************************************************************/
void spo2_light_get_da(const spo2_light_t *h, uint16_t *pIrDa, uint16_t *pRedDa)
{
    if(pIrDa  != 0) { *pIrDa  = h->irIntDa;  }
    if(pRedDa != 0) { *pRedDa = h->redIntDa; }
}

/*********************************************************************************************************
* 函数名称：spo2_light_apply_manual
* 功能功能：由上位机命令接管光强（对应 MATLAB 侧下发 0x33/0x81 调光命令的场景）
* 输入参数：h-状态机; irDa-红外光强 DA; redDa-红光光强 DA
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：接管后自动调节暂停，直到下一次 ResetAdj（重新插拔手指）才交还自动
*********************************************************************************************************/
void spo2_light_apply_manual(spo2_light_t *h, uint16_t irDa, uint16_t redDa)
{
    h->irIntDa  = ClampIrDa(irDa);
    h->redIntDa = ClampRedDa(redDa);

    h->manualMode       = 1;
    h->fingerOn         = 1;    /* 上位机既然在调光，就按"有信号"处理 */
    h->probeOn          = 1;
    h->offState         = (uint8_t)SPO2_OFF_NONE;
    h->probeOffCnt      = 0;
    h->roughAdjIrFlag   = 1;    /* 视为已标定，允许计算血氧 */
    h->roughAdjRedFlag  = 1;
    h->stableFingerCnt  = (uint16_t)SPO2_MAX_CNT_STABLE_FINGER;
}
