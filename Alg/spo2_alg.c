/*********************************************************************************************************
:* 模 块 名 称：spo2_alg.c
:* 摘        要：血氧算法顶层聚合（对外的唯一入口）
:* 当前版本：1.0.0
:* 作        者：Jaziel.Jiang(COPYRIGHT 2018 - 2026 Jaziel.Jiang. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：调用顺序与 MATLAB 的对应关系见 spo2_alg.h 头注释。
*********************************************************************************************************/

#include "spo2_alg.h"

/*********************************************************************************************************
* 函数名称：spo2_alg_init
* 功能功能：初始化血氧算法全部子模块
* 输入参数：a-算法实例(必须 static/全局)
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
*********************************************************************************************************/
void spo2_alg_init(spo2_alg_t *a)
{
    spo2_filt_init(&a->filt);
    spo2_pulse_init(&a->pulse);
    spo2_calc_init(&a->calc);
    spo2_light_init(&a->light);

    a->adjDiv   = 0;
    a->screenPending = 0;
    a->screenLostCnt = 0;
    a->piBadCnt = 0;

    a->out.fingerOn   = 0;
    a->out.probeOn    = 0;                              /* 上电默认"探头脱落" */
    a->out.offState   = (uint8_t)SPO2_OFF_PROBE;
    a->out.adjDone    = 0;
    a->out.screenDone = 0;
    a->out.piIrX100   = 0;
    a->out.piRedX100  = 0;
    a->out.piValid    = 0;
    a->out.rCalGainPm = a->calc.rCalGainPm;
    a->out.irAd       = 0;
    a->out.redAd      = 0;
    a->out.irDa       = (uint16_t)SPO2_MIN_IR_INT_DA_VAL;
    a->out.redDa      = (uint16_t)SPO2_MIN_RED_INT_DA_VAL;
    a->out.irFilt     = 0.0f;
    a->out.pulseRate  = (int16_t)SPO2_INVALID_DATA;
    a->out.spo2       = (int16_t)SPO2_INVALID_DATA;
    a->out.irMax      = 0;
    a->out.irMin      = 0;
    a->out.redMax     = 0;
    a->out.redMin     = 0;
    a->out.irRng      = 0;
    a->out.redRng     = 0;
    a->out.irAvg      = 0;
    a->out.redAvg     = 0;
    a->out.rVal       = (float)SPO2_R_SEED;
    a->out.rRaw       = (float)SPO2_R_SEED;
    a->out.rIndex     = 1;
    a->out.prRaw      = (uint16_t)SPO2_PR_SEED;
    a->out.medRR      = 0;
    a->out.peakCnt    = 0;
    a->out.fundHz     = 0.0f;
    a->out.adjTickCnt = 0;
}

/*********************************************************************************************************
* 函数名称：spo2_alg_reset
* 功能功能：算法整体复位（对应 MATLAB 停止采样后重新 InitData + ResetAdj 的动作）
* 输入参数：a-算法实例
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：屏缓冲也会被清掉，波形需要重新铺满一屏才会再出结果
*********************************************************************************************************/
void spo2_alg_reset(spo2_alg_t *a)
{
    int16_t gainPm;
    int16_t offset;

    /* 复位会重建整个算法状态机，但 R 值标定是"设备级"参数（换模拟器才需要重标），
     * 必须跨复位保留 —— 否则现场辛苦标定完，一按复位就白标了。*/
    spo2_calc_get_r_cal(&a->calc, &gainPm, &offset);

    spo2_alg_init(a);

    (void)spo2_calc_set_r_cal(&a->calc, gainPm, offset);
    a->out.rCalGainPm = a->calc.rCalGainPm;
}

/*********************************************************************************************************
* 函数名称：spo2_alg_set_manual_light
* 功能功能：把光强控制权交给上位机（对应 MATLAB 下发 0x33/0x81 调光命令）
* 输入参数：a-算法实例; irDa-红外光强 DA; redDa-红光光强 DA
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
*********************************************************************************************************/
void spo2_alg_set_manual_light(spo2_alg_t *a, uint16_t irDa, uint16_t redDa)
{
    spo2_light_apply_manual(&a->light, irDa, redDa);
}

/*********************************************************************************************************
* 函数名称：spo2_alg_step
* 功能功能：逐样本推进血氧算法（500Hz 实时调用）
* 输入参数：a-算法实例; irAd-红外码值(已归一); redAd-红光码值(已归一)
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：本函数不含任何外设访问，DAC/GPIO/串口都由上层负责
*           满一屏时只置 screenPending 标志：屏统计 + R 值 + PI 改由主
*           循环调用 spo2_alg_screen_run() 执行。因为屏统计要扫 2048 点，留在本拍内会把
*           "IR 采完到 RED 采完"拉开好几毫秒，两个通道不再成对同时刻，R 值与 PI 的
*           口径直接被污染 —— 这正是血氧 ±2% 最吃不起的误差(丢多少拍由 0x84 打印)。
*********************************************************************************************************/
void spo2_alg_step(spo2_alg_t *a, uint16_t irAd, uint16_t redAd)
{
    uint8_t screenFull;

    /* ---- ① 单通道滤波：只用车间(红外)通道算脉率，与 MATLAB CalcPulseRate(gDataAxes1) 一致 ---- */
    a->out.irFilt = spo2_filt_step(&a->filt, (float)irAd);

    /* ---- ② 逐点脉率检测 ---- */
    a->out.pulseRate = spo2_pulse_step(&a->pulse, a->out.irFilt);

    /* ---- ③ 写入屏幕缓冲(IR/RED 原始码值)：满一屏只置标志，重算挪到采样拍之外 ---- */
    screenFull = spo2_calc_push(&a->calc, irAd, redAd);

    if(screenFull != 0u)
    {
        if(a->screenPending != 0u)
        {
            /* 上一屏的重算还没被主循环取走，缓冲就被新屏覆写：
             * 这一屏的 R 值/PI 永久丢失，必须记账，不能静默吞掉。 */
            a->screenLostCnt++;
        }

        a->screenPending = 1u;
    }

    /* ---- ④ 更新光强调节所需的最新码值 ---- */
    spo2_light_set_sample(&a->light, irAd, redAd);
    a->out.irAd  = irAd;
    a->out.redAd = redAd;

    /* ---- ⑤ 光强调节节拍(每 SPO2_ADJ_PERIOD_SMP 样本一次，0.1s)----
     * 粗调/重调/细调都只是整数比较与加减，开销恒定且远小于采样与滤波，留在拍内。 */
    a->adjDiv++;
    if(a->adjDiv >= (uint16_t)SPO2_ADJ_PERIOD_SMP)
    {
        a->adjDiv = 0;
        a->out.adjTickCnt++;

        spo2_light_rough_step(&a->light);     /* RoughAdj */
        spo2_light_judge_step(&a->light);     /* JudgeReAdj + FineAdj，内部受 reAdjReq 门控 */

        if(a->light.resetFlag != 0u)          /* 刚 ResetAdj 过 → 同步复位 R 值/脉率缓冲 */
        {
            spo2_calc_reset(&a->calc);
            spo2_pulse_reset(&a->pulse);
            a->light.resetFlag = 0u;
        }
    }

    /* ---- ⑥ 脱落与状态汇总 ---- */
    a->out.fingerOn = spo2_light_is_finger_on(&a->light);
    a->out.probeOn  = spo2_light_is_probe_on(&a->light);
    a->out.offState = spo2_light_get_off_state(&a->light);
    a->out.adjDone  = spo2_light_is_rough_done(&a->light);

    spo2_light_get_da(&a->light, &a->out.irDa, &a->out.redDa);

    a->out.prRaw       = a->pulse.prRaw;
    a->out.medRR       = a->pulse.medRR;
    a->out.peakCnt     = a->pulse.peakCount;
    a->out.rCalGainPm  = a->calc.rCalGainPm;

    /* 门控：手指或探头坏 / 未标定 / 无脉动 → 血氧与脉率一并置无效
     * (与上位机"显示 ---"的规范一致)
     * 注：屏级重算挪出拍外后，本门控用到的是上一屏的 piValid，即 PI 类无效判定
     *     比原来晚一拍(2ms)生效；而 PI 本身 4.096s 才更新一次，可忽略。 */
    if((a->out.fingerOn == 0u) || (a->out.adjDone == 0u) || (a->out.piValid == 0u))
    {
        spo2_calc_set_invalid(&a->calc);
        a->out.spo2      = (int16_t)SPO2_INVALID_DATA;
        a->out.pulseRate = (int16_t)SPO2_INVALID_DATA;
        a->out.fundHz    = 0.0f;
    }
    else
    {
        a->out.spo2      = a->calc.spo2;
        a->out.pulseRate = a->pulse.pr;
    }
}

/*********************************************************************************************************
* 函数名称：spo2_alg_get_result
* 功能功能：读取最新结果快照
* 输入参数：a-算法实例
* 输出参数：void
* 返 回 值：结果结构体指针（只读）
* 生成日期：2026年09月24日
*********************************************************************************************************/
const spo2_result_t *spo2_alg_get_result(const spo2_alg_t *a)
{
    return &a->out;
}

/*********************************************************************************************************
:* 函数名称：spo2_alg_set_r_cal / spo2_alg_get_r_cal
:* 功能功能：读写 R 值现场标定参数（验收"血氧误差 ±2%"的调节手段）
:* 输入参数：a-算法实例; gainPm-千分比增益(500~1500); offset-绝对偏置(-300~300)
:* 输出参数：pGainPm, pOffset
:* 返 回 值：1-成功; 0-参数越界被拒
:* 生成日期：2026年09月24日
:* 注    意：标定参数会跨 spo2_alg_reset() 保留（见 spo2_alg_reset 的说明）
*********************************************************************************************************/
uint8_t spo2_alg_set_r_cal(spo2_alg_t *a, int16_t gainPm, int16_t offset)
{
    if(spo2_calc_set_r_cal(&a->calc, gainPm, offset) == 0u)
    {
        return 0u;
    }

    a->out.rCalGainPm = a->calc.rCalGainPm;

    return 1u;
}

void spo2_alg_get_r_cal(const spo2_alg_t *a, int16_t *pGainPm, int16_t *pOffset)
{
    spo2_calc_get_r_cal(&a->calc, pGainPm, pOffset);
}

/*********************************************************************************************************
:* 函数名称：spo2_alg_get_screen_point
:* 功能功能：读取屏缓冲里相对"最新样本"往前的第 back 对原始码值
:* 输入参数：a-算法实例; back-回溯深度(0=最新, 1=上一个, ...)
:* 输出参数：pIrAd, pRedAd
:* 返 回 值：1-成功; 0-越界
:* 生成日期：2026年09月24日
:* 注    意：专供"原始 AD 数据导出"（验收要求至少 1000 点真实数据）。
:*           故意做成"从最新往回读"：导出耗时 1.28s，期间采样仍在写缓冲，
:*           倒着读的位置只会越来越旧、永远不会被写指针追上，因此导出的
:*           SPO2_DUMP_POINTS 个点是一次无撕裂的一致快照。
*********************************************************************************************************/
uint8_t spo2_alg_get_screen_point(const spo2_alg_t *a, uint16_t back,
                                 uint16_t *pIrAd, uint16_t *pRedAd)
{
    uint16_t idx;

    if(back >= (uint16_t)SPO2_SCREEN_LEN)
    {
        return 0u;
    }

    /* pos 是"下一个待写位置"，故最新样本落在 pos-1；往回数 back 个后回绕 */
    idx = (uint16_t)((a->calc.pos + (uint16_t)SPO2_SCREEN_LEN - 1u - back)
                     % (uint16_t)SPO2_SCREEN_LEN);

    if(pIrAd  != 0) { *pIrAd  = a->calc.irBuf[idx];  }
    if(pRedAd != 0) { *pRedAd = a->calc.redBuf[idx]; }

    return 1u;
}

/*********************************************************************************************************
* 函数名称：spo2_alg_screen_pending
* 功能功能：查询是否有待执行的屏级重算
* 输入参数：a-算法实例
* 输出参数：void
* 返 回 值：1-有待执行的重算 0-没有
* 生成日期：2026年10月09日
* 注    意：主循环每次转完一圈都来问一次，开销是一次 u8 读比较。
*********************************************************************************************************/
uint8_t spo2_alg_screen_pending(const spo2_alg_t *a)
{
    return a->screenPending;
}

/*********************************************************************************************************
* 函数名称：spo2_alg_get_screen_lost_cnt
* 功能功能：读取被覆写丢弃的屏数(诊断用)
* 输入参数：a-算法实例
* 输出参数：void
* 返 回 值：累计丢屏数(应为 0)
* 生成日期：2026年10月09日
* 注    意：不为 0 说明主循环长期取不走重算活(串口打印过长、导出过密等)，
*           此时血氧读数会长时间不更新，必须查上游而不是改算法。
*********************************************************************************************************/
uint32_t spo2_alg_get_screen_lost_cnt(const spo2_alg_t *a)
{
    return a->screenLostCnt;
}

/*********************************************************************************************************
* 函数名称：spo2_alg_screen_run
* 功能功能：执行一屏的屏级重算(屏统计 + R 值 + 血氧查表 + PI + 幅度谱)
* 输入参数：a-算法实例
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年10月09日
* 注    意：(1) 为什么必须从 2ms 采样拍里挪出来：屏统计(2048 点×2 扫描)在 168MHz 下
*               要 1~4ms(取决于优化等级)，留在拍内会把"IR 采完到 RED 采完"之间拉开
*               好几毫秒 —— IR 与 RED 不再成对同时刻，R 值与 PI 的口径直接被污染，
*               而这正是血氧 ±2% 最吃不起的误差。挪到拍外后最多让整拍晚一点开始，
*               成对点仍然紧邻；晚了多少由 Timer 的 miss 计数如实上报。
*           (2) 与屏缓冲的并发：本函数读整块 irBuf/redBuf 期间，采样拍仍在往 pos
*               之后写新屏的点。满屏瞬间被覆写的点数 = 重算耗时 / 2ms(实测一般 1~2 点)，
*               屏统计会把这 1~2 个新点算进刚收满的那一屏，与 MATLAB"收满一屏再算"
*               的数值行为一致，误差在 2/2048 量级，可忽略。
*           (3) 调用顺序与挪出前逐步对应：屏统计 → 光强均值回灌 → 结果快照 →
*               PI 判据 → 请求重调，不改变任何数值路径。
*********************************************************************************************************/
void spo2_alg_screen_run(spo2_alg_t *a)
{
    if(a->screenPending == 0u)
    {
        return;                     //没满一屏：空转返回，主循环每次转一圈都会调用它
    }

    a->screenPending = 0u;

    /* 屏统计：无论是否算得出血氧都要算(光强调节要用)；R 值/血氧只在粗调完成后进入
     * —— 对应 MATLAB "if CheckRoughAdj() == 1" 的门控 */
    spo2_calc_screen_run(&a->calc, spo2_light_is_rough_done(&a->light));

    spo2_light_set_screen_avg(&a->light, a->calc.irAvg, a->calc.redAvg);

    a->out.irMax  = a->calc.irMax;
    a->out.irMin  = a->calc.irMin;
    a->out.redMax = a->calc.redMax;
    a->out.redMin = a->calc.redMin;
    a->out.irRng  = a->calc.irRng;
    a->out.redRng = a->calc.redRng;
    a->out.irAvg  = a->calc.irAvg;
    a->out.redAvg = a->calc.redAvg;
    a->out.rVal   = a->calc.rVal;
    a->out.rRaw   = a->calc.rRaw;
    a->out.rIndex = a->calc.rIndex;

    /* 灌注指数 PI：验收"灌注值数 3%"的直接证据(PIx100 = 300)；
     * 同时用作"光路通但无脉动"的二级判据 —— 掉手指时 PI 归零说明手指没夹好，
     * 连续 N 屏如此则把血氧/脉率一并判无效(见 spo2_alg_step 的门控)。 */
    a->out.piIrX100  = a->calc.piIrX100;
    a->out.piRedX100 = a->calc.piRedX100;

    if(spo2_light_is_rough_done(&a->light) != 0u)
    {
        if(a->calc.piIrX100 < (uint16_t)SPO2_PI_VALID_MIN_X100)
        {
            if(a->piBadCnt < 0xFFu)
            {
                a->piBadCnt++;
            }
        }
        else
        {
            a->piBadCnt = 0;                /* 脉动恢复，立刻清零 */
        }
    }

    a->out.piValid = (a->piBadCnt >= (uint8_t)SPO2_FINGER_OFF_CNT) ? 0u : 1u;

    if(spo2_light_is_rough_done(&a->light) != 0u)
    {
        spo2_light_request_readj(&a->light);   /* 对应 gAdjFlag = 1 */
    }

    a->out.screenDone = 1u;
}
