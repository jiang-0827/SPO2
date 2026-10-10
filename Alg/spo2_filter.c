/*********************************************************************************************************
:* 模 块 名 称：spo2_filter.c
:* 摘        要：脉搏波滤波链逐点实现（扣直流 → 0.5Hz 高通 → [50Hz 陷波] → 20Hz 低通）
:* 当前版本：1.0.0
:* 作        者：Jaziel.Jiang(COPYRIGHT 2018 - 2026 Jaziel.Jiang. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：每个函数与 MATLAB filter() 的 DF2T 实现逐行对应；运算顺序完全一致 ——
:*               这是"逐样本对拍"能收敛到 1e-5 量级的前提。
:*
:*               与 scipy.signal.lfilter（= MATLAB filter）的实测逐样本偏差（double 域）：
:*                 HP 4.7e-10 / LP 2.8e-09 / NOTCH 5.5e-12
*********************************************************************************************************/

#include "spo2_filter.h"

void spo2_iir_init(spo2_iir_t *f, const float *b, const float *a, uint8_t order)
{
    uint8_t i;

    f->b     = b;
    f->a     = a;
    f->order = order;

    for(i = 0u; i < SPO2_IIR_MAX_ORDER; i++)
    {
        f->z[i] = 0.0f;
    }
}

float spo2_iir_step(spo2_iir_t *f, float x)
{
    float   y;
    uint8_t i;
    uint8_t n;

    n = f->order;

    /* 三行一组的 DF2T 递推，与 MATLAB filter() 的 z 更新顺序一字不差：
     * 必须先算 y，再用"上一拍的 z[i+1]"更新 z[i]，故循环自 0 往 N-2 走。 */
    y = f->b[0] * x + f->z[0];

    for(i = 0u; i < (uint8_t)(n - 1u); i++)
    {
        f->z[i] = f->b[i + 1u] * x - f->a[i + 1u] * y + f->z[i + 1u];
    }

    f->z[n - 1u] = f->b[n] * x - f->a[n] * y;

    return y;
}

void spo2_filt_init(spo2_filt_t *f)
{
    /* 三级系数均来自 spo2_coeffs.c（gen_spo2_coeffs.m 生成）：
     * HP_EN=0 / NOTCH_EN=0 时对应数组即直通节 [1 0 0]/[1 0 0]，
     * 上层无需任何条件分支。 */
    spo2_iir_init(&f->hp,    SPO2_HP_B,    SPO2_HP_A,    (uint8_t)SPO2_HP_ORDER);
    spo2_iir_init(&f->notch, SPO2_NOTCH_B, SPO2_NOTCH_A, (uint8_t)SPO2_NOTCH_ORDER);
    spo2_iir_init(&f->lp,    SPO2_LP_B,    SPO2_LP_A,    (uint8_t)SPO2_LP_ORDER);

    f->bias = (float)SPO2_ADC_BIAS_CODE;
}

float spo2_filt_step(spo2_filt_t *f, float adc_code)
{
    float x;

    /* 级 0：先扣直流码值（整数相减，零相位、零代价） */
#if (SPO2_FILT_BIAS_EN != 0u)
    x = adc_code - f->bias;
#else
    x = adc_code;
#endif

    x = spo2_iir_step(&f->hp,    x);   /* 级 1：0.5 Hz 高通（默认生效） */
    x = spo2_iir_step(&f->notch, x);   /* 级 2：50 Hz 陷波（默认直通节） */
    x = spo2_iir_step(&f->lp,    x);   /* 级 3：5 阶 20 Hz 低通（主滤波） */

    return x;
}
