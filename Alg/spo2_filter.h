/*********************************************************************************************************
:* 模 块 名 称：spo2_filter.h
:* 摘        要：脉搏波滤波链逐点实现（扣直流 → 0.5Hz 高通 → [50Hz 陷波] → 20Hz 低通）
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：对应 MATLAB：SPO2DynamicPrj/FilterPulseWave.m
:*                     Wc = gFiltFc/(Fs/2); [b,a] = butter(gFiltDegree, Wc);
:*                     dataOut = filter(b, a, dataIn);
:*                     （设计期代码不上机，其产物由 gen_spo2_coeffs.m 固化为
:*                     spo2_coeffs.h/.c）
:*
:*               为什么这里要比 MATLAB 多做"扣直流 + 高通"两级：
:*                 MATLAB 的 filter 是块处理，输入是整屏 2048 点原始 AD（含 ~2000 码的直流），
:*                 输出里直流照样在，上位机画波形时靠 Y 轴范围把它压住即可；而 C 侧的输出要
:*                 喂给逐样本峰值检测与波形上传，必须先把直流和慢漂移拿掉，否则自适应门限
:*                 会被直流噪声主导。这两级都是零相位代价的"辅助级"：
:*                   - 扣直流：整数相减，不引入任何延迟；
:*                   - 0.5Hz 高通：只在 <1Hz 起作用，脉搏基波(0.5~2Hz)几乎不受影响（1Hz 处 -0.3dB）。
:*                 血氧饱和度用的是**原始码值的峰峰值**（见 spo2_calc.c），不经过本模块，
:*                 所以这里引入的任何相位畸变都不会影响 SpO2 的准确性。
:*
:*               沿袭的设计约束（移植时别改）：
:*                 1) 被关掉的级写成"直通节" b=[1 0 0], a=[1 0 0]：照常调用同一份递推代码，
:*                    不引入 if 分支 —— 分支越少，移植期越不容易错。
:*                 2) 系数一律按 a[0] 归一化（gen_spo2_coeffs.m 导出时完成），
:*                    递推式里不出现除法（M4F 的 VDIV.F32 ≈14 cycle，乘加只有 3~7 cycle）。
:*                 3) 递推式为 MATLAB filter() 内部使用的**转置直接II型(DF2T)**，运算顺序与
:*                    MATLAB 逐字一致，这是"逐样本对拍"能收敛到 1e-5 量级的前提。
:*
:*               用法（2ms 时间片里逐样本调用，单文件无任何外设依赖）：
:*                   #include "spo2_filter.h"
:*                   static spo2_filt_t g_filt;                 // 状态常驻，约 100 字节
:*                   void app_init(void)  { spo2_filt_init(&g_filt); }
:*                   void on_adc_pair(uint16_t ir_code)         // 每 2ms 一次
:*                   {
:*                     float y = spo2_filt_step(&g_filt, (float)ir_code);
:*                     // y = 滤波后的红外脉搏波（去直流、去漂移、20Hz 低通），单位仍是 ADC 码值域
:*                     // 换算引脚端电压：uV = y * SPO2_ADC_UV_PER_LSB
:*                   }
:* 注        意：输出与 MATLAB 逐点仿真（single 精度）逐样本等价，对拍判据 1e-5 量级
:*               （single 累加顺序差异）。
*********************************************************************************************************/
#ifndef SPO2_FILTER_H
#define SPO2_FILTER_H

#include <stdint.h>
#include "spo2_config.h"
#include "spo2_coeffs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- N 阶 IIR 的转置直接II型（DF2T）----
 *
 *   y[n]  = b0*x[n] + z[0]
 *   z[i]  = b[i+1]*x[n] - a[i+1]*y[n] + z[i+1]      (i = 0 .. N-2)
 *   z[N-1]= b[N]*x[n]   - a[N]*y[n]
 *
 * 用同一段代码覆盖二阶与五阶，省掉两套状态机；DF2T 只存 N 个状态
 * （DF1 要存 2N 个），且与 MATLAB filter() 的算术顺序完全一致。
 * 系数指针直接指向 Flash 里的 const 数组（spo2_coeffs.c），不占 RAM。 */
typedef struct {
    const float *b;                        /* [b0 .. bN]，已按 a0 归一化 */
    const float *a;                        /* [1  .. aN] */
    uint8_t      order;                    /* 阶数 N（= 数组长度 - 1） */
    float        z[SPO2_IIR_MAX_ORDER];    /* 跨样本常驻状态 */
} spo2_iir_t;

/* ---- 三级滤波链 ---- */
typedef struct {
    spo2_iir_t hp;     /* 级 1：0.5 Hz 高通（HP_EN=0 时系数为直通节） */
    spo2_iir_t notch;  /* 级 2：50 Hz 陷波（NOTCH_EN=0 时系数为直通节） */
    spo2_iir_t lp;     /* 级 3：5 阶 20 Hz 低通（主滤波） */
    float      bias;   /* 直流码值（SPO2_ADC_BIAS_CODE），最先扣除 */
} spo2_filt_t;

void  spo2_iir_init(spo2_iir_t *f, const float *b, const float *a, uint8_t order);
float spo2_iir_step(spo2_iir_t *f, float x);
void  spo2_filt_init(spo2_filt_t *f);
float spo2_filt_step(spo2_filt_t *f, float adc_code);

#ifdef __cplusplus
}
#endif

#endif /* SPO2_FILTER_H */
