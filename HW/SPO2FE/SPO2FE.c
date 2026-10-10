/*********************************************************************************************************
:* 模 块 名 称：SPO2FE.c
:* 摘        要：血氧模拟前端驱动（光强 DAC + IR/RED 通道选通 + 一次采集成对点）
:* 当前版本：1.0.0
:* 作        者：Jaziel.Jiang(COPYRIGHT 2018 - 2026 Jaziel.Jiang. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：本文件为血氧工程新增（心电工程没有对应模块，心电前端只有一路信号）
:* 注        意：建立延时用 ARM/SysTick 模块的 DelayNus() 实现，属于"忙等"，
:*               100μs×2/2ms ≈ 10% CPU；这段时间中断照常响应，不影响串口收发
:**********************************************************************************************************/
/*********************************************************************************************************
:*                                              包含头文件
*********************************************************************************************************/
#include "SPO2FE.h"
#include "ADC.h"
#include "SysTick.h"

/*********************************************************************************************************
:*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
:*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
:*                                              内部变量
*********************************************************************************************************/
static  u16 s_iIrDacCode  = 0;                          //当前红外光强 DAC 码
static  u16 s_iRedDacCode = 0;                          //当前红光光强 DAC 码
static  u8  s_iSel        = (u8)SPO2FE_SEL_NONE;        //当前选通状态
static  u16 s_iSettleUs   = SPO2_FE_SETTLE_US;         //通道建立延时(μs)
#if (SPO2_FE_INVERT != 0u)
static  u8  s_iInvert     = 1;                          //极性归一：亮→码值小
#else
static  u8  s_iInvert     = 0;
#endif

/*********************************************************************************************************
:*                                              内部函数声明
*********************************************************************************************************/
static  void  ConfigDac(void);          //配置 DAC 输出(SPO2_DAI)
static  void  ConfigCsPins(void);       //配置 IR/RED 选通脚
static  void  WriteDacU16(u16 code);    //按 12bit 码值写 DAC

/*********************************************************************************************************
:*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：ConfigDac
* 功能功能：配置 DAC_OUT1(PA4) 为 SPO2_DAI 光强控制输出
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：F4 的 DAC 在 APB1；PA4 要配成模拟输入态(GPIO_Mode_AN)才能被 DAC 独占
*********************************************************************************************************/
static  void  ConfigDac(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  DAC_InitTypeDef  DAC_InitStructure;

  RCC_AHB1PeriphClockCmd(SPO2FE_DAC_GPIO_CLK, ENABLE);    //使能 DAC 引脚所在端口时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);     //使能 DAC 时钟(APB1)

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin  = SPO2FE_DAC_GPIO_PIN;     //PA4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;            //模拟态，交给 DAC
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(SPO2FE_DAC_GPIO_PORT, &GPIO_InitStructure);

  DAC_StructInit(&DAC_InitStructure);
  DAC_InitStructure.DAC_Trigger        = DAC_Trigger_None;         //不用触发，直接写数据寄存器
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;  //不产生噪声/三角波
  DAC_InitStructure.DAC_OutputBuffer   = DAC_OutputBuffer_Enable;  //开输出缓冲，便于驱动板内缓冲级
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;  //未用，填默认值
  DAC_Init(SPO2FE_DAC_CHANNEL, &DAC_InitStructure);

  DAC_SetChannel1Data(DAC_Align_12b_R, 0);                //上电先输出 0（最暗）
  DAC_Cmd(SPO2FE_DAC_CHANNEL, ENABLE);
}

/*********************************************************************************************************
* 函数名称：ConfigCsPins
* 功能功能：配置 SPO2_IR_CS / SPO2_RED_CS 为推挽输出
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：上电先置为"都不选通"（全灭），避免上电瞬间两个 LED 同时点亮
*********************************************************************************************************/
static  void  ConfigCsPins(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(SPO2FE_CS_CLK, ENABLE);          //使能选通脚端口时钟

  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = (u16)(SPO2FE_IR_CS_PIN | SPO2FE_RED_CS_PIN);
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;          //通用推挽输出
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;        //低频控制信号，低速档即可(EMI 更小)
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;         //下拉，上电/复位期间保持"不选通"
  GPIO_Init(SPO2FE_CS_PORT, &GPIO_InitStructure);

#if (SPO2FE_CS_ACTIVE_HIGH != 0u)
  GPIO_ResetBits(SPO2FE_CS_PORT, (u16)(SPO2FE_IR_CS_PIN | SPO2FE_RED_CS_PIN));   //低=不选通
#else
  GPIO_SetBits(SPO2FE_CS_PORT, (u16)(SPO2FE_IR_CS_PIN | SPO2FE_RED_CS_PIN));     //高=不选通
#endif
}

/*********************************************************************************************************
* 函数名称：WriteDacU16
* 功能功能：把 12bit 码值写到 DAC 数据寄存器
* 输入参数：code-12bit 码值(0~4095)
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：
*********************************************************************************************************/
static  void  WriteDacU16(u16 code)
{
  if(code > (u16)SPO2_DAC_MAX_CODE)
  {
    code = (u16)SPO2_DAC_MAX_CODE;
  }

  DAC_SetChannel1Data(DAC_Align_12b_R, code);
}

/*********************************************************************************************************
:*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitSPO2FE
* 功能功能：初始化血氧前端
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：需在 InitSysTick() 之后调用(DelayNus 依赖 SysTick)；本函数不初始化 ADC，
*           ADC 由 InitADC() 负责
*********************************************************************************************************/
void  InitSPO2FE(void)
{
  ConfigCsPins();                                   //先配选通脚（保证上电全灭）
  ConfigDac();                                      //再配 DAC

  s_iIrDacCode  = 0;
  s_iRedDacCode = 0;
  s_iSel        = (u8)SPO2FE_SEL_NONE;
  s_iSettleUs   = SPO2_FE_SETTLE_US;

  SPO2FE_Select(SPO2FE_SEL_IR);                     //默认选通红外，便于上电即见波形
}

/*********************************************************************************************************
* 函数名称：SPO2FE_SetIntensityDa
* 功能功能：按"DA 值"设置红外/红光光强（算法侧用的就是 DA 值，0~250）
* 输入参数：irDa-红外光强 DA; redDa-红光光强 DA
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：DA(0~250) → DAC 码(0~4095) 是线性换算，只影响"调光步长"的物理含义；
*           由于光强调节是闭环的(看 ADC 码值决定加减)，线性系数不影响收敛结果
*********************************************************************************************************/
void  SPO2FE_SetIntensityDa(u16 irDa, u16 redDa)
{
  u16 irCode;
  u16 redCode;

  irCode  = (u16)(((u32)irDa  * (u32)SPO2_DAC_MAX_CODE) / (u32)SPO2_DAC_FULL_DA);
  redCode = (u16)(((u32)redDa * (u32)SPO2_DAC_MAX_CODE) / (u32)SPO2_DAC_FULL_DA);

  SPO2FE_SetIntensityCode(irCode, redCode);
}

/*********************************************************************************************************
* 函数名称：SPO2FE_SetIntensityCode
* 功能功能：按 DAC 12bit 码值直接设置红外/红光光强
* 输入参数：irCode-红外 DAC 码; redCode-红光 DAC 码
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：本板 DAC 只有一路(SPO2_DAI)，IR/RED 共用同一个光强控制脚，
*           所以这里"两路都写"实际上是"后写的生效"。为让上层代码保持
*           "两个独立光强"的语义，这里保存两个值、按当前选通状态输出对应的那一个
*********************************************************************************************************/
void  SPO2FE_SetIntensityCode(u16 irCode, u16 redCode)
{
  s_iIrDacCode  = (irCode  > (u16)SPO2_DAC_MAX_CODE) ? (u16)SPO2_DAC_MAX_CODE : irCode;
  s_iRedDacCode = (redCode > (u16)SPO2_DAC_MAX_CODE) ? (u16)SPO2_DAC_MAX_CODE : redCode;

  if(s_iSel == (u8)SPO2FE_SEL_RED)
  {
    WriteDacU16(s_iRedDacCode);
  }
  else
  {
    WriteDacU16(s_iIrDacCode);
  }
}

/*********************************************************************************************************
* 函数名称：SPO2FE_GetIrDacCode / SPO2FE_GetRedDacCode
* 功能功能：读取当前光强对应的 DAC 码
* 输入参数：void
* 输出参数：void
* 返 回 值：DAC 码值
* 生成日期：2026年09月24日
* 注    意：
*********************************************************************************************************/
u16  SPO2FE_GetIrDacCode(void)
{
  return s_iIrDacCode;
}

u16  SPO2FE_GetRedDacCode(void)
{
  return s_iRedDacCode;
}

/*********************************************************************************************************
* 函数名称：SPO2FE_Select
* 功能功能：选择光电通道(SPO2_IR_CS / SPO2_RED_CS)，并把该路对应的光强写到 DAC
* 输入参数：sel-SPO2FE_SEL_NONE / SPO2FE_SEL_IR / SPO2FE_SEL_RED
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：切通道顺带写 DAC 是因为本板 IR/RED 共用 SPO2_DAI 一个光强控制脚：
*           MATLAB 侧的调光命令也是"一次给 IR/RED 两个 DA"，语义与本实现一致
*********************************************************************************************************/
void  SPO2FE_Select(EnumSPO2FESel sel)
{
  if(sel == SPO2FE_SEL_IR)
  {
#if (SPO2FE_CS_ACTIVE_HIGH != 0u)
    GPIO_SetBits(SPO2FE_CS_PORT, SPO2FE_IR_CS_PIN);
    GPIO_ResetBits(SPO2FE_CS_PORT, SPO2FE_RED_CS_PIN);
#else
    GPIO_ResetBits(SPO2FE_CS_PORT, SPO2FE_IR_CS_PIN);
    GPIO_SetBits(SPO2FE_CS_PORT, SPO2FE_RED_CS_PIN);
#endif
    WriteDacU16(s_iIrDacCode);
  }
  else if(sel == SPO2FE_SEL_RED)
  {
#if (SPO2FE_CS_ACTIVE_HIGH != 0u)
    GPIO_ResetBits(SPO2FE_CS_PORT, SPO2FE_IR_CS_PIN);
    GPIO_SetBits(SPO2FE_CS_PORT, SPO2FE_RED_CS_PIN);
#else
    GPIO_SetBits(SPO2FE_CS_PORT, SPO2FE_IR_CS_PIN);
    GPIO_ResetBits(SPO2FE_CS_PORT, SPO2FE_RED_CS_PIN);
#endif
    WriteDacU16(s_iRedDacCode);
  }
  else
  {
#if (SPO2FE_CS_ACTIVE_HIGH != 0u)
    GPIO_ResetBits(SPO2FE_CS_PORT, (u16)(SPO2FE_IR_CS_PIN | SPO2FE_RED_CS_PIN));
#else
    GPIO_SetBits(SPO2FE_CS_PORT, (u16)(SPO2FE_IR_CS_PIN | SPO2FE_RED_CS_PIN));
#endif
    WriteDacU16(s_iIrDacCode);
  }

  s_iSel = (u8)sel;
}

/*********************************************************************************************************
* 函数名称：SPO2FE_GetSelect
* 功能功能：读取当前选通状态
* 输入参数：void
* 输出参数：void
* 返 回 值：SPO2FE_SEL_NONE / SPO2FE_SEL_IR / SPO2FE_SEL_RED
* 生成日期：2026年09月24日
* 注    意：
*********************************************************************************************************/
u8  SPO2FE_GetSelect(void)
{
  return s_iSel;
}

/*********************************************************************************************************
* 函数名称：SPO2FE_ReadPair
* 功能功能：采集一对 IR/RED 码值（含通道切换与建立延时）
* 输入参数：pIrAd-红外码值(输出); pRedAd-红光码值(输出)
* 输出参数：pIrAd, pRedAd
* 返 回 值：1-采集成功; 0-参数错误
* 生成日期：2026年09月24日
* 注    意：这是血氧采集的核心时序，对应 MATLAB 侧同一包里的"红外+红光"两个采样点：
*             (1) 选通红外 → 等前端建立 → 连续转 SPO2_ADC_OS 次取平均 → 记 IR
*             (2) 选通红光 → 等前端建立 → 连续转 SPO2_ADC_OS 次取平均 → 记 RED
*           两个通道都按 SPO2_FS 的节拍各出一个点，因此 IR/RED 各 500SPS
*           过采样平均的动机见 HW/ADC/ADC.c 的 ADCConvertVsigAvg() 说明
*********************************************************************************************************/
u8  SPO2FE_ReadPair(u16 *pIrAd, u16 *pRedAd)
{
  u16 raw;

  if((pIrAd == 0) || (pRedAd == 0))
  {
    return 0;
  }

  /* ---- (1) 红外通道 ---- */
  SPO2FE_Select(SPO2FE_SEL_IR);      //选通红外 LED
  DelayNus(s_iSettleUs);             //等前端(跨阻级 + 后级滤波)建立

  raw  = ADCConvertVsigAvg((u8)SPO2_ADC_OS);  //过采样平均：降随机噪声，保 AC 分辨率
  *pIrAd = SPO2FE_Normalize(raw);    //极性归一

  /* ---- (2) 红光通道 ---- */
  SPO2FE_Select(SPO2FE_SEL_RED);     //选通红光 LED
  DelayNus(s_iSettleUs);
  raw  = ADCConvertVsigAvg((u8)SPO2_ADC_OS);
  *pRedAd = SPO2FE_Normalize(raw);

  /* 采完停在红外通道：下一个节拍从这里开始，保证"IR 先 RED 后"的顺序稳定 */
  SPO2FE_Select(SPO2FE_SEL_IR);

  return 1;
}

/*********************************************************************************************************
* 函数名称：SPO2FE_SetSettleUs / SPO2FE_GetSettleUs
* 功能功能：设置/读取通道建立延时
* 输入参数：us-建立延时(μs)，会钳到 [5, 1000]
* 输出参数：void
* 返 回 值：void / 当前值
* 生成日期：2026年09月24日
* 注    意：调太小会出现"IR 通道里混进 RED 的充电尾巴"→ 两个通道波形一起跳；
*           调太大则挤占 2ms 预算。现场用"两个通道的暗值差分"来定标
*********************************************************************************************************/
void  SPO2FE_SetSettleUs(u16 us)
{
  if(us < 5u)    { us = 5u;    }
  if(us > 1000u) { us = 1000u; }

  s_iSettleUs = us;
}

u16  SPO2FE_GetSettleUs(void)
{
  return s_iSettleUs;
}

/*********************************************************************************************************
* 函数名称：SPO2FE_SetInvert / SPO2FE_GetInvert
* 功能功能：运行时改变/读取极性归一开关
* 输入参数：en-1 取反(亮→码值小) 0 不取反
* 输出参数：void
* 返 回 值：void / 当前开关
* 生成日期：2026年09月24日
* 注    意：默认值来自 SPO2_FE_INVERT；提供运行时开关是为了"上电看一眼 raw 再定"的现场标定
*********************************************************************************************************/
void  SPO2FE_SetInvert(u8 en)
{
  s_iInvert = (en == 0u) ? 0u : 1u;
}

u8  SPO2FE_GetInvert(void)
{
  return s_iInvert;
}

/*********************************************************************************************************
* 函数名称：SPO2FE_Normalize
* 功能功能：按当前极性把原始码值归一成"光越强→码值越小"的统一约定
* 输入参数：raw-ADC 原始码值
* 输出参数：void
* 返 回 值：归一后的码值
* 生成日期：2026年09月24日
* 注    意：见 SPO2FE.h 头注释第 2 条
*********************************************************************************************************/
u16  SPO2FE_Normalize(u16 raw)
{
  if(s_iInvert != 0u)
  {
    return (u16)((u16)SPO2_ADC_FULL - raw);
  }
  return raw;
}
