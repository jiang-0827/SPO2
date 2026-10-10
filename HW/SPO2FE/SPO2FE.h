/*********************************************************************************************************
:* 模 块 名 称：SPO2FE.h
:* 摘        要：血氧模拟前端驱动（光强 DAC + IR/RED 通道选通 + 一次采集成对点）
:* 当前版本：1.0.0
:* 作        者：Jaziel.Jiang(COPYRIGHT 2018 - 2026 Jaziel.Jiang. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：血氧板的 H4 只有 7 个引脚（见 SCH_Schematic1_2026-09-22），MCU 侧需要
:*               "两个数字输出 + 一个模拟输出 + 一个模拟输入"共 4 根信号线，本模块把它们
:*               全部收在一处：
:*                 H4-6 SPO2_IR_CS   数字输出  → 选通红外 LED（板内 SGM3005 模拟开关）
:*                 H4-3 SPO2_RED_CS  数字输出  → 选通红光 LED
:*                 H4-7 SPO2_DAI     模拟输出  → 光强控制（板内 LMV321 缓冲 → LED 驱动）
:*                 H4-4 SPO2_VSIG    模拟输入  → 脉搏波（由 HW/ADC 采样）
:*               H4-1 GND、H4-2 +5V（血氧板自带 RT9193 转 3V3A，MCU 不给 3V3）、H4-5 NC
:* 注        意：
:*   1) 前端的"分时复用"决定了采样时序：一个节拍里必须
:*         选 IR → 等建立 → 转一次 → 选 RED → 等建立 → 转一次
:*      建立时间是硬约束（太小会通道串扰，太大会挤占 2ms 预算），见 SPO2_FE_SETTLE_US。
:*   2) 极性归一（SPO2_FE_INVERT）在这一层做，好处是"算法永远看到同一种码值约定"：
:*         归一后：光越强 → 码值越小（与 MATLAB 里 IR_CENTRAL_LINE=2000、
:*                 "AD 值小于 80 判脱落"的隐含约定一致）。
:*      血氧板空载（无手指）时 LED 直射光电管，是"光最强"的极端，归一后应≈0。
:*      【上电必做的一次性标定】探头空置时看串口打印的 raw 值：
:*        - raw ≈ 0    → 前端是"光越强→码值越小"，SPO2_FE_INVERT 保持 1
:*        - raw ≈ 4095 → 前端是"光越强→码值越大"，把 SPO2_FE_INVERT 改成 0
:*   3) 未经实际接线确认前，下面 4 个引脚宏是**占位值**，必须与你的接法一致。
:**********************************************************************************************************
:* 硬件连接（[实测] 请按实际接线核对/修改）：
:*   SPO2_VSIG  →  PA5 (ADC12_IN5)    见 HW/ADC/ADC.h
:*   SPO2_DAI   →  PA4 (DAC_OUT1)     DAC 12bit，0~VDDA 线性输出
:*   SPO2_IR_CS →  PC4                GPIO 推挽输出
:*   SPO2_RED_CS→  PC5                GPIO 推挽输出
:*********************************************************************************************************/
#ifndef _SPO2FE_H_
#define _SPO2FE_H_

/*********************************************************************************************************
:*                                              包含头文件
*********************************************************************************************************/
#include "DataType.h"
#include "stm32f4xx_conf.h" //本头文件里用到了 GPIO_Pin_x / DAC_Channel_x，需要外设头文件
#include "spo2_config.h"    //SPO2_FE_SETTLE_US / SPO2_FE_INVERT / SPO2_DAC_* 都取自系统配置

/*********************************************************************************************************
:*                                              宏定义
*********************************************************************************************************/
#define SPO2FE_CS_PORT          GPIOC          //IR/RED 选通脚所在端口
#define SPO2FE_CS_CLK          RCC_AHB1Periph_GPIOC
#define SPO2FE_IR_CS_PIN        GPIO_Pin_4     //SPO2_IR_CS  → PC4
#define SPO2FE_RED_CS_PIN       GPIO_Pin_5     //SPO2_RED_CS → PC5

#define SPO2FE_DAC_CHANNEL      DAC_Channel_1  //SPO2_DAI → PA4 = DAC_OUT1
#define SPO2FE_DAC_GPIO_PORT    GPIOA
#define SPO2FE_DAC_GPIO_CLK     RCC_AHB1Periph_GPIOA
#define SPO2FE_DAC_GPIO_PIN     GPIO_Pin_4

/* 选通脚的有效电平：血氧板 SGM3005 的 INx 为高电平闭合，故按高有效接
 * （若换用其它模拟开关/驱动，改这里即可）*/
#define SPO2FE_CS_ACTIVE_HIGH   1

/*********************************************************************************************************
:*                                              枚举结构体定义
*********************************************************************************************************/
//当前选通状态
typedef enum
{
  SPO2FE_SEL_NONE = 0,   //两路都不选（"暗"态，可用来摸前端暗电平/零点）
  SPO2FE_SEL_IR,         //选通红外
  SPO2FE_SEL_RED,        //选通红光
  SPO2FE_SEL_MAX
} EnumSPO2FESel;

/*********************************************************************************************************
:*                                              API函数声明
*********************************************************************************************************/
void  InitSPO2FE(void);                              //初始化血氧前端(DAC光强 + 通道选通脚)

void  SPO2FE_SetIntensityDa(u16 irDa, u16 redDa);    //按"DA 值(0~250)"设置两个光强(内部换算成DAC码)
void  SPO2FE_SetIntensityCode(u16 irCode, u16 redCode);//按 DAC 12bit 码值直接设置两个光强
u16   SPO2FE_GetIrDacCode(void);                     //读取当前红外光强对应 DAC 码
u16   SPO2FE_GetRedDacCode(void);                    //读取当前红光光强对应 DAC 码

void  SPO2FE_Select(EnumSPO2FESel sel);              //选择光电通道
u8    SPO2FE_GetSelect(void);                        //读取当前选通状态

u8    SPO2FE_ReadPair(u16 *pIrAd, u16 *pRedAd);      //采集一对 IR/RED 码值(含选通与建立延时)

void  SPO2FE_SetSettleUs(u16 us);                    //设置通道建立延时(μs)
u16   SPO2FE_GetSettleUs(void);                      //读取通道建立延时(μs)

void  SPO2FE_SetInvert(u8 en);                       //运行时改变极性归一(1-取反 0-不取反)
u8    SPO2FE_GetInvert(void);                        //读取极性归一开关

u16   SPO2FE_Normalize(u16 raw);                     //按当前极性把原始码值归一成"亮→小"

#endif
