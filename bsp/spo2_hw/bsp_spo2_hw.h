/*********************************************************************************************************
* 模块名称：bsp_spo2_hw.h
* 摘    要：血氧模拟前端硬件驱动头文件 (STM32F407VET6, 天空星 LQFP100)
*           引脚分配 (已验证天空星引脚分配图):
*             - PC5 (Pin34): SPO2_RED_CS  (红光 LED 选通, 推挽输出, 高电平点亮)
*             - PC4 (Pin33): SPO2_IR_CS   (红外 LED 选通, 推挽输出, 高电平点亮)
*             - PA5 (Pin30): SPO2_VSIG    (ADC1_CH5 模拟输入, 采集脉搏波信号)
*             - PA4 (Pin29): SPO2_DAI     (DAC_OUT1 模拟输出, LED 光强控制)
*           注: PA4/PA5 在天空星引脚分配图中同时标注为 SPI_CS/SPI_CLK,
*               血氧模式下配置为模拟模式 (GPIO_Mode_AN), SPI 不可同时使用。
*********************************************************************************************************/
#ifndef _BSP_SPO2_HW_H_
#define _BSP_SPO2_HW_H_

#include "stm32f4xx.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define SPO2_DAC_DEFAULT_VAL    110
#define SPO2_DAC_MIN_VAL        40
#define SPO2_DAC_MAX_VAL        600

/*********************************************************************************************************
*                                              API 函数声明
*********************************************************************************************************/
void SPO2_HW_Init(void);
void SPO2_SetDAC(uint16_t dacVal);
uint16_t SPO2_GetDAC(void);
void SPO2_SetRedLED(uint8_t enable);
void SPO2_SetIrLED(uint8_t enable);
uint16_t SPO2_ReadADC(void);
void SPO2_SamplePair(uint16_t* pRedRaw, uint16_t* pIrRaw, uint16_t* pDarkRaw);
void SPO2_AutoGainControl(float curDCRed, float curDCIr);

#endif
