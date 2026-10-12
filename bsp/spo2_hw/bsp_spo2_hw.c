/*********************************************************************************************************
* 模块名称：bsp_spo2_hw.c
* 摘    要：血氧模拟前端硬件驱动实现 (STM32F407VET6, 天空星 LQFP100)
*           实现 GPIO、DAC1 (PA4)、ADC1 (PA5) 初始化及 4 阶段微秒级采样时序
*           引脚已验证天空星引脚分配图: PC4(Pin33), PC5(Pin34), PA4(Pin29), PA5(Pin30)
*********************************************************************************************************/
#include "bsp_spo2_hw.h"
#include "board.h"
#include <stddef.h>

static uint16_t s_u16DacVal = SPO2_DAC_DEFAULT_VAL;

/*********************************************************************************************************
*                                              API 函数实现
*********************************************************************************************************/

void SPO2_HW_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DAC_InitTypeDef DAC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;

	/* 1. 时钟使能 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	/* 注: F4 的 ADC 分频由 ADC_CommonInit 的 ADC_Prescaler 设置, 无 RCC_ADCCLKConfig */

	/* 2. GPIO 配置: PC4 (IR_CS), PC5 (RED_CS) 为推挽输出 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);

	/* 3. GPIO 配置: PA5 (ADC) 和 PA4 (DAC) 为模拟模式 */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 4. DAC1 (PA4) 初始化 */
	DAC_InitStructure.DAC_Trigger                      = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration               = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_InitStructure.DAC_OutputBuffer                 = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_Cmd(DAC_Channel_1, ENABLE);
	SPO2_SetDAC(s_u16DacVal);

	/* 5. ADC1 (PA5, CH5) 独立单通道模式配置 */
	ADC_DeInit();

	ADC_CommonInitStruct.ADC_Mode             = ADC_Mode_Independent;
	ADC_CommonInitStruct.ADC_Prescaler        = ADC_Prescaler_Div4;
	ADC_CommonInitStruct.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);

	ADC_InitStructure.ADC_Resolution             = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode           = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode     = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge   = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign              = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion        = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_Cmd(ADC1, ENABLE);
}

void SPO2_SetDAC(uint16_t dacVal)
{
	if (dacVal < SPO2_DAC_MIN_VAL) dacVal = SPO2_DAC_MIN_VAL;
	if (dacVal > SPO2_DAC_MAX_VAL) dacVal = SPO2_DAC_MAX_VAL;
	s_u16DacVal = dacVal;
	DAC_SetChannel1Data(DAC_Align_12b_R, s_u16DacVal);
}

uint16_t SPO2_GetDAC(void)
{
	return s_u16DacVal;
}

void SPO2_SetRedLED(uint8_t enable)
{
	if (enable)
		GPIO_SetBits(GPIOC, GPIO_Pin_5);
	else
		GPIO_ResetBits(GPIOC, GPIO_Pin_5);
}

void SPO2_SetIrLED(uint8_t enable)
{
	if (enable)
		GPIO_SetBits(GPIOC, GPIO_Pin_4);
	else
		GPIO_ResetBits(GPIOC, GPIO_Pin_4);
}

uint16_t SPO2_ReadADC(void)
{
	uint32_t timeout = 20000;
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_56Cycles);
	ADC_SoftwareStartConv(ADC1);
	while ((ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) && (--timeout));
	return ADC_GetConversionValue(ADC1);
}

void SPO2_SamplePair(uint16_t* pRedRaw, uint16_t* pIrRaw, uint16_t* pDarkRaw)
{
	/* 阶段 0: 暗电平采样 */
	if (pDarkRaw != NULL)
	{
		*pDarkRaw = SPO2_ReadADC();
	}

	/* 阶段 1: 红光通道采样 */
	SPO2_SetRedLED(1);
	delay_us(150);
	if (pRedRaw != NULL)
	{
		*pRedRaw = SPO2_ReadADC();
	}
	SPO2_SetRedLED(0);

	/* 阶段 2: 暗态弛豫 */
	delay_us(100);

	/* 阶段 3: 红外光通道采样 */
	SPO2_SetIrLED(1);
	delay_us(150);
	if (pIrRaw != NULL)
	{
		*pIrRaw = SPO2_ReadADC();
	}
	SPO2_SetIrLED(0);
}

void SPO2_AutoGainControl(float curDCRed, float curDCIr)
{
	float avgDC = (curDCRed + curDCIr) / 2.0f;
	uint16_t curDac = s_u16DacVal;

	if (avgDC < 50.0f) return;

	if (avgDC < 1500.0f)
	{
		if (curDac < SPO2_DAC_MAX_VAL)
		{
			curDac += (avgDC < 1000.0f) ? 2 : 1;
			SPO2_SetDAC(curDac);
		}
	}
	else if (avgDC > 2400.0f)
	{
		if (curDac > SPO2_DAC_MIN_VAL)
		{
			curDac -= (avgDC > 2800.0f) ? 2 : 1;
			SPO2_SetDAC(curDac);
		}
	}
}
