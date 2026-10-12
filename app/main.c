#include "board.h"
#include "bsp_uart.h"
#include "bsp_spo2_hw.h"
#include "bsp_timer.h"
#include "SPO2_Algorithm.h"
#include <stdio.h>
#include <string.h>

/*
 * 上位机输出模式:
 *   1 = 课程"串口波形小工具"  (文本, 每行恰好 2 个数, 用 [[id,val]] 更新右侧参数)
 *   2 = VOFA+                 (JustFloat, 4 通道二进制浮点)
 *   3 = 调试文本              (串口助手直读, 含原始 ADC, 数值较多)
 *
 * 说明: 波形小工具按"数字"解析串口流 —— 一行里第 1 个数画到第一道波形,
 *       第 2 个数画到第二道波形, 遇到换行重新计数。因此该模式下每行必须
 *       只发 2 个数, 多发一个数就会把波形配错。
 */
#define OUT_MODE  1

/* 参数刷新间隔: 每 N 次采样刷新一次右侧参数栏 (200Hz / 100 = 2Hz) */
#define PARAM_DIV  100

#if OUT_MODE == 2
/* VOFA+ JustFloat 协议帧尾: 固定 4 字节 (IEEE754 quiet NaN) */
static const uint8_t s_justFloatTail[4] = {
	0x00, 0x00, 0x80, 0x7F
};
#endif

static const char* StatusStr(SPO2_Status_t st)
{
	switch (st)
	{
	case SPO2_STATUS_NORMAL:     return "OK";
	case SPO2_STATUS_FINGER_OFF: return "FINGER_OFF";
	case SPO2_STATUS_PROBE_OFF:  return "PROBE_OFF";
	case SPO2_STATUS_CALCULATING:return "CALC";
	default:                     return "?";
	}
}

static const char* OffStr(SPO2_Status_t st)
{
	switch (st)
	{
	case SPO2_STATUS_PROBE_OFF:  return "PROBE_OFF";
	case SPO2_STATUS_FINGER_OFF: return "FINGER_OFF";
	default:                     return "CONNECTED";
	}
}

int main(void)
{
	uint16_t rawRed, rawIr, rawDark;
	SPO2_Result_t result;
	uint32_t agcCounter = 0;

#if OUT_MODE == 1
	uint32_t paramDiv = 0;
#elif OUT_MODE == 3
	char     buf[128];
	uint32_t printDiv = 0;
#endif

	board_init();
	uart1_init(115200U);
	SPO2_HW_Init();
	SPO2_AlgInit();
	Timer2_Init();

#if OUT_MODE == 1
	/* 参数名只需设置一次, 小工具会保存到本地, 下次自动显示 */
	printf("{{1,SpO2}}\r\n{{2,pulseRate}}\r\n{{3,status}}\r\n{{4,off}}\r\n{{5,DAC}}\r\n");
#elif OUT_MODE == 3
	printf("SPO2 System Ready\r\n");
	printf("R/I/D = raw ADC, SpO2/PR/R = result\r\n");
#endif

	while (1)
	{
		if (g_tim2_5ms_flag > 0)
		{
			g_tim2_5ms_flag--;

			SPO2_SamplePair(&rawRed, &rawIr, &rawDark);
			SPO2_AlgProcess(rawRed, rawIr, rawDark);
			SPO2_GetResult(&result);

			agcCounter++;
			if (agcCounter >= 200)
			{
				agcCounter = 0;
				SPO2_AutoGainControl(result.fDCRed, result.fDCIr);
			}

#if OUT_MODE == 1
			/* 每行恰好 2 个数 -> 第一道(红) 第二道(蓝) */
			printf("%.2f,%.2f\r\n", result.fDispRed, result.fDispIr);

			paramDiv++;
			if (paramDiv >= PARAM_DIV)
			{
				paramDiv = 0;
				printf("[[1,%d %.2f]]\r\n", result.u8SpO2, result.fRVal / 1000.0f);
				printf("[[2,%d]]\r\n", result.u16PulseRate);
				printf("[[3,%s]]\r\n", StatusStr(result.status));
				printf("[[4,%s]]\r\n", OffStr(result.status));
				printf("[[5,%d]]\r\n", SPO2_GetDAC());
			}
#elif OUT_MODE == 2
			/* VOFA+ JustFloat: 4 通道 IEEE754 float32 (小端), 16 字节数据 + 4 字节帧尾 */
			{
				float ch[4];
				ch[0] = result.fDispRed;
				ch[1] = result.fDispIr;
				ch[2] = (float)result.u8SpO2;
				ch[3] = (float)result.u16PulseRate;
				uart_send_buf((uint8_t*)ch, 16U);
				uart_send_buf((uint8_t*)s_justFloatTail, 4U);
			}
#else
			/* 调试文本: 降频打印, 避免 115200 发不完阻塞 200Hz 采样 */
			printDiv++;
			if (printDiv >= 20)
			{
				printDiv = 0;
				sprintf(buf,
				        "R=%4u I=%4u D=%4u | SpO2=%3d PR=%3d R=%.3f | DCr=%6.0f DCi=%6.0f | %s\r\n",
				        rawRed, rawIr, rawDark,
				        result.u8SpO2, result.u16PulseRate, result.fRVal,
				        result.fDCRed, result.fDCIr,
				        StatusStr(result.status));
				printf("%s", buf);
			}
#endif
		}
	}
}
