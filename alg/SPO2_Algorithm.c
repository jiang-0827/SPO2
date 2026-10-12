/*********************************************************************************************************
* 模块名称：SPO2_Algorithm.c
* 摘    要：血氧饱和度 (SpO2) 与脉率 (PR) 核心数字信号处理算法
*           1. 8Hz 二阶巴特沃斯数字低通滤波 (IIR) + 10点滑动均值滤波
*           2. 一阶低通 DC 基线自适应跟踪 (DC_ALPHA=0.992)
*           3. 自适应生理不应期与双倍频次峰拦截
*           4. 生理有效域 R 值滤波与 11 档非线性标定表
*           5. 智能双重导联脱落判别
* 当前版本：1.0.0 (从 STM32F103 参考项目移植到 STM32F407)
*********************************************************************************************************/

#include "SPO2_Algorithm.h"
#include <string.h>
#include <stdlib.h>

/*********************************************************************************************************
*                                              宏定义与常量
*********************************************************************************************************/
#define DC_ALPHA                0.992f
#define MA_WINDOW_SIZE          10
#define R_BUFF_SIZE             8
#define PR_BUFF_SIZE            8

#define MIN_PEAK_DIST_SAMPLES   80
#define MAX_PEAK_DIST_SAMPLES   600
#define SPO2_CALIB_COEF         0.825f

#define IIR_B0  0.0133592f
#define IIR_B1  0.0267184f
#define IIR_B2  0.0133592f
#define IIR_A1 -1.6474600f
#define IIR_A2  0.7008968f

static const u16 s_arrRRTable[11] = {580, 620, 650, 670, 700, 730, 760, 780, 810, 840, 880};

/*********************************************************************************************************
*                                              结构体定义
*********************************************************************************************************/
typedef struct {
  float x1, x2;
  float y1, y2;
} IIR_Biquad_t;

/*********************************************************************************************************
*                                              内部状态变量
*********************************************************************************************************/
static SPO2_Result_t s_spo2Result;

static IIR_Biquad_t s_iirRed;
static IIR_Biquad_t s_iirIr;

static float s_arrRedAcBuf[MA_WINDOW_SIZE];
static float s_arrIrAcBuf[MA_WINDOW_SIZE];
static u8    s_u8MaIdx = 0;
static u8    s_u8MaCount = 0;

static float s_arrRValBuf[R_BUFF_SIZE];
static u16   s_arrPRBuf[PR_BUFF_SIZE];
static u8    s_u8RValIdx = 0;
static u8    s_u8PRIdx = 0;

static float s_fPrevIrAc = 0.0f;
static float s_fPeakIr   = -9999.0f;
static float s_fValleyIr = 9999.0f;
static float s_fPeakRed  = -9999.0f;
static float s_fValleyRed= 9999.0f;
static float s_fRunningPP = 30.0f;
static u16   s_u16SamplesSinceLastPeak = 0;
static u16   s_u16RefractoryCount = 0;
static u16   s_u16LastRR = 160;
static u8    s_u8LookingForPeak = 1;

static u16   s_u16ProbeOffCount = 0;
static u16   s_u16FingerOffCount = 0;
static u16   s_u16DCOffCount = 0;

/*********************************************************************************************************
*                                              内部辅助函数
*********************************************************************************************************/
static float ProcessIIR(IIR_Biquad_t* pFilt, float input)
{
  float output = (IIR_B0 * input) + (IIR_B1 * pFilt->x1) + (IIR_B2 * pFilt->x2)
               - (IIR_A1 * pFilt->y1) - (IIR_A2 * pFilt->y2);
  pFilt->x2 = pFilt->x1;
  pFilt->x1 = input;
  pFilt->y2 = pFilt->y1;
  pFilt->y1 = output;
  return output;
}

static void SortFloatArray(float* pArr, u8 len)
{
  u8 i, j;
  float temp;
  for (i = 0; i < len - 1; i++)
  {
    for (j = 0; j < len - 1 - i; j++)
    {
      if (pArr[j] > pArr[j + 1])
      {
        temp = pArr[j];
        pArr[j] = pArr[j + 1];
        pArr[j + 1] = temp;
      }
    }
  }
}

static void SortU16Array(u16* pArr, u8 len)
{
  u8 i, j;
  u16 temp;
  for (i = 0; i < len - 1; i++)
  {
    for (j = 0; j < len - 1 - i; j++)
    {
      if (pArr[j] > pArr[j + 1])
      {
        temp = pArr[j];
        pArr[j] = pArr[j + 1];
        pArr[j + 1] = temp;
      }
    }
  }
}

/*********************************************************************************************************
*                                              API 函数实现
*********************************************************************************************************/
void SPO2_AlgInit(void)
{
  u8 i;
  memset(&s_spo2Result, 0, sizeof(SPO2_Result_t));
  s_spo2Result.status = SPO2_STATUS_NORMAL;
  s_spo2Result.fDispRed = 500.0f;
  s_spo2Result.fDispIr  = 500.0f;
  s_spo2Result.u8SpO2   = 98;
  s_spo2Result.u16PulseRate = 72;
  s_spo2Result.fRVal    = 650.0f;

  memset(&s_iirRed, 0, sizeof(IIR_Biquad_t));
  memset(&s_iirIr, 0, sizeof(IIR_Biquad_t));

  for (i = 0; i < R_BUFF_SIZE; i++)
  {
    s_arrRValBuf[i] = 650.0f;
    s_arrPRBuf[i]   = 72;
  }

  s_u8MaIdx = 0;
  s_u8MaCount = 0;
  s_fPrevIrAc = 0.0f;
  s_fPeakIr   = -9999.0f;
  s_fValleyIr = 9999.0f;
  s_fPeakRed  = -9999.0f;
  s_fValleyRed= 9999.0f;
  s_fRunningPP = 30.0f;
  s_u16SamplesSinceLastPeak = 0;
  s_u16RefractoryCount = 0;
  s_u16LastRR = 160;
  s_u8LookingForPeak = 1;
  s_u16ProbeOffCount = 0;
  s_u16FingerOffCount = 0;
  s_u16DCOffCount = 0;
}

void SPO2_GetResult(SPO2_Result_t* pResult)
{
  if (pResult != NULL)
  {
    memcpy(pResult, &s_spo2Result, sizeof(SPO2_Result_t));
  }
}

void SPO2_AlgProcess(u16 rawRed, u16 rawIr, u16 rawDark)
{
  u8 i;
  float sumRed = 0.0f;
  float sumIr  = 0.0f;
  float rawAcRed, rawAcIr;
  float lpfRed, lpfIr;
  float redAcFilt, irAcFilt;
  int deltaRed = (int)rawRed - (int)rawDark;
  int deltaIr  = (int)rawIr  - (int)rawDark;

  /* 1. 探头脱落 vs 手指脱落检测 */
  if ((abs(deltaRed) < 35 && abs(deltaIr) < 35) || rawIr < 30 || rawIr > 4060 || rawRed < 30 || rawRed > 4060)
  {
    s_u16ProbeOffCount++;
    if (s_u16ProbeOffCount > 20)
    {
      s_spo2Result.status = SPO2_STATUS_PROBE_OFF;
      s_spo2Result.u8SpO2 = 0;
      s_spo2Result.u16PulseRate = 0;
    }
  }
  else
  {
    s_u16ProbeOffCount = 0;

    if (rawIr > 3500 || rawRed > 3500 || rawIr < 100 || rawRed < 100)
    {
      s_u16DCOffCount++;
      if (s_u16DCOffCount > 20)
      {
        if (s_spo2Result.status != SPO2_STATUS_PROBE_OFF)
        {
          s_spo2Result.status = SPO2_STATUS_FINGER_OFF;
          s_spo2Result.u8SpO2 = 0;
          s_spo2Result.u16PulseRate = 0;
        }
      }
    }
    else
    {
      s_u16DCOffCount = 0;
    }
  }

  /* 2. DC 基线一阶低通自适应跟踪 */
  if (s_spo2Result.fDCRed < 10.0f)
  {
    s_spo2Result.fDCRed = (float)rawRed;
    s_spo2Result.fDCIr  = (float)rawIr;
  }
  else
  {
    s_spo2Result.fDCRed = DC_ALPHA * s_spo2Result.fDCRed + (1.0f - DC_ALPHA) * (float)rawRed;
    s_spo2Result.fDCIr  = DC_ALPHA * s_spo2Result.fDCIr  + (1.0f - DC_ALPHA) * (float)rawIr;
  }

  /* 3. AC 提取与 8Hz 二阶巴特沃斯低通滤波 */
  rawAcRed = (float)rawRed - s_spo2Result.fDCRed;
  rawAcIr  = (float)rawIr  - s_spo2Result.fDCIr;

  lpfRed = ProcessIIR(&s_iirRed, rawAcRed);
  lpfIr  = ProcessIIR(&s_iirIr, rawAcIr);

  s_arrRedAcBuf[s_u8MaIdx] = lpfRed;
  s_arrIrAcBuf[s_u8MaIdx]  = lpfIr;
  s_u8MaIdx = (s_u8MaIdx + 1) % MA_WINDOW_SIZE;
  if (s_u8MaCount < MA_WINDOW_SIZE) s_u8MaCount++;

  for (i = 0; i < s_u8MaCount; i++)
  {
    sumRed += s_arrRedAcBuf[i];
    sumIr  += s_arrIrAcBuf[i];
  }
  redAcFilt = sumRed / (float)s_u8MaCount;
  irAcFilt  = sumIr  / (float)s_u8MaCount;

  s_spo2Result.fDispRed = redAcFilt + 500.0f;
  s_spo2Result.fDispIr  = irAcFilt  + 500.0f;

  /* 4. 动态寻峰寻谷与心率/血氧精准提取 */
  s_u16SamplesSinceLastPeak++;
  if (s_u16RefractoryCount > 0)
  {
    s_u16RefractoryCount--;
  }

  if (irAcFilt > s_fPeakIr)
  {
    s_fPeakIr  = irAcFilt;
    s_fPeakRed = redAcFilt;
  }
  if (irAcFilt < s_fValleyIr)
  {
    s_fValleyIr  = irAcFilt;
    s_fValleyRed = redAcFilt;
  }

  if (!s_u8LookingForPeak && (irAcFilt > s_fPrevIrAc))
  {
    s_u8LookingForPeak = 1;
    s_fPeakIr  = irAcFilt;
    s_fPeakRed = redAcFilt;
  }

  if (s_u8LookingForPeak && (irAcFilt < s_fPrevIrAc))
  {
    float acPpIr  = s_fPeakIr  - s_fValleyIr;
    float acPpRed = s_fPeakRed - s_fValleyRed;

    if ((s_u16RefractoryCount == 0) &&
        (s_fPeakIr > 4.0f) && (acPpIr >= 12.0f) && (acPpRed >= 8.0f) && 
        (acPpIr >= (0.40f * s_fRunningPP)) && 
        (s_u16SamplesSinceLastPeak >= MIN_PEAK_DIST_SAMPLES) &&
        s_spo2Result.fDCIr > 100.0f && s_spo2Result.fDCRed > 100.0f)
    {
      float rawR;
      u16 curPR;

      curPR = (u16)((60UL * (u32)SPO2_SAMPLE_RATE_HZ) / s_u16SamplesSinceLastPeak);
      
      if (!((s_spo2Result.u16PulseRate >= 60 && s_spo2Result.u16PulseRate <= 85) && (curPR >= 125)))
      {
        if (curPR >= 30 && curPR <= 180)
        {
          u16 tempPR[PR_BUFF_SIZE];
          s_arrPRBuf[s_u8PRIdx] = curPR;
          s_u8PRIdx = (s_u8PRIdx + 1) % PR_BUFF_SIZE;
          memcpy(tempPR, s_arrPRBuf, sizeof(tempPR));
          SortU16Array(tempPR, PR_BUFF_SIZE);
          s_spo2Result.u16PulseRate = (tempPR[3] + tempPR[4]) / 2;
        }

        rawR = ((acPpRed * 1000.0f) / acPpIr) * SPO2_CALIB_COEF;
        if (rawR >= 500.0f && rawR <= 900.0f)
        {
          float tempR[R_BUFF_SIZE];
          float filteredR;
          u8 tableIdx = 0;
          u8 calcSpO2 = 0;

          s_arrRValBuf[s_u8RValIdx] = rawR;
          s_u8RValIdx = (s_u8RValIdx + 1) % R_BUFF_SIZE;
          memcpy(tempR, s_arrRValBuf, sizeof(tempR));
          SortFloatArray(tempR, R_BUFF_SIZE);
          filteredR = (tempR[3] + tempR[4]) / 2.0f;
          s_spo2Result.fRVal = filteredR;

          tableIdx = 0;
          while ((filteredR >= (float)s_arrRRTable[tableIdx]) && (tableIdx < 10))
          {
            tableIdx++;
          }
          calcSpO2 = (u8)(101 - tableIdx);
          if (calcSpO2 >= 100) calcSpO2 = 99;
          if (calcSpO2 < 90)   calcSpO2 = 90;

          s_spo2Result.u8SpO2 = calcSpO2;
        }

        s_spo2Result.status = SPO2_STATUS_NORMAL;
        s_u16FingerOffCount = 0;
        s_u16DCOffCount = 0;

        s_fRunningPP = 0.8f * s_fRunningPP + 0.2f * acPpIr;
        if (s_fRunningPP < 15.0f) s_fRunningPP = 15.0f;
        if (s_fRunningPP > 45.0f) s_fRunningPP = 45.0f;

        s_u16LastRR = s_u16SamplesSinceLastPeak;

        {
          u16 adaptRefractory = (s_u16LastRR * 60) / 100;
          if (adaptRefractory < 95)  adaptRefractory = 95;
          if (adaptRefractory > 140) adaptRefractory = 140;
          s_u16RefractoryCount = adaptRefractory;
        }

        s_u16SamplesSinceLastPeak = 0;
      }
    }

    s_u8LookingForPeak = 0;
    s_fValleyIr  = irAcFilt;
    s_fValleyRed = redAcFilt;
  }

  /* 5. 长时间无心跳/平线脱落判断 */
  if (s_u16SamplesSinceLastPeak > MAX_PEAK_DIST_SAMPLES)
  {
    s_u16FingerOffCount++;
    if (s_u16FingerOffCount > 40)
    {
      if (s_spo2Result.status != SPO2_STATUS_PROBE_OFF)
      {
        s_spo2Result.status = SPO2_STATUS_FINGER_OFF;
        s_spo2Result.u8SpO2 = 0;
        s_spo2Result.u16PulseRate = 0;
      }
    }
  }

  if (s_spo2Result.status != SPO2_STATUS_NORMAL)
  {
    s_spo2Result.fDispRed = 500.0f;
    s_spo2Result.fDispIr  = 500.0f;
  }

  s_fPrevIrAc = irAcFilt;
}
