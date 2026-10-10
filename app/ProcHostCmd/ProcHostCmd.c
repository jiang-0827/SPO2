/*********************************************************************************************************
:* 模 块 名 称：ProcHostCmd.c
:* 摘        要：ProcHostCmd模块，上位机命令解析与分发
:* 当前版本：1.0.0
:* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：本文件由心电工程同名文件移植而来
:* 注        意：命令处理函数一律"改状态/回ACK/打一行日志"，绝不做耗时操作，
:*               保证2ms任务不被拖死
:**********************************************************************************************************/
/*********************************************************************************************************
:*                                              包含头文件
*********************************************************************************************************/
#include <stdio.h>
#include "ProcHostCmd.h"
#include "Main.h"
#include "PackUnpack.h"
#include "SendDataToHost.h"
#include "ADC.h"
#include "Timer.h"      //Get2msTickCnt / Get2msMissCnt / ClearTickStatic（丢拍统计）
#include "SPO2FE.h"
#include "spo2_config.h"

/*********************************************************************************************************
:*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
:*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
:*                                              内部变量
*********************************************************************************************************/

/*********************************************************************************************************
:*                                              内部函数声明
*********************************************************************************************************/
static u8  OnWaveSwitch(u8* pMsg);      //波形数据帧上传开关应答处理
static u8  OnSPO2Sample(u8* pMsg);      //血氧采样开关(上位机原生命令 0x33/0x80)应答处理
static u8  OnSetIntensity(u8* pMsg);    //手动设置光强应答处理
static u8  OnGetResult(u8* pMsg);       //查询血氧结果应答处理
static u8  OnResetAlg(u8* pMsg);        //算法复位应答处理
static u8  OnGetStatus(u8* pMsg);       //查询运行状态应答处理
static u8  OnSetSettle(u8* pMsg);       //设置通道建立延时应答处理
static u8  OnSetInvert(u8* pMsg);       //设置极性归一应答处理
static u8  OnSetChannel(u8* pMsg);      //设置ADC规则通道应答处理
static u8  OnClrCount(u8* pMsg);        //清零统计量应答处理
static u8  OnSetRCal(u8* pMsg);         //R值现场标定应答处理(验收:血氧±2%)
static u8  OnDumpAd(u8* pMsg);          //导出1024点原始AD应答处理(验收:≥1000点数据)
static u8  OnGetMetric(u8* pMsg);       //查询血氧指标(PI/脱落状态)应答处理
static u8  OnSysGeneric(u8* pMsg);      //系统模块通用命令(只回ACK)

/*********************************************************************************************************
:*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：OnWaveSwitch
* 功能功能：波形数据帧上传开关应答处理
* 输入参数：pMsg，命令数据首地址，pMsg[0]为开关(0-停止，1-上传)
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：与心电工程同款命令，便于用同一套调试脚本
*********************************************************************************************************/
static u8 OnWaveSwitch(u8* pMsg)
{
  if(pMsg[0] > 1)
  {
    return(CMD_ACK_PARAM_ERR);  //参数非法
  }

  SetWaveSendEnable(pMsg[0]);   //设置上传开关

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnSPO2Sample
* 功能功能：血氧采样开关应答处理(上位机 gCmdStartSamp / gCmdStopSamp 走的就是这条命令)
* 输入参数：pMsg，pMsg[0]=1 开始采样 / 0 停止采样
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：本工程里"开始采样"= 打开波形上传 + 复位算法重新标定（等价于上位机按下
*           "开始采样"后 InitData()+ResetAdj() 的动作）
*********************************************************************************************************/
static u8 OnSPO2Sample(u8* pMsg)
{
  if(pMsg[0] > 1)
  {
    return(CMD_ACK_PARAM_ERR);
  }

  if(pMsg[0] != 0)
  {
    SPO2AlgReset();          //重新标定（清屏 + 复位 R 值/脉率缓冲）
    SetWaveSendEnable(1);    //开始上传波形
  }
  else
  {
    SetWaveSendEnable(0);    //停止上传波形
  }

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnSetIntensity
* 功能功能：手动设置光强应答处理（对应上位机 SendAdjCmd 下发的 0x33/0x81 调光命令）
* 输入参数：pMsg，pMsg[0..1]=红外DA(小端)，pMsg[2..3]=红光DA(小端)
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：收到本命令后光强由上位机接管，自动调节暂停，直到算法复位才交还
*********************************************************************************************************/
static u8 OnSetIntensity(u8* pMsg)
{
  u16 irDa;
  u16 redDa;

  irDa  = (u16)pMsg[0] | ((u16)pMsg[1] << 8);   //小端拼成16位
  redDa = (u16)pMsg[2] | ((u16)pMsg[3] << 8);

  if((irDa > (u16)SPO2_DAC_FULL_DA) || (redDa > (u16)SPO2_DAC_FULL_DA))
  {
    return(CMD_ACK_PARAM_ERR);   //超出 DA 值域(0~250)
  }

  SPO2SetManualLight(irDa, redDa);   //交给算法层：接管光强并立即生效

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnGetResult
* 功能功能：查询血氧结果应答处理
* 输入参数：pMsg
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：回一帧 DAT_SPO2_RESULT 帧，同时打印一行便于串口助手直接看
*********************************************************************************************************/
static u8 OnGetResult(u8* pMsg)
{
  const spo2_result_t* r;
  u16 fundHzX100;

  (void)pMsg;

  r = GetSPO2Result();

  fundHzX100 = (u16)(r->fundHz * 100.0f + 0.5f);

  SendSPO2ResultToHost(r->spo2, r->pulseRate, fundHzX100);

  LogPrintf("SPO2 result: spo2=%d pr=%d off=%d(f/p=%d/%d) adj=%d irDA=%d redDA=%d rVal=%d fund=%d\r\n",
         (int)r->spo2,
         (int)r->pulseRate,
         (int)r->offState,
         (int)r->fingerOn,
         (int)r->probeOn,
         (int)r->adjDone,
         (int)r->irDa,
         (int)r->redDa,
         (int)(r->rVal + 0.5f),
         (int)fundHzX100);

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnResetAlg
* 功能功能：算法复位应答处理
* 输入参数：pMsg
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：对应 MATLAB 停止采样后重新 InitData() + ResetAdj() 的动作
*********************************************************************************************************/
static u8 OnResetAlg(u8* pMsg)
{
  (void)pMsg;

  SPO2AlgReset();

  LogPrintf("SPO2 algorithm reset done\r\n");

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnGetStatus
* 功能功能：查询运行状态应答处理
* 输入参数：pMsg
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：一行把"采集/算法/光强/统计"的关键量全打出来，现场排查只认这一行
*********************************************************************************************************/
static u8 OnGetStatus(u8* pMsg)
{
  const spo2_result_t* r;

  (void)pMsg;

  r = GetSPO2Result();

  LogPrintf("=== SPO2 status ===\r\n");
  LogPrintf(" ADC ch=%d conv=%lu timeout=%lu ovr=%lu\r\n",
         (int)GetADCChannel(),
         (unsigned long)GetADCConvCnt(),
         (unsigned long)GetADCTimeoutCnt(),
         (unsigned long)GetADCOvrCnt());
  /* ---- 实时性证据链(2026-10-09 新增)：一拍到底有没有偷偷丢数据 ----
   *   beat      = 硬件时基走过的 2ms 拍数(每秒 +500)
   *   miss      = 丢拍数 = 少采的 IR/RED 对数，**应恒为 0**(验收 T6 的硬判据)
   *   tickMax   = 单拍(采集+算法+上报)最长耗时 us，预算 2000us，应明显小于它
   *   heavyMax  = 一次屏级重算(屏统计+R+PI)最长耗时 us
   *   heavyRun  = 屏级重算次数(每 4.096s 一次，应与 beat/2048 一致)
   *   lostScreen= 重算没被取走就被覆写丢掉的屏数，**应恒为 0**
   * 注：ADC 的 conv/timeout/ovr 发现不了"整拍被拖长"，所以这三项必须单独看。*/
  LogPrintf(" tick: beat=%lu miss=%lu | tickMax=%lus us heavyMax=%lus us heavyRun=%lu lostScreen=%lu\r\n",
            (unsigned long)Get2msTickCnt(),
            (unsigned long)Get2msMissCnt(),
            (unsigned long)SPO2GetTickMaxUs(),
            (unsigned long)SPO2GetHeavyMaxUs(),
            (unsigned long)SPO2GetHeavyRunCnt(),
            (unsigned long)SPO2GetScreenLostCnt());
  LogPrintf(" FE settle=%dus invert=%d select=%d irDAC=%d redDAC=%d\r\n",
         (int)SPO2FE_GetSettleUs(),
         (int)SPO2FE_GetInvert(),
         (int)SPO2FE_GetSelect(),
         (int)SPO2FE_GetIrDacCode(),
         (int)SPO2FE_GetRedDacCode());
  LogPrintf(" irAd=%d redAd=%d  irAvg=%d redAvg=%d  (IR中线=%d RED中线=%d)\r\n",
         (int)r->irAd, (int)r->redAd, (int)r->irAvg, (int)r->redAvg,
         (int)SPO2_IR_CENTRAL_LINE, (int)SPO2_RED_CENTRAL_LINE);
  LogPrintf(" finger=%d probe=%d offState=%d(%s) adjDone=%d irDA=%d redDA=%d adjTick=%d\r\n",
         (int)r->fingerOn, (int)r->probeOn, (int)r->offState,
         (r->offState == (u8)SPO2_OFF_NONE) ? "OK" :
         ((r->offState == (u8)SPO2_OFF_FINGER) ? "FINGER-OFF" : "PROBE-OFF"),
         (int)r->adjDone, (int)r->irDa, (int)r->redDa, (int)r->adjTickCnt);
  LogPrintf(" irMax=%d irMin=%d irRng=%d  redMax=%d redMin=%d redRng=%d\r\n",
         (int)r->irMax, (int)r->irMin, (int)r->irRng, (int)r->redMax, (int)r->redMin, (int)r->redRng);
  /* PI 打印成百分比（x100 口径，300 = 3.00%）—— 验收项"灌注值数 3%"直接看这一行 */
  LogPrintf(" PI=%d.%02d%% (red=%d.%02d%%) piValid=%d (accept: expect 3.00%%)\r\n",
         (int)(r->piIrX100 / 100u), (int)(r->piIrX100 % 100u),
         (int)(r->piRedX100 / 100u), (int)(r->piRedX100 % 100u),
         (int)r->piValid);
  LogPrintf(" rVal=%d (raw=%d, calGain=%d/1000) idx=%d -> spo2=%d\r\n",
         (int)(r->rVal + 0.5f), (int)(r->rRaw + 0.5f), (int)r->rCalGainPm,
         (int)r->rIndex, (int)r->spo2);
  LogPrintf(" pr=%d (prRaw=%d medRR=%d peaks=%d) fund=%d.%02dHz\r\n",
         (int)r->pulseRate, (int)r->prRaw, (int)r->medRR, (int)r->peakCnt,
         (int)r->fundHz, (int)((r->fundHz - (float)(int)r->fundHz) * 100.0f + 0.5f));
  LogPrintf(" waveSend=%d dumpBusy=%d\r\n", (int)GetWaveSendEnable(), (int)SPO2DumpAdIsBusy());

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnSetSettle
* 功能功能：设置前端通道建立延时应答处理
* 输入参数：pMsg，pMsg[0..1]=μs(小端)
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：现场标定用：用"两个通道的暗值差分"找最小可用建立时间
*********************************************************************************************************/
static u8 OnSetSettle(u8* pMsg)
{
  u16 us;

  us = (u16)pMsg[0] | ((u16)pMsg[1] << 8);

  if((us < 5) || (us > 1000))
  {
    return(CMD_ACK_PARAM_ERR);
  }

  SPO2FE_SetSettleUs(us);

  LogPrintf("FE settle = %d us\r\n", (int)us);

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnSetInvert
* 功能功能：设置极性归一应答处理
* 输入参数：pMsg，pMsg[0]=1 取反(亮→码值小) / 0 不取反
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：上机标定第一步就是"探头空置看 raw"，用这条命令在线试出一个正确值
*********************************************************************************************************/
static u8 OnSetInvert(u8* pMsg)
{
  if(pMsg[0] > 1)
  {
    return(CMD_ACK_PARAM_ERR);
  }

  SPO2FE_SetInvert(pMsg[0]);

  LogPrintf("FE invert = %d\r\n", (int)SPO2FE_GetInvert());

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnSetChannel
* 功能功能：设置ADC规则通道应答处理
* 输入参数：pMsg，pMsg[0]=ADC_Channel_x(0~18)
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：切通道不会改GPIO；切到内部通道17(VREFINT)可做"不用接线的自检"
*********************************************************************************************************/
static u8 OnSetChannel(u8* pMsg)
{
  if(pMsg[0] > 18)
  {
    return(CMD_ACK_PARAM_ERR);
  }

  SetADCChannel(pMsg[0]);

  LogPrintf("ADC channel = %d\r\n", GetADCChannel());

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnClrCount
* 功能功能：清零统计量应答处理
* 输入参数：pMsg
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：
*********************************************************************************************************/
static u8 OnClrCount(u8* pMsg)
{
  (void)pMsg;

  ClearADCStatic();     //ADC：转换数 / 超时数 / 溢出数
  ClearTickStatic();    //Timer：2ms 拍到期数 / 丢拍数
  SPO2ClrTickStat();    //Main：单拍峰值耗时 / 屏级重算耗时与次数 / 丢屏数

  LogPrintf("stats cleared: ADC + tick(miss) + timing(tickMax/heavyMax/lostScreen)\r\n");

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
:* 函数名称：OnSetRCal
:* 功能功能：R 值现场标定应答处理（验收"血氧误差 ±2%"的调节手段）
:* 输入参数：pMsg，pMsg[0..1]=增益千分比(i16小端,500~1500)，pMsg[2..3]=偏置(i16小端,-300~300)
:* 输出参数：void
:* 返 回 值：应答消息
:* 生成日期：2026年09月24日
:* 注    意：标定的是 R 值不是血氧值，改一处全曲线一起动，标定后可复现。
:*           把这次发下去的 gain/offset 记进实验报告，就是完整的标定记录。
*********************************************************************************************************/
static u8 OnSetRCal(u8* pMsg)
{
  i16 gainPm;
  i16 offset;

  gainPm = (i16)((u16)pMsg[0] | ((u16)pMsg[1] << 8));   //小端拼成有符号16位
  offset = (i16)((u16)pMsg[2] | ((u16)pMsg[3] << 8));

  if(SPO2SetRCal(gainPm, offset) == 0)
  {
    LogPrintf("R cal REJECTED: gain=%d offset=%d (allow gain %d~%d, offset %d~%d)\r\n",
           (int)gainPm, (int)offset,
           (int)SPO2_R_CAL_GAIN_MIN, (int)SPO2_R_CAL_GAIN_MAX,
           (int)SPO2_R_CAL_OFF_MIN, (int)SPO2_R_CAL_OFF_MAX);

    return(CMD_ACK_PARAM_ERR);
  }

  LogPrintf("R cal OK: R_used = R_raw * %d/1000 + %d (persists across reset)\r\n",
         (int)gainPm, (int)offset);

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
:* 函数名称：OnDumpAd
:* 功能功能：导出原始 AD 数据应答处理（验收前置条件"至少采样1000个真实AD数据"）
:* 输入参数：pMsg
:* 输出参数：void
:* 返 回 值：应答消息
:* 生成日期：2026年09月24日
:* 注    意：导出是非阻塞的（由 2ms 任务分片发送，详见 Main.c 的 DumpAdTick），
:*           本处理函数立即返回，不会拖住命令通道。
:*           用法：发完命令后把串口收到的 BEGIN/END 之间的内容存成 csv 即可。
*********************************************************************************************************/
static u8 OnDumpAd(u8* pMsg)
{
  (void)pMsg;

  if(SPO2DumpAdIsBusy() != 0)
  {
    LogPrintf("AD dump already running\r\n");

    return(CMD_ACK_NOT_ACC);
  }

  SPO2DumpAdStart();

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
:* 函数名称：OnGetMetric
:* 功能功能：查询血氧指标（灌注指数 PI / 脱落状态 / R 值）应答处理
:* 输入参数：pMsg
:* 输出参数：void
:* 返 回 值：应答消息
:* 生成日期：2026年09月24日
:* 注    意：一行把验收要看的三个量全打出来：
:*             PI        —— "灌注值数 3%"，正常应打印 3.00% 附近
:*             offState  —— "自动判别手指脱落与探头脱落"，0 正常/1 手指/2 探头
:*             rVal/rRaw —— "血氧误差 ±2%" 标定时看这两个数的差是否等于标定参数
*********************************************************************************************************/
static u8 OnGetMetric(u8* pMsg)
{
  const spo2_result_t* r;
  u8 flags;

  (void)pMsg;

  r = GetSPO2Result();

  /* 状态位打包：bit0 PI有效 bit1 手指在 bit2 探头在 bit3 粗调完成 */
  flags = (u8)(((r->piValid != 0u) ? 0x01u : 0x00u)
             | ((r->fingerOn != 0u) ? 0x02u : 0x00u)
             | ((r->probeOn  != 0u) ? 0x04u : 0x00u)
             | ((r->adjDone  != 0u) ? 0x08u : 0x00u));

  SendSPO2MetricToHost(r->piIrX100, r->offState, flags, (u16)(r->rVal + 0.5f));

  LogPrintf("SPO2 metric: PI=%d.%02d%% (red=%d.%02d%%) offState=%d (finger=%d probe=%d)\r\n",
         (int)(r->piIrX100 / 100u), (int)(r->piIrX100 % 100u),
         (int)(r->piRedX100 / 100u), (int)(r->piRedX100 % 100u),
         (int)r->offState, (int)r->fingerOn, (int)r->probeOn);
  LogPrintf("             rVal=%d (raw=%d) idx=%d spo2=%d pr=%d adjDone=%d piValid=%d\r\n",
         (int)(r->rVal + 0.5f), (int)(r->rRaw + 0.5f), (int)r->rIndex,
         (int)r->spo2, (int)r->pulseRate, (int)r->adjDone, (int)r->piValid);

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
* 函数名称：OnSysGeneric
* 功能功能：系统模块通用命令(模块复位应答/模块类型设置/读自检结果)应答处理
* 输入参数：pMsg
* 输出参数：void
* 返 回 值：应答消息
* 生成日期：2026年09月24日
* 注    意：上位机在"开始采样"时会连发这几条命令，本工程只回ACK保证流程不断
*********************************************************************************************************/
static u8 OnSysGeneric(u8* pMsg)
{
  (void)pMsg;

  return(CMD_ACK_OK);
}

/*********************************************************************************************************
:*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitProcHostCmd
* 功能功能：初始化ProcHostCmd模块
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void  InitProcHostCmd(void)
{

}

/*********************************************************************************************************
* 函数名称：ProcHostCmd
* 功能功能：处理来自上位机的数据
* 输入参数：recData
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void ProcHostCmd(u8 recData)
{
  StructPackType pack;    //定义结构体变量
  u8 ack;                 //存储应答消息

  while(UnPackData(recData))   //解析成功
  {
    pack = GetUnPackRslt();    //获取解析结果

    switch(pack.packModuleId)  //模块ID
    {
      case MODULE_WAVE:         //wave模块信息
        switch(pack.packSecondId)
        {
          case CMD_GEN_WAVE:    //波形上传开关
            ack = OnWaveSwitch(pack.arrData);
            break;
          default:
            ack = CMD_ACK_BAD_CMD;
            break;
        }
        SendAckPack(MODULE_WAVE, pack.packSecondId, ack);  //发送命令应答消息
        break;

      case MODULE_SPO2:         //血氧模块信息(0x33)
        switch(pack.packSecondId)
        {
          case CMD_SPO2:            //开始/停止采样(上位机原生命令)
            ack = OnSPO2Sample(pack.arrData);
            break;
          case CMD_SPO2_SET_INT:    //手动设置光强
            ack = OnSetIntensity(pack.arrData);
            break;
          case CMD_SPO2_GET_RSLT:   //查询血氧结果
            ack = OnGetResult(pack.arrData);
            break;
          case CMD_SPO2_RST:        //算法复位
            ack = OnResetAlg(pack.arrData);
            break;
          case CMD_SPO2_STS:        //查询运行状态
            ack = OnGetStatus(pack.arrData);
            break;
          case CMD_SPO2_SET_SETTLE: //设置通道建立延时
            ack = OnSetSettle(pack.arrData);
            break;
          case CMD_SPO2_SET_INV:    //设置极性归一
            ack = OnSetInvert(pack.arrData);
            break;
          case CMD_SPO2_SET_CH:     //切换ADC通道
            ack = OnSetChannel(pack.arrData);
            break;
          case CMD_SPO2_CLR_CNT:    //清零统计量
            ack = OnClrCount(pack.arrData);
            break;
          case CMD_SPO2_R_CAL:      //R值现场标定(验收:血氧误差 ±2%)
            ack = OnSetRCal(pack.arrData);
            break;
          case CMD_SPO2_DUMP_AD:    //导出1024点原始AD(验收:≥1000点真实数据)
            ack = OnDumpAd(pack.arrData);
            break;
          case CMD_SPO2_GET_METRIC: //查询PI/脱落状态/R值(验收:灌注指数与脱落判别)
            ack = OnGetMetric(pack.arrData);
            break;
          default:
            ack = CMD_ACK_BAD_CMD;
            break;
        }
        SendAckPack(MODULE_SPO2, pack.packSecondId, ack);  //发送命令应答消息
        break;

      case MODULE_SYS:          //系统模块信息
        switch(pack.packSecondId)
        {
          case CMD_GET_POST_RSLT:   //读自检结果(上位机开始采样时会发)
          case DAT_RST:             //模块复位信息
          case CMD_PAT_TYPE:        //病人类型设置
          case CMD_MODULE_TYPE:     //模块类型设置(上位机 gCmdModuleType = 0x91)
            ack = OnSysGeneric(pack.arrData);
            break;
          default:
            ack = CMD_ACK_BAD_CMD;
            break;
        }
        SendAckPack(MODULE_SYS, pack.packSecondId, ack);   //发送命令应答消息
        break;

      default:
        break;
    }
  }
}
