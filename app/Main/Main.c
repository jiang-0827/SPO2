/*********************************************************************************************************
:* 模 块 名 称：Main.c
:* 摘        要：本文件包含所有硬件初始化的函数和main函数
:* 当前版本：1.0.0
:* 作        者：Jaziel.Jiang(COPYRIGHT 2018 - 2026 Jaziel.Jiang. All rights reserved.)
:* 生成日期：2026年09月24日
:* 说        明：本文件由心电工程(Main.c)移植而来，任务框架(2ms/1s时间片)与心电完全一致：
:*               2ms 时间片 = 一个"采集成对点 + 跑算法 + 上报"的节拍
:*               1s  时间片 = 结果上报 / 状态打印
:* 注        意：注意勾选Options for Target 'Target1'->Target->Use MicroLIB，否则printf无法使用
:**********************************************************************************************************
:* 与心电工程 Main.c 的关键差异：
:*   1) 心电的数据源是"TIM2 触发 → ADC1(DMA) → 环形队列"，Main 从队列里取点；
:*      血氧的数据源是"2ms 时间片 → SPO2FE_ReadPair()（内部切通道+等建立+转两次）"，
:*      一次拿回 IR/RED 一对点，见 HW/SPO2FE/SPO2FE.c。
:*   2) 心电只上传波形；血氧还要上传"光强帧"和"血氧结果帧"（因为原来这两样是
:*      MATLAB 上位机自己算/自己显示的，现在算法搬到了 MCU 上）。
:*   3) 血氧多了一步"把算法算出的光强 DA 值写给 DAC"，也就是把 MATLAB 的
:*      SendAdjCmd(→从机) 变成了本地闭环。
:*********************************************************************************************************/
/*********************************************************************************************************
:*                                              包含头文件
*********************************************************************************************************/
#include <stdio.h>
#include "Main.h"
#include "stm32f4xx_conf.h"
#include "DataType.h"
#include "NVIC.h"
#include "SysTick.h"
#include "RCC.h"
#include "Timer.h"
#include "UART1.h"
#include "LED.h"
#include "ProcHostCmd.h"
#include "PackUnpack.h"
#include "SendDataToHost.h"
#include "ADC.h"
#include "SPO2FE.h"
#include "spo2_alg.h"
#include "spo2_config.h"

/*********************************************************************************************************
:*                                              宏定义
*********************************************************************************************************/
/* ============ 串口输出模式选择 ============
 * 0 = 上位机 PCT 协议模式（与 MATLAB 上位机/原模块协议一致，默认）：
 *       0x33/0x02 波形帧(红外+红光，各24bit，与上位机解析式逐字节对齐)
 *       0x33/0x03 光强帧(红外DA+红光DA)
 *       0x33/0x04 血氧结果帧(本工程新增：血氧+脉率+主频)
 *     此模式下串口只跑协议帧，不再打周期文本（避免污染帧流）
 * 1 = VOFA+ 调试模式：发送 FireWater 文本帧(每2个样本一帧)，并打一行文本状态
 *     方便没有上位机时用 VOFA+/串口助手直接看波形
 */
#define  SPO2_OUT_MODE          1


/* VOFA+ 模式下每 N 个样本发一帧文本（N=2 → 250Hz） */
#define  SPO2_VOFA_DIV          2

/* ============ 双波形通道选择（串口波形小工具 V1.0.5 只支持 2 道波形）============
 * 工具规则：一行里的第 1 个数 → 波形1，第 2 个数 → 波形2，第 3 个数又回到波形1……
 *           遇到回车换行才重新计数。**所以一行只能发两个数**，多发就会错位。
 * 可选值：
 *   0 = IR 原始码值(归一后)     1 = IR 脉搏波(滤波后×10，交流域)
 *   2 = RED 原始码值(归一后)    3 = IR 屏均值(直流)
 *   4 = 脉率                    5 = 血氧
 * 【当前默认 CH1=IR原始码值、CH2=RED原始码值】——把两条脱落判据的输入都放上屏：
 *   探头脱落要求"两路同时 ≥ SPO2_PROBE_OFF_CODE_MIN(3900)"，
 *   手指脱落要求"任一路 < SPO2_IR/RED_OFF_MAX_VAL(150)"，
 *   两个门限的输入都在屏上，插拔手指/探头时两路怎么变一目了然。
 * 想看脉搏波形状就把 CH2 改回 1（IR 滤波后×10）；想看 IR 的屏均值就改成 3。*/
#define  SPO2_WAVE_CH1          0
#define  SPO2_WAVE_CH2          2

/*********************************************************************************************************
:*                                              内部变量
*********************************************************************************************************/
/* 血氧算法实例：内含 2048 点×2 的屏缓冲(8KB)（FFT 已移除），必须放在 .bss，
 * 千万不要改成局部变量（默认栈 1KB，会直接 HardFault） */
static  spo2_alg_t  s_spo2Alg;

static  u8  s_iWaveSendEnable = 1;      //波形数据帧上传开关，默认开启，便于上电就能看到波形
#if (SPO2_OUT_MODE == 0)
static  u32 s_iTickCnt        = 0;      //2ms 节拍计数（诊断：每秒应增 500）
#endif

/* ---- 原始 AD 数据导出状态机（验收前置条件"≥1000 点真实 AD 数据"）---- */
static  u8  s_iDumpBusy    = 0;         //1=正在导出
static  u16 s_iDumpBack    = 0;         //已导出的"回溯深度"（0=最新点）
static  u16 s_iDumpTickDiv = 0;         //分片节拍分频
static  u16 s_iDumpSkip    = 0;         //启动前空跑的节拍数(先让串口队列排空)
static  u8  s_iDumpDropCnt = 0;         //被串口队列丢掉的字节数(诊断)
static  u8  s_iDumpHdrDone = 0;         //BEGIN 文本头是否已打印(必须晚于命令 ACK 帧)

/* ---- 实时性统计(2026-10-09 新增)：拍耗时峰值 + 屏级重算耗时与次数 ----
 * 只读 DWT->CYCCNT，不改任何采样时序；由命令 0x84 打印、0x88 清零。
 * 判据：tickMax 应 < 2000us(否则 miss 会涨)；heavyMax 是屏级重算的真实开销，
 *       它现在落在拍外，只要小于"两拍之间的空闲"(约 1.6ms)就不影响成对采样。*/
static  u32  s_iTickMaxUs    = 0;       //单拍(采集+算法+上报)最长耗时 us
static  u32  s_iHeavyMaxUs   = 0;       //一次屏级重算(屏统计+R+PI)最长耗时 us
static  u32  s_iHeavyRunCnt  = 0;       //屏级重算执行次数


/*********************************************************************************************************
:*                                              枚举结构体定义
*********************************************************************************************************/

/*********************************************************************************************************
:*                                              内部函数声明
*********************************************************************************************************/
static  void  InitSoftware(void);   //初始化软件相关的模块
static  void  InitHardware(void);   //初始化硬件相关的模块
static  void  Proc2msTask(void);    //2ms任务处理
static  void  Proc1SecTask(void);   //1s任务处理
static  void  ProcWaveData(void);   //采集一对IR/RED点 → 跑算法 → 写光强 → 上报
static  void  PrintBootInfo(void);  //打印上电自检信息
static  void  DumpAdTick(void);     //原始AD导出：每拍发一行(非阻塞分片)
static  void  ProcSPO2HeavyTask(void);  //屏级重算(统计+R+PI+FFT)，在采样拍之外执行

#if (SPO2_OUT_MODE != 0)
static  void  VofaWaveTick(u16 irAd, u16 redAd);   //VOFA+ 文本帧
#endif

/*********************************************************************************************************
:*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
:* 函数名称：InitSoftware
:* 功能功能：所有的与软件相关的模块初始化工作在此函数完成
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年09月24日
:* 注    意：
**********************************************************************************************************/
static  void  InitSoftware(void)
{
  spo2_alg_init(&s_spo2Alg);  //初始化血氧算法(滤波/脉率/血氧/光强调节四件套)
  InitPackUnpack();       //初始化PackUnpack模块
  InitProcHostCmd();      //初始化ProcHostCmd模块
  InitSendDataToHost();   //初始化SendDataToHost模块
}

/*********************************************************************************************************
:* 函数名称：InitHardware
:* 功能功能：所有的与硬件相关的模块初始化工作在此函数完成
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年09月24日
:* 注    意：顺序要求：InitRCC必须在InitTimer/InitADC之前(它们要按查到的时钟算分频)，
:*           InitNVIC必须在所有开中断的模块之前；InitSPO2FE 必须在 InitSysTick 之后
:*           (它用 DelayNus 做通道建立延时)，且要在 InitADC 之后(它调用 ADC 转换)
**********************************************************************************************************/
static  void  InitHardware(void)
{
  InitRCC();                  //查询系统时钟(168MHz由启动文件的SystemInit配置)
  InitNVIC();                 //初始化NVIC模块(设置中断优先级分组)
  InitSysTick();              //初始化SysTick模块(1ms节拍，供DelayNms/DelayNus使用)
  InitUART1(UART1_BAUD_RATE); //初始化UART模块
  InitTimer();                //初始化Timer模块(2ms/1s时间片标志)
  InitLED();                  //初始化LED模块
  InitADC();                  //初始化ADC模块(SPO2_VSIG 单通道软件触发)
  InitSPO2FE();               //初始化血氧前端(DAC光强 + IR/RED通道选通)
}

/*********************************************************************************************************
:* 函数名称：PrintBootInfo
:* 功能功能：打印上电自检信息
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年09月24日
:* 注    意：PLL未锁定时说明8MHz晶振没起振，芯片在跑16MHz内部RC，此时采样率与波特率都会
:*           偏4倍，必须先把这一行看对，再去看后面的数据
**********************************************************************************************************/
static  void  PrintBootInfo(void)
{
  printf("\r\n==============================================\r\n");
  printf(" SPO2 algorithm on STM32F407VET6 (LCKFB Firebase-Tx)\r\n");
  printf(" SYSCLK = %lu Hz, PLL lock = %d\r\n",
         (unsigned long)GetSysClockFreq(), (int)IsRunByPLL());
  printf(" APB1 TIM clk = %lu Hz, APB2(PCLK2) = %lu Hz\r\n",
         (unsigned long)GetAPB1TimClockFreq(), (unsigned long)GetAPB2ClockFreq());
  /* 注意：MicroLIB 不支持 %f，所有浮点量都先转成整数再打印 */
  printf(" SPO2 sample rate = %d SPS/ch, screen = %d points (%d ms)\r\n",
         (int)SPO2_FS, (int)SPO2_SCREEN_LEN,
         (int)((float)SPO2_SCREEN_LEN * 1000.0f / SPO2_FS));
  /* H4 接线自检：这 5 个信号对应血氧板 H4 排针的 7/6/5/4/3 脚（1=GND, 2=+5V） */
  printf(" H4: VSIG=PA5(CH%d) DAI=PA4(DAC1) IR_CS=PC4 RED_CS=PC5 CAP=PC6\r\n",
         (int)ADC_VSIG_CHANNEL);
  printf(" FE settle = %dus, invert = %d (light stronger -> code smaller)\r\n",
         (int)SPO2FE_GetSettleUs(), (int)SPO2FE_GetInvert());
  printf(" IR central = %d, RED central = %d, off threshold = %d\r\n",
         (int)SPO2_IR_CENTRAL_LINE, (int)SPO2_RED_CENTRAL_LINE, (int)SPO2_IR_OFF_MAX_VAL);
  /* 下面几行是"五参数验收标准"的血氧条目对照，方便现场一眼核对配置是否符合验收 */
  printf(" [accept] PR range = %d-%d bpm, SpO2 range = 90-99%%, PI expect = 3.00%%\r\n",
         (int)SPO2_PR_MIN, (int)SPO2_PR_MAX);
  printf(" [accept] ADC oversampling = %d (random noise /sqrt(%d))\r\n",
         (int)SPO2_ADC_OS, (int)SPO2_ADC_OS);
  printf(" [accept] off detect: probe(no light) code >= %d, finger(too bright) code < %d, en = %d\r\n",
         (int)SPO2_PROBE_OFF_CODE_MIN, (int)SPO2_IR_OFF_MAX_VAL, (int)SPO2_PROBE_OFF_EN);
  {
    i16 rCalGain;
    i16 rCalOff;

    SPO2GetRCal(&rCalGain, &rCalOff);
    printf(" [accept] R cal: gain = %d/1000, offset = %d\r\n", (int)rCalGain, (int)rCalOff);
  }
#if (SPO2_OUT_MODE != 0)
  printf(" Wave output: VOFA+ FireWater text @%dHz\r\n", (int)(SPO2_FS / SPO2_VOFA_DIV));
#else
  printf(" Wave output: PCT protocol frames (0x33/0x02 wave, 0x33/0x03 int, 0x33/0x04 result)\r\n");
#endif
  printf(" UART1 baudrate = %d\r\n", UART1_BAUD_RATE);
  printf("==============================================\r\n");

  if(IsRunByPLL() == 0)
  {
    printf(" [WARN] PLL not locked! Check the 8MHz HSE crystal.\r\n");
  }
}

#if (SPO2_OUT_MODE != 0)
/*********************************************************************************************************
:* 函数名称：WrStr
:* 功能功能：把一段字符串写进串口发送队列（自动算长度，非阻塞）
:* 输入参数：s-以 '\0' 结尾的字符串
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年10月08日
**********************************************************************************************************/
static  void  WrStr(const char *s)
{
  u8 n = 0;

  while(s[n] != '\0')
  {
    n++;
  }

  if(n > 0u)
  {
    WriteUART1((u8*)s, n);           //WriteUART1 不修改内容，这里只是去 const
  }
}

/*********************************************************************************************************
:* 函数名称：WaveChValue
:* 功能功能：按通道编号取出要画的值（通道编号见 SPO2_WAVE_CH1/CH2 的说明）
:* 输入参数：ch-通道编号; r-算法结果; irAd/redAd-本拍归一后码值
:* 输出参数：void
:* 返 回 值：该通道的整数值
:* 生成日期：2026年10月08日
:* 注    意：irFilt 是浮点，×10 之后转整数 —— 一是 MicroLIB 不支持 %f，
:*           二是放大 10 倍后 1 个整数台阶 = 0.1 码，波形细节不会丢。
**********************************************************************************************************/
static  int  WaveChValue(u8 ch, const spo2_result_t *r, u16 irAd, u16 redAd)
{
  switch(ch)
  {
    case 0:  return (int)irAd;                        /* IR 原始码值(归一后) */
    case 1:  return (int)(r->irFilt * 10.0f);         /* IR 脉搏波(交流，×10) */
    case 2:  return (int)redAd;                       /* RED 原始码值(归一后) */
    case 3:  return (int)r->irAvg;                    /* IR 屏均值(直流) */
    case 4:  return (int)r->pulseRate;                /* 脉率 */
    case 5:  return (int)r->spo2;                     /* 血氧 */
    default: return 0;
  }
}

/*********************************************************************************************************
:* 函数名称：VofaWaveTick
:* 功能功能：双波形输出（每 SPO2_VOFA_DIV 个样本发一行，250Hz）
:* 输入参数：irAd-红外原始码值; redAd-红光原始码值
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年09月24日
:* 注    意：【一行只能发两个数】串口波形小工具 V1.0.5 的规则是"第1个数画波形1、
:*           第2个数画波形2、第3个数又画回波形1……遇回车换行重新计数"。
:*           原来一行发 5 个数会被错位成 波形1=[ir,red,spo2]、波形2=[irFilt,pr]，
:*           在屏上完全看不出原信号 —— 这就是波形"乱七八糟"的原因。
:*           现在严格发两个数，其余信息走"参数面板"（见 Proc1SecTask 的 [[i,v]] 命令）。
:*           带宽：250 行/s × 约 12 字节 ≈ 3KB/s，115200 下占 26%。
**********************************************************************************************************/
static  void  VofaWaveTick(u16 irAd, u16 redAd)
{
  static u8  s_iVofaDiv = 0;      //降采样计数
  const spo2_result_t* r;
  char arrLine[32];               //文本行缓冲
  int  len;                       //格式化长度

  s_iVofaDiv++;

  if(s_iVofaDiv < SPO2_VOFA_DIV)
  {
    return;
  }
  s_iVofaDiv = 0;

  r = spo2_alg_get_result(&s_spo2Alg);

  /* 双波形：两个数，逗号分隔，\r\n 结尾（工具的官方格式）*/
  len = sprintf(arrLine, "%d,%d\r\n",
                WaveChValue((u8)SPO2_WAVE_CH1, r, irAd, redAd),
                WaveChValue((u8)SPO2_WAVE_CH2, r, irAd, redAd));

  WriteUART1((u8*)arrLine, (u8)len);   //写入发送队列(非阻塞)
}

/*********************************************************************************************************
:* 函数名称：SendWaveToolParamNames
:* 功能功能：设置波形小工具右侧"参数面板"的 5 个参数名（上电只发一次）
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年10月08日
:* 注    意：工具文档明确说"参数名会被小工具保存到电脑里，不适合频繁调用"，
:*           所以只在 main() 里调一次；每秒刷新的只有参数"值"（见 Proc1SecTask）。
**********************************************************************************************************/
static  void  SendWaveToolParamNames(void)
{
  WrStr("{{1,PI}}\r\n");       /* 灌注指数 ×100（300 = 3.00%）*/
  WrStr("{{2,PR}}\r\n");       /* 脉率 bpm */
  WrStr("{{3,SpO2}}\r\n");     /* 血氧 % */
  WrStr("{{4,DAC}}\r\n");      /* 光强 DA：红外/红光 */
  WrStr("{{5,OFF}}\r\n");      /* 脱落状态/手指/探头 */
}
#endif

/*********************************************************************************************************
:* 函数名称：DumpAdTick
:* 功能功能：原始 AD 数据导出的分片发送（每 SPO2_DUMP_TICK_DIV 个节拍发一行）
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年09月24日
:* 注    意：(1) 这是"非阻塞"实现：每次只往串口发送队列写一行(约 80 字节)，
:*              队列由中断负责搬运，不占用 2ms 节拍。
:*           (2) 从"最新点"往回读（back = 0,1,2,...），原因见 spo2_config.h
:*              中 SPO2_DUMP_POINTS 处的推导：倒着读才能取到无撕裂的一致快照。
:*           (3) 启动后先空跑 SPO2_DUMP_TICK_DIV*4 拍，等之前积压的波形帧把
:*              串口队列排空，避免第一行就被挤掉。
**********************************************************************************************************/
static  void  DumpAdTick(void)
{
  char arrLine[4 + SPO2_DUMP_PER_LINE * 12 + 2];  //每对最多 "4095,4095," = 10 字节，留足余量
  u16  irAd;
  u16  redAd;
  u16  j;
  int  len;
  u8   wr;

  if(s_iDumpBusy == 0)
  {
    return;
  }

  /* BEGIN 头必须晚于命令 ACK 帧（ACK 由 ProcHostCmd 在本函数所在任务返回后才发），
   * 否则那 10 字节二进制会插在头和第一行数据之间，污染纯文本流 */
  if(s_iDumpHdrDone == 0)
  {
    s_iDumpHdrDone = 1;
    LogPrintf("\r\n===SPO2 AD DUMP BEGIN (pairs=%d, lines=%d, newest first)===\r\n",
           (int)SPO2_DUMP_POINTS, (int)(SPO2_DUMP_POINTS / SPO2_DUMP_PER_LINE));
    LogPrintf("#format: ir,red,ir,red,... x%d/line\r\n", (int)SPO2_DUMP_PER_LINE);
  }

  /* 启动前的"排空等待"：让积压的波形帧先发完，否则第一行就可能被队列丢掉 */
  if(s_iDumpSkip > 0)
  {
    s_iDumpSkip--;
    return;
  }

  s_iDumpTickDiv++;
  if(s_iDumpTickDiv < (u16)SPO2_DUMP_TICK_DIV)
  {
    return;
  }
  s_iDumpTickDiv = 0;

  len = 0;

  for(j = 0; j < (u16)SPO2_DUMP_PER_LINE; j++)
  {
    if(s_iDumpBack >= (u16)SPO2_DUMP_POINTS)
    {
      break;                                        //导完了
    }
    if(spo2_alg_get_screen_point(&s_spo2Alg, s_iDumpBack, &irAd, &redAd) == 0)
    {
      break;                                        //越界保护
    }

    len += sprintf(&arrLine[len], "%d,%d,", (int)irAd, (int)redAd);
    s_iDumpBack++;
  }

  if(len > 0)
  {
    arrLine[len - 1] = '\n';                        //把最后一个逗号换成换行
    wr = WriteUART1((u8*)arrLine, (u8)len);
    if(wr < (u8)len)
    {
      /* 队列满导致的丢字节：钳到 255，避免 u8 回绕成"没丢"的假象 */
      s_iDumpDropCnt = (u8)(s_iDumpDropCnt + ((u8)len - wr));
      if(s_iDumpDropCnt < (u8)((u8)len - wr))
      {
        s_iDumpDropCnt = 0xFFu;
      }
    }
  }

  if(s_iDumpBack >= (u16)SPO2_DUMP_POINTS)
  {
    s_iDumpBusy = 0;
    LogPrintf("===SPO2 AD DUMP END (pairs=%d, dropped=%d bytes)===\r\n",
           (int)SPO2_DUMP_POINTS, (int)s_iDumpDropCnt);
  }
}

/*********************************************************************************************************
:* 函数名称：ProcWaveData
:* 功能功能：一个采样节拍的全部工作：采一对点 → 跑算法 → 写光强 → 上报
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年09月24日
:* 注    意：(1) 不论上传开关是否打开都要把算法跑完，否则屏缓冲/光强标定会断；
:*              开关只决定"要不要发帧"。
:*           (2) 光强 DA 的写入必须发生在下一次 SPO2FE_ReadPair() 之前 ——
:*              ReadPair 内部选通通道时会把对应的 DA 写到 DAC，
:*              所以这里算完就写，下一拍生效。
**********************************************************************************************************/
static  void  ProcWaveData(void)
{
  const spo2_result_t* r;
  u16 irAd;
  u16 redAd;

  /* ---- 1. 采集成对点（内部：选通IR→等建立→转→选通RED→等建立→转） ---- */
  if(SPO2FE_ReadPair(&irAd, &redAd) == 0)
  {
    return;
  }

#if (SPO2_OUT_MODE == 0)
  s_iTickCnt++;                           //仅协议模式用它做"每秒增 500"的自检
#endif

  /* ---- 2. 跑血氧算法（滤波→脉率→屏缓冲→光强调节→满屏时算血氧） ---- */
  spo2_alg_step(&s_spo2Alg, irAd, redAd);
  r = spo2_alg_get_result(&s_spo2Alg);

  /* ---- 3. 把算法给出的光强 DA 写到前端（下一拍的 ReadPair 生效） ---- */
  SPO2FE_SetIntensityDa(r->irDa, r->redDa);

  /* ---- 4. 上报 ----
   * 注意：AD 数据导出期间停发波形帧 —— 导出的是纯文本行，混进波形帧会让
   * "把串口收到的内容直接存成 csv"这一步变得很麻烦；导出只持续 1.28s，
   * 停发对波形观察几乎无影响。*/
  if(s_iDumpBusy == 0)
  {
    if(GetWaveSendEnable())
    {
      VofaWaveTick(irAd, redAd);         //VOFA+ 文本帧
    }
  }

  /* ---- 5. 原始 AD 导出（每 SPO2_DUMP_TICK_DIV 拍发一行，非阻塞） ---- */
  DumpAdTick();
}

/*********************************************************************************************************
:* 函数名称：Proc2msTask
:* 功能功能：2ms任务处理
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2018年01月01日
:* 注    意：
**********************************************************************************************************/
static  void  Proc2msTask(void)
{
  u8  uart1RecData; //串口数据
  u32 start;
  u32 us;

  if(Get2msFlag())  //判断2ms标志状态
  {
    start = GetCycleCount();            //拍起点：实测"满负荷一拍"到底吃掉多少时间

    while(ReadUART1(&uart1RecData, 1))  //队列中有接收数据
    {
      ProcHostCmd(uart1RecData);        //处理命令
    }

    ProcWaveData();                     //采集 + 算法 + 上报

    LEDFlicker(250);                    //每500ms翻转一次LED，表示主循环在跑

    us = CycleToUs(GetCycleCount() - start);
    if(us > s_iTickMaxUs)
    {
      s_iTickMaxUs = us;                //峰值保持：只有超预算的那一拍会被留下痕迹
    }

    Clr2msFlag();                       //清除2ms标志
  }
}

/*********************************************************************************************************
:* 函数名称：Proc1SecTask
:* 功能功能：1s任务处理
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2018年01月01日
:* 注    意：这一行/这一帧就是整机的自检：节拍数每秒应增 500，timeout/ovr 应恒为 0
**********************************************************************************************************/
static  void  Proc1SecTask(void)
{
  const spo2_result_t* r;
#if (SPO2_OUT_MODE == 0)
  u16 fundHzX100;                 //仅协议模式用（结果帧里的主频×100）
  u8  statFlags;                  //仅协议模式用（0x05 指标帧里的状态位）
#endif

  if(Get1SecFlag()) //判断1s标志状态
  {
    r = spo2_alg_get_result(&s_spo2Alg);
#if (SPO2_OUT_MODE == 0)
    fundHzX100 = (u16)(r->fundHz * 100.0f + 0.5f);

    /* 状态位打包（装进 0x05 指标帧的 arrData[3]）：
     *   bit0 PI 有效   bit1 手指在   bit2 探头在   bit3 粗调完成 */
    statFlags = (u8)(((r->piValid != 0u) ? 0x01u : 0x00u)
                   | ((r->fingerOn != 0u) ? 0x02u : 0x00u)
                   | ((r->probeOn  != 0u) ? 0x04u : 0x00u)
                   | ((r->adjDone  != 0u) ? 0x08u : 0x00u));
#endif

#if (SPO2_OUT_MODE != 0)
    {
      /* ---- 参数面板：用工具的 [[序号,值]] 命令刷新右侧 5 个参数 ----
       * 【为什么不用自由文本】工具会把数据流里**所有数字**都当成波形点，
       * 所以这个模式下除了"双波形那一行"，其余数字都必须包在 {{...}} / [[...]] 里，
       * 否则每秒一行状态文本就会在波形上打出一串假点。
       * 需要看完整数字（conv/to/ovr/avg/rVal/fund 等）时，发 0x84 查状态即可。*/
      char arrStat[64];
      int  statLen;

      statLen = sprintf(arrStat, "[[1,%d]]\r\n", (int)r->piIrX100);          /* PI×100 */
      WriteUART1((u8*)arrStat, (u8)statLen);

      statLen = sprintf(arrStat, "[[2,%d]]\r\n", (int)r->pulseRate);         /* 脉率 */
      WriteUART1((u8*)arrStat, (u8)statLen);

      statLen = sprintf(arrStat, "[[3,%d]]\r\n", (int)r->spo2);              /* 血氧 */
      WriteUART1((u8*)arrStat, (u8)statLen);

      statLen = sprintf(arrStat, "[[4,%d/%d]]\r\n",
                        (int)r->irDa, (int)r->redDa);                        /* 光强 DA */
      WriteUART1((u8*)arrStat, (u8)statLen);

      statLen = sprintf(arrStat, "[[5,%d/%d/%d]]\r\n",
                        (int)r->offState, (int)r->fingerOn, (int)r->probeOn); /* 状态 */
      WriteUART1((u8*)arrStat, (u8)statLen);
    }
#else
    /* 上位机协议模式：每秒回一次"血氧结果帧"(0x04) + "血氧指标帧"(0x05)，
     * 让上位机/串口助手既能看到血氧脉率，也能看到 PI 与脱落状态 */
    SendSPO2ResultToHost(r->spo2, r->pulseRate, fundHzX100);

    SendSPO2MetricToHost(r->piIrX100, r->offState, statFlags, (u16)(r->rVal + 0.5f));
#endif

    Clr1SecFlag();  //清除1s标志
  }
}

/*********************************************************************************************************
:*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：SetWaveSendEnable
* 功能功能：设置波形数据帧上传开关
* 输入参数：en，1-上传，0-停止
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：
**********************************************************************************************************/
void  SetWaveSendEnable(u8 en)
{
  s_iWaveSendEnable = (en == 0) ? 0 : 1;
}

/*********************************************************************************************************
* 函数名称：GetWaveSendEnable
* 功能功能：获取波形数据帧上传开关
* 输入参数：void
* 输出参数：void
* 返 回 值：开关状态
* 生成日期：2026年09月24日
* 注    意：
**********************************************************************************************************/
u8  GetWaveSendEnable(void)
{
  return(s_iWaveSendEnable);
}

/*********************************************************************************************************
* 函数名称：GetSPO2Result
* 功能功能：读取血氧算法最新结果
* 输入参数：void
* 输出参数：void
* 返 回 值：结果结构体指针(只读)
* 生成日期：2026年09月24日
* 注    意：
**********************************************************************************************************/
const spo2_result_t* GetSPO2Result(void)
{
  return spo2_alg_get_result(&s_spo2Alg);
}

/*********************************************************************************************************
* 函数名称：SPO2AlgReset
* 功能功能：血氧算法整机复位（清屏 + 重新标定）
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：对应 MATLAB 停止采样后重新 InitData() + ResetAdj() 的动作
**********************************************************************************************************/
void  SPO2AlgReset(void)
{
  spo2_alg_reset(&s_spo2Alg);

#if (SPO2_OUT_MODE == 0)
  s_iTickCnt = 0;
#endif

}

/*********************************************************************************************************
* 函数名称：SPO2SetManualLight
* 功能功能：上位机接管光强（对应上位机 SendAdjCmd 下发的调光命令）
* 输入参数：irDa-红外光强DA; redDa-红光光强DA
* 输出参数：void
* 返 回 值：void
* 生成日期：2026年09月24日
* 注    意：立即写到前端，不必等下一次 ReadPair
**********************************************************************************************************/
void  SPO2SetManualLight(u16 irDa, u16 redDa)
{
  spo2_alg_set_manual_light(&s_spo2Alg, irDa, redDa);

  SPO2FE_SetIntensityDa(irDa, redDa);
}

/*********************************************************************************************************
:* 函数名称：SPO2DumpAdStart / SPO2DumpAdIsBusy / SPO2DumpAdGetDropCnt
:* 功能功能：原始 AD 数据导出（验收前置条件"各参数需要至少采样1000个真实AD数据"）
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：见下
:* 生成日期：2026年09月24日
:* 注    意：导出内容 = 屏缓冲里最新的 SPO2_DUMP_POINTS(1024) 对 IR/RED 原始码值，
:*           每行 8 对，格式 "ir1,red1,ir2,red2,...,ir8,red8"，从最新点倒序。
:*           把两行 BEGIN/END 之间的内容存成 .csv 即可直接进实验报告。
:*           整个导出非阻塞（每 10ms 发一行），不影响采样与算法节拍。
**********************************************************************************************************/
void  SPO2DumpAdStart(void)
{
  s_iDumpBack    = 0;
  s_iDumpTickDiv = 0;
  s_iDumpDropCnt = 0;
  s_iDumpSkip    = (u16)(SPO2_DUMP_TICK_DIV * 4u);   //先等 4 行的时间把串口队列排空
  s_iDumpHdrDone = 0;
  s_iDumpBusy    = 1;

  /* 注意：这里**故意不打印** BEGIN 头。命令处理器返回后，ProcHostCmd 会紧接着
   * 下发一帧 10 字节的二进制 ACK，若此刻已经打了文本头，ACK 的二进制字节就会
   * 插在"BEGIN"和第一行数据之间，把纯文本流搞脏。所以头改由 DumpAdTick 在
   * "ACK 已经发出去之后"的第一拍打印，见下面的 s_iDumpHdrDone 分支。*/
}

u8  SPO2DumpAdIsBusy(void)
{
  return s_iDumpBusy;
}

u8  SPO2DumpAdGetDropCnt(void)
{
  return s_iDumpDropCnt;
}

/*********************************************************************************************************
:* 函数名称：SPO2SetRCal / SPO2GetRCal
:* 功能功能：R 值现场标定（验收"血氧误差 ±2%"的唯一调节手段）
:* 输入参数：gainPm-千分比增益(500~1500); offset-绝对偏置(-300~300)
:* 输出参数：pGainPm, pOffset
:* 返 回 值：1-成功; 0-参数越界
:* 生成日期：2026年09月24日
:* 注    意：标定后立即生效（不需要复位），且参数跨复位保留
**********************************************************************************************************/
u8  SPO2SetRCal(i16 gainPm, i16 offset)
{
  return spo2_alg_set_r_cal(&s_spo2Alg, gainPm, offset);
}

void  SPO2GetRCal(i16* pGainPm, i16* pOffset)
{
  spo2_alg_get_r_cal(&s_spo2Alg, pGainPm, pOffset);
}

/*********************************************************************************************************
* 函数名称：main
* 功能功能：主函数
* 输入参数：void
* 输出参数：void
* 返 回 值：int
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
int main(void)
{
  InitSoftware();   //初始化软件相关函数

  InitHardware();   //初始化硬件相关函数

  PrintBootInfo();  //打印上电自检信息

#if (SPO2_OUT_MODE != 0)
  /* 配置"串口波形小工具"右侧参数面板的 5 个变量名：
   * 只在上位电发一次(工具会把变量名存到本地，频繁发送没有意义) */
  SendWaveToolParamNames();
#endif

  while(1)
  {
    Proc2msTask();          //2ms采样任务(采集+算法+上报，必须守住 2000us 预算)
    Proc1SecTask();         //1s任务(结果帧/指标帧/参数面板)
    ProcSPO2HeavyTask();    //屏级重算：满一屏才有一次活，落在 2ms 拍之外执行
  }
}

/*********************************************************************************************************
:* 函数名称：ProcSPO2HeavyTask
:* 功能功能：屏级重算(屏统计 + R 值 + PI)的执行钩子，在主循环空闲时调用
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：void
:* 生成日期：2026年10月09日
:* 注    意：(1) 为什么不能再留在 2ms 拍里：屏统计在 168MHz 下要数百 us 级，
:*               留在拍内会把"IR 采完到 RED 采完"拉开好几毫秒，两个通道不再成对
:*               同时刻，R 值与 PI 的口径直接被污染 —— 正是血氧 ±2% 最吃不起的误差。
:*           (2) 挪出来之后：一拍仍然只做"选通→建立→转换→滤波→脉率→入缓冲"，
:*               IR/RED 保持紧邻；整拍至多晚开始一次，晚了多少由 Timer 的 miss 计数
:*               和这里的 heavyMax 一起给出证据。
:*           (3) 与串口导出的关系：导出期间 2ms 拍照跑，重算同样在空闲时被取走，
:*               两者互不阻塞；若主循环长期取不走(lostScreen 增长)才需要减慢输出。
*********************************************************************************************************/
static  void  ProcSPO2HeavyTask(void)
{
  u32 start;
  u32 us;

  if(spo2_alg_screen_pending(&s_spo2Alg) == 0u)
  {
    return;                         //绝大多数循环圈都走这条快路：一次读 + 一次比较
  }

  start = GetCycleCount();

  spo2_alg_screen_run(&s_spo2Alg);

  us = CycleToUs(GetCycleCount() - start);

  s_iHeavyRunCnt++;

  if(us > s_iHeavyMaxUs)
  {
    s_iHeavyMaxUs = us;             //这就是"屏级重算到底吃掉多少时间"的直接实测值
  }
}

/*********************************************************************************************************
:* 函数名称：SPO2GetTickMaxUs / SPO2GetHeavyMaxUs / SPO2GetHeavyRunCnt
:*           SPO2GetScreenLostCnt / SPO2ClrTickStat
:* 功能功能：实时性统计量的读与清零(供 ProcHostCmd 的 0x84 状态打印与 0x88 清零调用)
:* 输入参数：void
:* 输出参数：void
:* 返 回 值：对应统计量
:* 生成日期：2026年10月09日
:* 注    意：清零只做一次 32 位写，放在 2ms 任务里执行不会打断采样时序。
*********************************************************************************************************/
u32  SPO2GetTickMaxUs(void)
{
  return(s_iTickMaxUs);
}

u32  SPO2GetHeavyMaxUs(void)
{
  return(s_iHeavyMaxUs);
}

u32  SPO2GetHeavyRunCnt(void)
{
  return(s_iHeavyRunCnt);
}

u32  SPO2GetScreenLostCnt(void)
{
  return(spo2_alg_get_screen_lost_cnt(&s_spo2Alg));
}

void SPO2ClrTickStat(void)
{
  s_iTickMaxUs   = 0;
  s_iHeavyMaxUs  = 0;
  s_iHeavyRunCnt = 0;

  s_spo2Alg.screenLostCnt = 0;        //丢屏计数直接清(实例是 Main.c 的文件内静态)
}
