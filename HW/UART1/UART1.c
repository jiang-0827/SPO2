/*********************************************************************************************************
* 模 块 名 称：UART1.c
* 摘        要：UART1模块，USART1初始化、收发环形缓冲区、中断服务函数
* 当前版本：1.0.0
* 作        者：SZLY(COPYRIGHT 2018 - 2026 SZLY. All rights reserved.)
* 生成日期：2026年09月20日
* 说        明：本文件由STM32F103版本移植而来
* 注        意：F4系列GPIO需要额外配置复用功能(GPIO_PinAFConfig)且GPIO时钟在AHB1总线
**********************************************************************************************************
* 取消版本：
* 作        者：
* 生成日期：
* 修改内容：
* 修改文件：
*********************************************************************************************************/
/*********************************************************************************************************
*                                              包含头文件
*********************************************************************************************************/
#include <stdio.h>
#include "UART1.h"
#include "Queue.h"
#include "stm32f4xx_conf.h"

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
//串口发送状态
typedef enum
{
  UART_STATE_OFF, //串口未发送数据
  UART_STATE_ON,  //串口正在发送数据
  UART_STATE_MAX
}EnumUARTState;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
static  StructCirQue s_structUARTSendCirQue;  //串口发送循环队列
static  StructCirQue s_structUARTRecCirQue;   //串口接收循环队列
static  u8  s_arrSendBuf[UART1_BUF_SIZE];     //串口发送循环队列中的缓冲区
static  u8  s_arrRecBuf[UART1_BUF_SIZE];      //串口接收循环队列中的缓冲区

static  u8  s_iUARTTxSts;                     //串口发送数据状态

/*********************************************************************************************************
*                                              内部函数声明
*********************************************************************************************************/
static  void  InitUARTBuf(void);      //初始化串口缓冲区（发送缓冲区和接收缓冲区）
static  u8    WriteReceiveBuf(u8 d);  //把接收到的数据写入接收缓冲区
static  u8    ReadSendBuf(u8 *p);     //读取发送缓冲区中的数据

static  void  ConfigUART(u32 bound);  //配置串口相关的参数（包括GPIO、RCC、USART、NVIC）
static  void  EnableUARTTx(void);     //使能串口发送（WriteUARTx中调用，每次发送完数据之后需要关闭）

static  void  SendCharUsedByFputc(u16 ch);  //发送字符函数（专供fputc函数使用）

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitUARTBuf
* 功能功能：初始化串口缓冲区（发送缓冲区和接收缓冲区）
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
static  void  InitUARTBuf(void)
{
  i16 i;

  for(i = 0; i < UART1_BUF_SIZE; i++)
  {
    s_arrSendBuf[i] = 0;
    s_arrRecBuf[i]  = 0;
  }

  InitQueue(&s_structUARTSendCirQue, s_arrSendBuf, UART1_BUF_SIZE);
  InitQueue(&s_structUARTRecCirQue,  s_arrRecBuf,  UART1_BUF_SIZE);
}

/*********************************************************************************************************
* 函数名称：WriteReceiveBuf
* 功能功能：写数据到串口接收缓冲区
* 输入参数：d，待写入串口接收缓冲区的数
* 输出参数：void
* 返 回 值：写入数据成功标志：0-不成功，1-成功
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
static  u8  WriteReceiveBuf(u8 d)
{
  u8 ok = 0;  //写入数据成功标志：0-不成功，1-成功

  ok = EnQueue(&s_structUARTRecCirQue, &d, 1);

  return ok;  //返回写入数据成功标志：0-不成功，1-成功
}

/*********************************************************************************************************
* 函数名称：ReadSendBuf
* 功能功能：读取串口发送缓冲区中的数据
* 输入参数：p，读出数据存放的首地址
* 输出参数：void
* 返 回 值：读取数据成功标志：0-不成功，1-成功
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
static  u8  ReadSendBuf(u8 *p)
{
  u8 ok = 0;  //读取数据成功标志：0-不成功，1-成功

  ok = DeQueue(&s_structUARTSendCirQue, p, 1);

  return ok;  //返回读取数据成功标志：0-不成功，1-成功
}

/*********************************************************************************************************
* 函数名称：ConfigUART
* 功能功能：配置串口相关的参数（包括GPIO、RCC、USART、NVIC）
* 输入参数：bound，波特率
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：与F103的差异有三点：
*           1、GPIO时钟在AHB1(F1在APB2)；
*           2、PA9/PA10要先设置复用功能映射GPIO_AF_USART1，模式为GPIO_Mode_AF；
*           3、USART1仍在APB2总线，但F407的PCLK2=84MHz(F103为72MHz)，波特率分频由库函数
*              内部按RCC_GetClocksFreq()自动计算，故USART_InitStructure无需修改
**********************************************************************************************************/
static  void  ConfigUART(u32 bound)
{
  GPIO_InitTypeDef  GPIO_InitStructure;   //GPIO_InitStructure用于存GPIO的参数
  USART_InitTypeDef USART_InitStructure;  //USART_InitStructure用于存USART的参数
  NVIC_InitTypeDef  NVIC_InitStructure;   //NVIC_InitStructure用于存NVIC的参数

  //使用RCC模块时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);   //使能GPIOA时钟(F4的GPIO在AHB1)
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  //使能USART1的时钟

  //配置TX、RX引脚的复用功能
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);   //PA9复用为USART1_TX
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);   //PA10复用为USART1_RX

  //配置TX的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;               //定义TX引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;             //复用功能模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽输出
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;             //上拉
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO口速度
  GPIO_Init(GPIOA, &GPIO_InitStructure);                    //根据参数初始化TX的GPIO

  //配置RX的GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;              //定义RX引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;             //复用功能模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            //推挽输出(对输入无影响)
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;             //上拉，悬空时为高，避免误接收
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         //IO口速度
  GPIO_Init(GPIOA, &GPIO_InitStructure);                    //根据参数初始化RX的GPIO

  //配置USART的参数
  USART_StructInit(&USART_InitStructure);                   //初始化USART_InitStructure
  USART_InitStructure.USART_BaudRate   = bound;             //设置波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;   //设置串口字符长度
  USART_InitStructure.USART_StopBits   = USART_StopBits_1;  //设置停止位
  USART_InitStructure.USART_Parity     = USART_Parity_No;   //设置无偶校验位
  USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;           //设置模式
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //设置硬件流控模式
  USART_Init(USART1, &USART_InitStructure);                 //根据参数初始化USART1

  //配置NVIC
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         //中断通道号
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //设置抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; //设置响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //使能中断
  NVIC_Init(&NVIC_InitStructure);                           //根据参数初始化NVIC

  //使能USART1中断
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  //使能接收缓冲区非空中断
  USART_ITConfig(USART1, USART_IT_TXE,  DISABLE); //发送缓冲区空中断先关闭，有数据要发时再开
  USART_Cmd(USART1, ENABLE);                      //使能USART1

  s_iUARTTxSts = UART_STATE_OFF;  //串口发送数据状态设置为未发送数据
}

/*********************************************************************************************************
* 函数名称：EnableUARTTx
* 功能功能：使能串口发送（在WriteUARTx中调用，每次发送完数据之后需要关闭，即使能发送中断）
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：s_iUARTTxSts = UART_STATE_ON;这句要放在USART_ITConfig之前，否则会导致中断触发无法执行
**********************************************************************************************************/
static  void  EnableUARTTx(void)
{
  s_iUARTTxSts = UART_STATE_ON;                     //串口发送数据状态设置为正在发送数据

  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);     //使能发送中断
}

/*********************************************************************************************************
* 函数名称：SendCharUsedByFputc
* 功能功能：发送字符函数（专供fputc函数使用）
* 输入参数：ch，待发送的字符
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：printf走的是这条阻塞路径，等的是TC(发送完成)而不是TXE，保证最后一个字符
*           不会卡在移位寄存器里与后续的协议帧混在一起
**********************************************************************************************************/
static  void  SendCharUsedByFputc(u16 ch)
{
  USART_SendData(USART1, (u8)ch);

  //等待发送完成
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {

  }
}

/*********************************************************************************************************
* 函数名称：USART1_IRQHandler
* 功能功能：USART1中断服务函数
* 输入参数：void
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void USART1_IRQHandler(void)
{
  u8  uData = 0;

  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收缓冲区非空中断
  {
    NVIC_ClearPendingIRQ(USART1_IRQn);  //清除USART1中断挂起
    uData = USART_ReceiveData(USART1);  //把USART1接收到的数据保存到uData

    WriteReceiveBuf(uData);  //把接收到的数据写入接收缓冲区
  }

  if(USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET) //溢出标志为1
  {
    USART_ClearFlag(USART1, USART_FLAG_ORE);             //清除溢出标志
    USART_ReceiveData(USART1);                           //读取USART_DR
  }

  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)    //发送缓冲区空中断
  {
    USART_ClearITPendingBit(USART1, USART_IT_TXE);       //清除发送中断标志
    NVIC_ClearPendingIRQ(USART1_IRQn);                   //清除USART1中断挂起

    if(ReadSendBuf(&uData))                              //读发送缓冲区数据到uData，读到才发送
    {
      USART_SendData(USART1, uData);                     //把uData写入USART_DR
    }

    if(QueueEmpty(&s_structUARTSendCirQue))              //当发送缓冲区为空时
    {
      s_iUARTTxSts = UART_STATE_OFF;                     //串口发送数据状态设置为未发送数据
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);     //关闭串口发送缓冲区空中断
    }
  }
}

/*********************************************************************************************************
*                                              API函数实现
*********************************************************************************************************/
/*********************************************************************************************************
* 函数名称：InitUART1
* 功能功能：初始化UART模块
* 输入参数：bound，波特率
* 输出参数：void
* 返 回 值：void
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
void InitUART1(u32 bound)
{
  InitUARTBuf();        //初始化串口缓冲区（发送缓冲区和接收缓冲区）

  ConfigUART(bound);    //配置串口相关的参数（包括GPIO、RCC、USART、NVIC）
}

/*********************************************************************************************************
* 函数名称：WriteUART1
* 功能功能：写串口（把写入数据到串口发送缓冲区）
* 输入参数：pBuf，要写入数据的首地址；len，需要写入数据的个数
* 输出参数：void
* 返 回 值：成功写入数据的个数（一般小于等于len）
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
u8  WriteUART1(u8 *pBuf, u8 len)
{
  u8 wLen = 0;  //实际写入数据的个数

  __disable_irq();  //临界区: 防与TXE中断并发修改队列计数(elemNum)
  wLen = (u8)EnQueue(&s_structUARTSendCirQue, pBuf, len);
  __enable_irq();

  if(wLen < UART1_BUF_SIZE)
  {
    if(s_iUARTTxSts == UART_STATE_OFF)
    {
      EnableUARTTx();
    }
  }

  return wLen;  //返回实际写入数据的个数
}

/*********************************************************************************************************
* 函数名称：ReadUART1
* 功能功能：读串口（读取串口接收缓冲区中的数据）
* 输入参数：pBuf，读取数据存放的首地址；len，希望读取数据的个数
* 输出参数：pBuf，读取数据存放的首地址
* 返 回 值：成功读取数据的个数（一般小于等于len）
* 生成日期：2018年01月01日
* 注    意：
**********************************************************************************************************/
u8  ReadUART1(u8 *pBuf, u8 len)
{
  u8 rLen = 0;  //实际读取数据长度

  __disable_irq();  //临界区: 防与接收中断并发修改队列计数
  rLen = (u8)DeQueue(&s_structUARTRecCirQue, pBuf, len);
  __enable_irq();

  return rLen;  //返回实际读取数据的长度
}

/*********************************************************************************************************
* 不使用MicroLIB时，需要下面这几个函数把C库的printf重定向到USART1
*********************************************************************************************************/
#if !defined(__MICROLIB)
//如果使用微库则不需要下面这些函数
#if (__ARMCLIB_VERSION <= 6000000)
//兼容AC5及更早版本的编译器结构体
struct __FILE
{
  int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
  x = x;
}
#endif

/*********************************************************************************************************
* 函数名称：fputc
* 功能功能：重定向printf
* 输入参数：ch，f
* 输出参数：void
* 返 回 值：int
* 生成日期：2018年01月01日
* 注    意：工程需勾选Options for Target->Target->Use MicroLIB，或在上面做非微库适配
**********************************************************************************************************/
int fputc(int ch, FILE* f)
{
  SendCharUsedByFputc((u8) ch);  //调用字符发送函数（专供fputc函数使用）

  return ch;                     //返回ch
}
