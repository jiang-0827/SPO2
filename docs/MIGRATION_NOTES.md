> ### ⚠️ 时效声明（2026-10-10 代码精简之后）
>
> 本文是血氧 F407 工程的**移植与设计长文**（原为仓库根目录 `README.md`），记录了"从 MATLAB 血氧工程 + 心电 F407 工程移植过来"的完整依据，仍然值得读。
> 但 2026-10-10 做了一轮减法，以下几节已经**与当前代码不符**，请以根目录 `README.md` 和代码为准：
>
> | 本文章节 | 现状 |
> |---|---|
> | §1.5 `SPO2_CAP`（H4-5）基线复位开关 | **整条链已移除**（`SPO2_CAP_EN` 原为 0、代码从未生效）。板级结论保留在 `Alg/spo2_config.h` 的 10 行注释里 |
> | §1.5 / §6 / §9 / §11 / §12.4 中所有 "FFT / 幅度谱 / 主频" 相关内容 | **`Alg/spo2_fft.c/h` 已移出工程**；结果帧 `DAT_SPO2_RESULT` 的 `fundHz` 两字节保留但恒为 0 |
> | §2 目录结构、§3 新增文件清单 | 少了 `spo2_fft.c/.h`；`app/main.c`、`board/`、`bsp/`（原厂示例，未参与编译）已移出主树 |
> | §7 / §14.8 尺寸数据 | 已刷新为 ROM 28.2KB / **RAM 11.88KB**（原 32.4KB，FFT 工作区 20KB 被释放） |
> | §12 / §14.7 命令清单 | 命令从 11 条减为 10 条，去掉 `CMD_SPO2_SET_CAP(0x8C)` |
>
> 另外本文 §1.2、§9、§12 里标注 `[实测]` 的参数**仍未标定完成**（`SPO2_PR_ABS_MIN`、`SPO2_PR_THR_ALPHA`、R 值 gain/offset、PI 口径、`SPO2_FE_SETTLE_US`），这些不是文档过时，而是确实还没做完 —— 见根 README 的"验收状态"表。


---

# 血氧算法 STM32F407 工程（由 MATLAB 血氧工程 + 心电 F407 工程移植而来）

本文档说明：**血氧算法从 MATLAB(SPO2DynamicPrj) 到 STM32F407 的完整移植**——新增了哪些文件、
修改了哪些文件、Keil 工程配置改了哪几项、以及从零到跑通的测试步骤。

- 算法原型：`SPO2DynamicPrj/`（MATLAB，血氧上位机软件 + 算法）
- 移植模板：`D:\SZU\心电\算法\STM\心率采集算法final\STM32F407_ProjectTemplate`（心电 F407 工程）
- 目标工程：本目录
- MCU：立创·天空星 STM32F407VET6（Cortex-M4F @168MHz，单精度硬件 FPU）
- 编译验证：Keil MDK AC5 命令行全量编译 + 链接，**30 个源文件 + 启动文件 0 error 0 warning**，
  链接通过（普通库与 MicroLIB 两种配置都验证过），
  已生成可直接烧录的 `SPO2_Algorithm.hex`
- **验收：已按《五参数验收标准》里的“血氧验收标准”逐条实现，对照表与操作流程见第 12 节**

---

## 1. 血氧板硬件接口（移植的物理前提）

血氧板（`SCH_Schematic1_2026-09-22`，PD 光电管 + LMV324 跨阻/滤波 + SGM3005 模拟开关选通
IR/RED 双光源 + LMV321 缓冲，板上无 MCU）通过 **H4 “单片机引脚” 7pin 排针**与 MCU 相连：

| H4 引脚 | 网络名 | 方向 | 本工程 MCU 侧分配 | 说明 |
|---|---|---|---|---|
| 1 | GND | — | **GND** | 共地（必须接） |
| 2 | +5V | 输入 | **5V** | 血氧板自带 RT9193 转 3V3A，**MCU 不要给 3V3** |
| 3 | SPO2_RED_CS | MCU→板 | **PC5**（GPIO 推挽输出） | 红光通道选通（SGM3005 INx 高有效） |
| 4 | SPO2_VSIG | 板→MCU | **PA5**（ADC12_IN5，模拟输入） | 脉搏波模拟输出 |
| 5 | **SPO2_CAP** | MCU→板 | **PC6**（GPIO 推挽输出） | **基线复位开关**（2026-10-08 版新增，见 1.5） |
| 6 | SPO2_IR_CS | MCU→板 | **PC4**（GPIO 推挽输出） | 红外通道选通 |
| 7 | SPO2_DAI | MCU→板 | **PA4**（DAC_OUT1） | 光强控制（板上 LMV321 缓冲 → LED 驱动） |

**共 5 根信号线 + 2 根电源线，一个 7pin 排线直接对接即可。**

> ⚠️ 注意 **H4 引脚号是"从下往上"排的**（原理图里 1 在下面、7 在上面）：
> `1=GND, 2=+5V, 3=RED_CS, 4=VSIG, 5=CAP, 6=IR_CS, 7=DAI`。
> 焊排线前先用万用表对一下 1 脚（GND）和 2 脚（+5V），接反会烧板。
>
> 引脚宏集中在三处，接法不同时只改这三处：
> `HW/ADC/ADC.h`（SPO2_VSIG 通道）、`HW/SPO2FE/SPO2FE.h`（DAC + 两个 CS + CAP）。
> 注意 PA4/PA5 同时是 DAC_OUT1/DAC_OUT2，本工程 **PA4 给 DAC、PA5 给 ADC**，互不冲突。

MCU 侧另外还要接的（不属于 H4）：

| 功能 | 引脚 | 说明 |
|---|---|---|
| 串口（调试/上位机） | **PA9(TX) / PA10(RX)** | USART1，接 USB-TTL；115200 8N1 |
| 板载指示灯 | **PB2** | 每 500ms 翻转，证明主循环没卡死 |
| 下载/调试 | **PA13(SWDIO) / PA14(SWCLK)** | ST-Link / DAP-Link |

> **接线上机前请自查**：本工程的引脚假设是"PA4/PA5/PC4/PC5/PC6 在所用核心板上都引到排针、且没有被板载外设占用"。
> 立创·天空星（F407VET6）的板载用户 LED 在 PB2、USB 转串口在 PA9/PA10，与上述 5 个脚不冲突；
> 若换用其它核心板，请对照该板原理图确认这 5 个脚是否被板载 Flash / SD / RGB 灯之类占用。

### 1.1 采集时序（本工程的核心时序，决定了为什么不能用“定时器触发+DMA”）

H4 只有一路模拟输出 VSIG，IR/RED 靠两个 CS 脚**分时复用**，所以一个 2ms 采样节拍里必须：

```
选通 IR(PC4=1,PC5=0,同时把IR的DA写到DAC) → 等前端建立(SPO2_FE_SETTLE_US=100us)
    → ADC 连转 SPO2_ADC_OS=4 次取平均 → 得到 IR 码值
选通 RED(PC4=0,PC5=1,同时把RED的DA写到DAC) → 等前端建立(100us)
    → ADC 连转 4 次取平均 → 得到 RED 码值
```

一次节拍总开销 ≈ 2×(100us 建立 + 4×23us 转换) ≈ 384us，占 2ms 预算的 **19%**，仍然充裕。
（4 次过采样平均把随机噪声降到 1/2，见 1.4；这是为验收“滤除随机噪声”与“血氧 ±2%”加的。）
IR/RED 各得 500 SPS，与 MATLAB 侧 `Fs = 500`（`FilterPulseWave.m` / `CalcAmpSpec.m`）严格对齐。

### 1.2 前端极性：`SPO2_FE_INVERT`（**本板已实测确定为 0**）

算法内部统一约定 **“光越强 → 归一码值越小”**（这样才与 MATLAB 里 `IR_CENTRAL_LINE = 2000`、
“AD 值 < 80 判手指脱落”的隐含约定一致）。

`SPO2_FE_INVERT` 决定要不要把原始 ADC 翻一次：

| 设置 | 归一码值 | 适用条件 |
|---|---|---|
| **0** | = 原始 ADC | 原始极性本来就是“码值小 = 光强” |
| 1 | = 4095 − 原始 ADC | 原始极性是“码值大 = 光强” |

**本工程实测结论：必须为 0。** 依据（`wave.csv`，手指拿掉那一段）：

| 状态 | `SPO2_FE_INVERT=1` 时看到的 `irAd` | 换算原始 ADC |
|---|---|---|
| 手指夹着 | ≈ 750 | 4095 − 750 = 3345（偏暗） |
| 手指拿掉（LED 直射光电管） | ≈ 4000 | 4095 − 4000 = **95**（很亮） |

即“光越强 → 原始 ADC 越小”，**本来就是统一的，再翻一次反而搞反**。设错会导致三个连锁故障：

1. **光强闭环变成正反馈**：IR 归一 285 被当成“光已够足”而停调（实际很暗），
   RED 归一 3997 被当成“光不够”而把 DA 一路顶到 250（实际过曝），`adjDone` 永远不置 1；
2. **脉搏波方向翻转**，画出来是朝下的尖峰；
3. **两个脱落判据互相对调**：无手指（raw≈95）被当成“无光”，无光（raw≈4000）
   被当成“光过强”，报出来的脱落类型正好相反。

不改代码也能在线试（复位后回到宏里的值）：
`取反` → `33 81 86 81 80 80 80 80 80 BB`；`不取反` → `33 81 86 80 80 80 80 80 80 BA`。

### 1.4 降低随机噪声：ADC 过采样（对应验收“滤除随机噪声”）

`SPO2_ADC_OS = 4`：每个通道在同一个采样节拍内连转 4 次求平均。**不改 500SPS 采样率、
不引入相位延迟**（区别于“靠滤波器降噪”），只把白噪声压到 1/√4 = 1/2（+6dB，等效多约 1bit）。

为什么必须做：验收标准里模拟器灌注指数固定 **3%**，直流调到中线 2000 码时交流峰峰值只有
≈120 码，单次 12bit 转换的量化台阶 1LSB 就有 ~0.8% 的相对误差，两次通道误差叠加后
会显著吃掉“血氧 ±2%”的预算。

代价是每拍多约 134us（4 次转换 vs 1 次），见 1.1 的开销核算。若你的前端建立时间更慢、
2ms 预算紧张，把 `SPO2_ADC_OS` 降到 2 即可（降噪 ×0.7）。

### 1.5 新增的 `SPO2_CAP`（H4-5）：前端基线复位开关

**这一脚是 2026-10-08 版血氧板新增的**（旧版 H4-5 是 NC），本工程已配套实现驱动。

硬件结构（`SCH_Schematic1_2026-10-08` 的"信号放大滤波电路"）：

```
TIA(U1.2) ── C8 100nF ──┬── R8 10kΩ ── U1.3(+)
                         └── R23 270Ω ── Q1(AO3401A, P-MOS) ── VREF(1.65V)
                                              ↑ 栅极 = SPO2_CAP = H4-5
```

它解决的是**交流耦合前端的直流漂移**问题：

| 现象 | 原因 | `SPO2_CAP` 怎么治 |
|---|---|---|
| 改光强（DAC 变了）后波形基线跳一个台阶，要很久才回来 | C8 是串联交流耦合，一级放大的直流电平台阶被"记住"在 C8 上 | Q1 导通 → 把 C8 后面的节点通过 270Ω 拉回 VREF，C8 重新建立到 VREF |
| 插拔手指后要等好几秒波形才正常 | 同上（光电流突变 = 直流突变） | 同上 |
| 基线缓慢漂移、慢慢顶到 ADC 量程边 | C8 后节点没有直流泄放通路 | 定期/按需复位一次 |

- 复位时间常数：`R23 × C8 = 270Ω × 100nF = 27µs`，5τ ≈ 135µs → 脉冲取 **200µs**；
- 电平：AO3401A 是 P 沟道、源极接 VREF(1.65V)，所以**栅极低 = 导通（复位）**，
  **栅极高 = 断开（正常测量）**。对应 `SPO2_CAP_ACTIVE_LOW = 1`；
- ⚠️ **这一脚必须接、必须驱动**：`SPO2_CAP` 网络上只有 2 个端点
  （H4-5 与 Q1 栅极），**板上没有上拉电阻**。栅极悬空时 Q1 通断不确定，
  会把信号节点随机拉向或拉离 VREF。所以本工程在 `InitSPO2FE()` 里第一件事就是把它置成确定的"断开态"。

固件侧的三个触发点（都已实现）：

| 触发 | 位置 | 说明 |
|---|---|---|
| 上电初始化 | `HW/SPO2FE/SPO2FE.c::ConfigCapPin()` | 置"断开态"，引脚有下拉，复位期间也不会浮空 |
| 光强变化 ≥ `SPO2_CAP_ADJ_DELTA` | `app/Main/Main.c::ProcWaveData()` → `SPO2FE_CapReqPulse()` | 用"请求"标志，在**下一个采样节拍、新光强已写进 DAC 之后、转换之前**执行脉冲 |
| 算法复位 | `app/Main/Main.c::SPO2AlgReset()` | 把"上次光强"清掉，强制下一拍补一次复位 |

> 为什么用"请求"而不是"就地打脉冲"：`SPO2FE_SetIntensityDa()` 只是改了待写入的 DA 值，
> 真正的 DAC 写入发生在下一次 `ReadPair()` 选通通道时。就地打脉冲 → 复位的是**旧光强**的基线，
> 紧接着光强一变又漂回去，等于白复位。

**现场判极性的标准操作**（命令 `0x8C`，不必重编译）：

```
① 手指夹好、等 adjDone=1
② 发 33 81 8C 80 80 80 80 80 80 C0    ← 动作0：置"断开态"，记 irAd / irAvg / 波形
③ 发 33 81 8C 81 80 80 80 80 80 C1    ← 动作1：置"复位态"，再记一次
④ 发 33 81 8C 82 80 80 80 80 80 C2    ← 动作2：打一个 200µs 复位脉冲（正常用法）
⑤ 发 33 81 8C 83 80 80 80 80 80 C3    ← 动作3：按原始电平强驱 = 低
   发 33 81 8C 83 81 80 80 80 80 C4    ← 动作3：按原始电平强驱 = 高
```

判据：**波形正常、`irAvg` 稳在中点（≈2048）的那一档才是"常态"**；
另一档会把交流节点短路到 VREF，波形会塌掉或顶到量程边。
若结论是"常态该为低电平"，把 `spo2_config.h` 的 `SPO2_CAP_ACTIVE_LOW` 改成 0 重编译即可。

> 校验和自检：上面 4 帧的算法规程已用"开始采样 `33 81 80 81 80 80 80 80 80 B5`"
> 与"查询状态 `33 81 84 80 80 80 80 80 80 B8`"两条已知帧反向验证通过。
> 需要其它参数组合时，用第 12.6 节末尾那段 Python 算，`module_id=0x33, second_id=0x8C`。

### 1.3 另一处需要实测确认的硬件特性：VSIG 是不是直流耦合

移植过来的“光强自动调节”状态机（`RoughAdj/FineAdj/JudgeReAdj`）是靠 **VSIG 的直流电平**
判断光强的（目标：IR 直流 ≈ 2000 码、RED ≈ 1700 码）。如果实测发现 VSIG 是**交流耦合**
（不管手指在不在，直流电平都稳在 1.65V 附近），这套闭环就没法工作，此时把
`spo2_config.h` 的 `SPO2_ADJ_EN` 改成 **0**：自动调光短路、DA 固定为起始值、
“粗调完成”恒为真，血氧与脉率照常计算。

---

## 2. 目录结构与心电工程的对应关系

完全沿用心电 F407 工程的分层与命名（`ARM / HW / App / Alg / module / libraries`）：

| 本工程（血氧） | 心电工程 | 关系 |
|---|---|---|
| `Alg/spo2_*.c/h` | `Alg/ecg_*.c/h` | **新增**（血氧算法本体） |
| `HW/SPO2FE/SPO2FE.c/h` | — | **新增**（血氧前端：DAC + 通道选通 + 采集成对点） |
| `HW/ADC/ADC.c/h` | `HW/ADC/ADC.c/h` + `U16Queue.c/h` | **重写**（去掉了 TIM 触发 + DMA + 环形队列） |
| `HW/RCC`、`HW/Timer`、`HW/UART1`(+Queue)、`HW/LED` | 同名 | **原样搬移**（未改一行） |
| `ARM/NVIC`、`ARM/SysTick` | 同名 | **原样搬移** |
| `app/DataType`、`app/PackUnpack` | 同名 | 搬移；`PackUnpack.h` **只加了 1 个二级ID** |
| `app/SendDataToHost` | 同名 | 搬移 + **改写为血氧的三种帧** |
| `app/ProcHostCmd` | 同名 | 搬移 + **命令表换成血氧的** |
| `app/Main/Main.c` | `App/Main/Main.c` | 搬移 + **数据源改成 SPO2FE_ReadPair** |
| `module/`、`libraries/` | 同名 | 保留（`stm32f4xx_it.c` 删掉重复的 `SysTick_Handler`） |

> `app/main.c`、`bsp/uart/bsp_uart.c/h`、`board/board.c/h`（立创模板自带的 ADC 例程）**仍留在磁盘上，
> 但没有被加进 Keil 的编译分组**，与心电工程的做法一致。原因：它们会与本工程重复定义
> `main()` / `USART1_IRQHandler()` / `fputc()`；想用回立创的 `board_init()` 延时库时，
> 只要在 IncludePath 里加回 `..\..\board`，并把 `HW/UART1/UART1.c` 从分组里去掉即可。

---

## 3. 新增文件清单（15 个源文件 + 11 个头文件）

### 3.1 `Alg/`：血氧算法本体（与硬件零耦合，不含任何外设访问）

| 文件 | 对应 MATLAB | 职责 |
|---|---|---|
| `spo2_config.h` | `SPO2.m` + `InitData.m` 的一堆 global 常量 | **系统唯一配置处**：采样率、量程、屏幕窗、所有阈值、光强中线、调试节流 |
| `spo2_coeffs.h/.c` | `FilterPulseWave.m` 的 `butter(5,20/(500/2))`、`CalcSPO2.m` 的 `RR_TABLE` | 滤波系数（0.5Hz 高通 2 阶 / 50Hz 陷波 / 20Hz 低通 5 阶）+ R 值查表 |
| `spo2_filter.h/.c` | `FilterPulseWave.m` | 逐样本滤波链：扣直流 → 0.5Hz 高通 → [50Hz 陷波] → 20Hz 低通。用 MATLAB `filter()` 同款**转置直接II型(DF2T)**递推，支持 2~5 阶 |
| `spo2_pulse.h/.c` | `CalcPulseRate.m` | 逐点脉率：自适应门限 + 不应期(**135 样本=0.27s**，等价 `MinPeakDistance`，按验收放宽到 200bpm) → `median(RR)` → `60*Fs/medRR`(=30000/medRR @500SPS) → gPRBuf 6 点中值 → 越界[30,200]置无效 |
| `spo2_calc.h/.c` | `RealAnalyzeData.m` + `CalcSPO2.m` | 2048 点屏缓冲、每屏 max/min/峰峰值/中点、**灌注指数 PI**、`R = redADRng*1000/irADRng`（可选直流归一）→ 6 点中值 → R 值现场标定 → 查 `RR_TABLE` 得 SpO2 |
| `spo2_light.h/.c` | `RoughAdj.m`/`FineAdj.m`/`JudgeReAdj.m`/`ResetAdj.m`/`CheckRoughAdj.m` | 光强自动调节状态机 + **手指脱落 / 探头脱落双判别**（含两级防抖） |
| `spo2_fft.h/.c` | `CalcAmpSpec.m` | 2048 点 radix-2 FFT + 归一化（功率）谱 + 带内主频，零 `math.h` 依赖 |
| `spo2_alg.h/.c` | `RealAnalyzeData.m` + `RealDraw` 定时器的调用顺序 | **顶层聚合**：把上面 5 个模块按 MATLAB 的调用节拍串起来，对外只有 `spo2_alg_init/step/reset/get_result` |

### 3.2 `HW/`：血氧专用硬件层

| 文件 | 职责 |
|---|---|
| `HW/ADC/ADC.h/.c` | ADC1 独立模式、单通道、**软件触发单次转换**（阻塞等待 EOC，带超时保护），含转换/超时/溢出统计；**`ADCConvertVsigAvg(n)` 过采样求平均（降随机噪声，保 AC 分辨率）** |
| `HW/SPO2FE/SPO2FE.h/.c` | 血氧前端：DAC 光强输出（DA 0~250 ↔ DAC 码 0~4095）、IR/RED 通道选通、极性归一、建立延时、**基线复位开关 `SPO2_CAP`（PC6，2026-10-08 版新增）**，以及 **`SPO2FE_ReadPair()` 这个采集时序**（每通道取 `SPO2_ADC_OS` 次的平均） |

### 3.3 从心电工程原样搬移的文件（未改内容）

`ARM/NVIC/NVIC.c/h`、`ARM/SysTick/SysTick.c/h`、`HW/RCC/RCC.c/h`、`HW/Timer/Timer.c/h`、
`HW/UART1/UART1.c/h`、`HW/UART1/Queue.c/h`、`HW/LED/LED.c/h`、`app/DataType/DataType.h`、
`app/PackUnpack/PackUnpack.c`、`app/SendDataToHost/*`（后两者内容有改动，见第 4 节）。

---

## 4. 修改的文件清单（改了什么、为什么）

| 文件 | 修改内容 | 原因 |
|---|---|---|
| `app/PackUnpack/PackUnpack.h` | `EnumSPO2SecondID` 里 **新增 `DAT_SPO2_RESULT = 0x04`、`DAT_SPO2_METRIC = 0x05`**；给 `DAT_SPO2_WAVE/DAT_SPO2_DATA/CMD_SPO2` 补了注释 | 血氧/脉率原来由上位机自己算，现在算法搬到 MCU 上，必须新增一帧把结果回传；0x05 承载验收要看的 **PI / 脱落状态 / R 值**。0x04、0x05 在原协议里空闲，MATLAB 上位机只认 0x02/0x03，会直接忽略它们，不干扰原有显示 |
| `app/SendDataToHost/SendDataToHost.h/.c` | 删掉心电的 `SendWaveToHost(u8*)`（5 点×8bit）；改为 `SendSPO2WaveToHost()` / `SendSPO2IntensityToHost()` / `SendSPO2ResultToHost()` / **`SendSPO2MetricToHost()`**；新增 `SPO2_HOST_WAVE_INVERT` | 血氧的波形帧是“红外 24bit + 红光 24bit”，与 `RealAnalyzeData.m` 的 `unpackRslt(3..8)` 逐字节对齐；光强帧对应 `gIrIntensity/gRedIntensity` 的解析式 |
| `app/ProcHostCmd/ProcHostCmd.h/.c` | 命令表整体换成血氧：`0x33/0x80` 采样开关（上位机原生）、`0x81` 手动调光、`0x82` 查结果、`0x83` 算法复位、`0x84` 查状态、`0x85` 设建立延时、`0x86` 设极性、`0x87` 切 ADC 通道、`0x88` 清零统计、**`0x89` R 值现场标定、`0x8A` 导出 1024 点原始 AD、`0x8B` 查 PI/脱落状态**；`MODULE_SYS` 增加 `0x91` 模块类型设置 | 对应心电工程里那批“骨架命令”；其中 `0x80`/`0x81` 就是 MATLAB 上位机的 `gCmdStartSamp/StopSamp` 与 `SendAdjCmd`；`0x89`~`0x8B` 是本次为“五参数验收”新增的 |
| `app/Main/Main.h` | 对外接口换成 `GetSPO2Result()` / `SPO2AlgReset()` / `SPO2SetManualLight()`，并新增 `SPO2DumpAdStart/IsBusy/GetDropCnt`、`SPO2SetRCal/GetRCal` | 供 `ProcHostCmd` 取结果、复位算法、接管光强、导出原始 AD 数据、做 R 值标定 |
| `app/Main/Main.c` | 数据源改为 `SPO2FE_ReadPair()`；每拍跑 `spo2_alg_step()` 后把算法算出的 DA 写给 DAC；新增 `SPO2_OUT_MODE` 串口输出模式开关；**新增 `DumpAdTick()` 分片非阻塞导出原始 AD**；每秒加发一帧 `0x05` 指标帧 | 心电是“从 ADC 环形队列取点”，血氧必须自己控制通道切换；光强从“上位机下发”变成**本地闭环**；导出与指标帧对应验收 |
| `libraries/CMSIS/.../startup_stm32f40xx.s` | `Stack_Size` 从 `0x400`(1KB) 加到 **`0x800`(2KB)** | 2ms 任务链上新增了 `DumpAdTick`（102 字节文本行缓冲）与更长的 `printf` 调用；满一屏时还有 2048 点 FFT 的栈局部变量。1KB 余量太薄，溢出即随机 HardFault |
| `HW/SPO2FE/SPO2FE.h/.c` | 新增 `SPO2_CAP`（PC6）驱动：`ConfigCapPin` / `SPO2FE_CapSet` / `SPO2FE_CapPulse` / `SPO2FE_CapReqPulse` / `SPO2FE_CapForceLevel` / `SPO2FE_CapPinLevel`，并在 `ReadPair()` 里按"请求"插入复位脉冲 | 血氧板 2026-10-08 版把 H4-5 从 NC 改成了 `SPO2_CAP`（接 Q1/AO3401A 栅极）。该网络**只有 2 个端点、板上无上拉**，栅极悬空会让 Q1 通断不确定，所以必须由 MCU 显式驱动 |
| `Alg/spo2_config.h` | 新增 `SPO2_CAP_EN` / `SPO2_CAP_ACTIVE_LOW` / `SPO2_CAP_PULSE_US` / `SPO2_CAP_ADJ_DELTA` | 复位开关的开关、极性、脉宽、自动触发门限 |
| `app/ProcHostCmd/*` | 新增命令 **`0x8C`** `CMD_SPO2_SET_CAP`（断开/复位/打脉冲/强驱电平）；`0x84` 状态打印增加 `SPO2_CAP` 一行 | 现场用命令判"常态电平"到底该高还是低，不必重编译 |
| `HW/ADC/ADC.h/.c` | 去掉 TIM 外部触发 / DMA / `U16Queue`，改为软件触发单次转换 + 统计；API 保留 `SetADCChannel/GetADCChannel/GetADCVoltageMv/ClearADCStatic` 风格 | 硬件触发表达不了“转两次之间要翻 GPIO”的时序（见 1.1） |
| `module/stm32f4xx_it.c` | **删除 `SysTick_Handler()` 定义** | 与 `ARM/SysTick/SysTick.c` 里的定义重复，会链接冲突（心电工程同款处理） |
| `project/MDK(V5)/Project.uvprojx` | 见第 5 节 | 加入新增文件、头文件路径、DAC 驱动 |

---

## 5. Keil 工程配置项改动（`project/MDK(V5)/Project.uvprojx`）

### 5.1 Options for Target → C/C++ → Include Paths

```
..\..\app\Main;..\..\app\DataType;..\..\app\PackUnpack;..\..\app\ProcHostCmd;
..\..\app\SendDataToHost;..\..\HW\RCC;..\..\HW\Timer;..\..\HW\UART1;
..\..\HW\ADC;..\..\HW\SPO2FE;..\..\HW\LED;..\..\ARM\NVIC;..\..\ARM\SysTick;
..\..\Alg;..\..\module;..\..\libraries\CMSIS\Include;
..\..\libraries\CMSIS\Device\ST\STM32F4xx\Include;
..\..\libraries\STM32F4xx_StdPeriph_Driver\inc
```

（原来是 `app;board;bsp;...;bsp\uart`，现在 `board`/`bsp` 已不再需要，`Alg`/`HW\SPO2FE` 是新增的）

### 5.2 Defined symbols（未变）

```
USE_STDPERIPH_DRIVER,STM32F40_41xxx
```

### 5.3 Target 页（未变，但必须确认）

| 项 | 值 | 说明 |
|---|---|---|
| Device | STM32F407VETx | |
| Use MicroLIB | **勾选** | 否则 `printf` 不能用（本项目没用 `%f`，MicroLIB 完全够用） |
| Output 名 | `SPO2_Algorithm` | 由原来的 `Project` 改名 |
| Create HEX File | 勾选 | |
| C99 Mode | 勾选 | `Alg/*.c` 里用了块内声明与 `//` 注释 |

### 5.4 编译分组（Groups）——整组替换

```
STARTUP : startup_stm32f40xx.s
APP     : Main.c  ProcHostCmd.c  PackUnpack.c  SendDataToHost.c
ALG     : spo2_alg.c  spo2_filter.c  spo2_coeffs.c  spo2_pulse.c
          spo2_calc.c  spo2_light.c  spo2_fft.c
HW      : RCC.c  Timer.c  UART1.c  Queue.c  ADC.c  SPO2FE.c  LED.c
ARM     : NVIC.c  SysTick.c
MODULE  : stm32f4xx_it.c
CMSIS   : system_stm32f4xx.c
Driver  : misc.c  stm32f4xx_rcc.c  stm32f4xx_gpio.c  stm32f4xx_usart.c
          stm32f4xx_tim.c  stm32f4xx_adc.c  stm32f4xx_dac.c  stm32f4xx_dma.c
DOC     : README.md
```

> 与原工程相比的要点：
> 1. **新增 `stm32f4xx_dac.c`**（血氧板的 SPO2_DAI 必须用 DAC 输出，原来没有）；
> 2. Driver 组里的源文件数量从“几乎全库”收敛到**只挂用到的 8 个**（换来了更短的链接时间和
>    更小的工程文件体积；注意 `stm32f4xx_dma.c` 本工程其实用不到，留着是为了后续加 DMA 时不
>    再动工程配置）；
> 3. 原来的 `main.c`(立创 ADC 例程)、`bsp_uart.c`、`board.c` **不再出现在任何分组里**。

### 5.5 编译器版本（本机 Keil 首次打开工程必读）

原模板工程（和心电工程一样）的 `.uvprojx` 里写死了一个编译器版本：

```xml
<pArmCC>5060960::V5.06 update 7 (build 960)::.\AC5</pArmCC>
<pCCUsed>5060960::V5.06 update 7 (build 960)::.\AC5</pCCUsed>
```

它要求 **AC5 = 5.06 update 7 (build 960)**，且安装目录为 `Keil\ARM\AC5`。而本机 Keil
（`D:\Keil_v5`，MDK Plus 5.24a，见 `D:\Keil_v5\TOOLS.INI`）注册的编译器只有：

```
ARMCCPATH0="ARMCC"    ("5.06 update 4 (build 528)")   ← 实际是 5.06 update 5 (build 528)
ARMCCPATH1=".\ARMCLANG" ("V6.7")
```

两边对不上，uVision 就会报：

```
*** Target 'Project' uses ARM-Compiler 'V5.06 update 7 (build 960)' which is not available.
*** Rebuild aborted.
```

**本工程已经修好了**：把那两行 `<pArmCC>` / `<pCCUsed>` 删掉，uVision 就会按
`TOOLS.INI` 里 `DEFAULT_ARMCC_VERSION_CM4` 的规则回落到本机已装的 **AC5 (build 528)**。
（如果是新建工程或改坏了，按下面任一种方式恢复。）

**修复方式（任选其一）**

1. **让 uVision 自己选（最省事，不需要改文件）**
   `Options for Target → Target 页 → ARM Compiler 下拉框`，选
   `5.06 update 4 (build 528)`（或 `Use default compiler version 5` / `V6.7`），
   点 OK → 重新编译。uVision 会把选中的版本写回 `.uvprojx`。

2. **改工程文件**
   用记事本打开 `project/MDK(V5)/Project.uvprojx`，删掉 `<pArmCC>` 和 `<pCCUsed>` 两行，
   保存后在 uVision 里**重新打开工程**（重要：uVision 内存里还留着旧设置，必须重新加载）。

3. **装 AC5 5.06u7**
   从 ARM/Keil 官网下载 "Arm Compiler 5.06u7"，装到 `D:\Keil_v5\ARM\AC5`，然后在
   `Manage Project Items → Folders/Extensions` 里注册。

> ⚠️ 注意 1：如果 uVision 已经打开了本工程，改完文件后必须 **Project → Close Project 再重新打开**
> （或直接重启 uVision），否则界面用的还是内存里的旧编译器设置，错误会照旧出现。
>
> ⚠️ 注意 2：本工程的代码与 MicroLIB 配置在 **AC5 (build 528) 与 AC6.7** 下都能编过，
> 但如果选了 AC6，`Options for Target → C/C++` 里的 `Optimization`、`C99 Mode`
> 等选项名会变成 AC6 的写法，建议就用 AC5 保持与心电工程一致。
>
> 已用 **uVision 批处理真实构建**验证（`UV4.exe -b -j0`，在隔离副本上跑）：
> `Program Size: Code=15444 RO-data=952 RW-data=120 ZI-data=30872`，
> **`0 Error(s), 0 Warning(s)`**，hex 正常生成。

---

## 6. 数据流与算法映射（MATLAB → C）

```
                     ┌──────────────────────── 2ms 时间片 (500Hz) ────────────────────────┐
 血氧板 H4            │                                                                    │
 SPO2_VSIG ──(PA5)──► ADC1 单次转换                                                     │
 SPO2_DAI  ◄─(PA4)── DAC1 光强 ◄── spo2_light (RoughAdj/FineAdj) 算出的 DA               │
 SPO2_IR_CS◄─(PC4)── GPIO ──┐                                                          │
 SPO2_RED_CS◄─(PC5)─ GPIO ──┴─► SPO2FE_ReadPair() ──► irAd / redAd                      │
                                                        │                              │
                                                        ▼                              │
                                      spo2_alg_step()  ──► ① spo2_filter  (20Hz低通)   │
                                                        ──► ② spo2_pulse   (脉率)      │
                                                        ──► ③ spo2_calc    (屏缓冲)    │
                                                        ──► ④ spo2_light   (光强/脱落) │
                                      满 2048 点(4.096s)时:                            │
                                                        ──► ⑤ spo2_calc_screen_run     │
                                                              = irMax/irMin/redMax/irMin │
                                                              = R 值 → 查表 → SpO2       │
                                                        ──► ⑥ spo2_fft (幅度谱主频)     │
                     └──────────────────────────────────────────────────────────────────┘
                                                        │
                              UART1(115200) ◄───────────┘
                                 ├─ 0x33/0x02 波形帧(红外24bit+红光24bit)  ← 与 MATLAB 上位机逐字节对齐
                                 ├─ 0x33/0x03 光强帧(红外DA+红光DA)        ← 上位机"光强"显示
                                 ├─ 0x33/0x04 结果帧(血氧+脉率+主频×100)   ← 本工程新增
                                 └─ 0x33/0x05 指标帧(PI + 脱落状态 + R值)  ← 本工程新增，验收用
```

MATLAB 模块 → C 模块的逐条对应，以及**刻意偏离**的地方，都写在 `Alg/*.h` 的头注释里，其中最需要
知道的三处：

1. **脉率检测的输入域换了**（`spo2_pulse.h`）：MATLAB 把 `findpeaks` 直接作用在原始 24bit AD
   （含 ~2000 码直流）上，`minpeakheight=100` 在那个量纲下几乎等于无门限；C 侧把峰值检测搬到
   了滤波后的交流域（与心电移植一致），量纲变了，所以改用**自适应门限**，而不应期
   （250 样本 = 0.5s = `MinPeakDistance`）与后端链路（`median(RR)` → `30000/medRR` →
   6 点中值 → 越界置 -100）**逐行照搬**。
2. **`SendAdjCmd` 变成了本地 DAC**（`spo2_light.h`）：MATLAB 每改一次光强就把 DA 值下发给
   “从机”；本工程 MCU 自己就是那个从机，所以只维护 DA 值、由上层写 DAC。上位机仍然可以按
   原协议发 `0x33/0x81` 接管光强（`manualMode`），直到算法复位才交还自动。
3. **幅度谱输出功率谱**（`spo2_fft.h`）：`CalcAmpSpec.m` 输出 `|X|/max|X|`，本工程输出
   `|X|²/max|X|²` —— 为了不依赖 `math.h` 开方；主频定位两者完全等价。

另外还有两处防御性差异：`FineAdj` 的 ±1 在 C 侧加了 `[MIN,MAX]` 钳位（MATLAB 不钳位，落到
`uint16` 会下溢）；`SPO2_ADJ_EN=0` 时自动调光整体短路（见 1.3）。

---

## 7. 编译验证结果（已实际执行）

```
工具 : D:\Keil_v5\ARM\ARMCC\bin  (AC5)
命令 : armcc --cpu Cortex-M4.fp -c --c99 -O1
              -DUSE_STDPERIPH_DRIVER -DSTM32F40_41xxx -I<18个路径>
       armasm --cpu Cortex-M4.fp -o startup.o startup_stm32f40xx.s
       armlink --cpu Cortex-M4.fp --strict
               --scatter spo2.sct            (IROM 0x08000000/512K,
                                              IRAM 0x20000000/128K, IRAM2 0x10000000/64K)
结果 : sources compiled : 30 C + 1 asm
       error-modules    : 0
       warning count    : 0
       RESULT: 0 error, 0 warning
       Total RO  Size (Code + RO Data)                23288 (  22.74kB)
       Total RW  Size (RW Data + ZI Data)             32664 (  31.90kB)
       Total ROM Size (Code + RO Data + RW Data)      23416 (  22.87kB)
产物 : SPO2_Algorithm.axf + SPO2_Algorithm.hex（可直接烧录）
```

> 注：本机 Keil 的 AC5 是 build 528（不是工程文件里写的 build 960），但**编译结果与
> MicroLIB 配置一致**；ARMCLANG(AC6) 与 AC5 都能编过，本工程用 AC5 以与心电工程保持一致。
> 命令行复刻时唯一的坑是：勾选 MicroLIB 时 uVision 会给汇编器加 `--pd "__MICROLIB SET"`，
> 命令行不带它的话启动文件会走 two-region 内存模型分支，链接报
> `L6218E __initial_sp / __use_two_region_memory`（与代码无关，是构建脚本的事）。

`Total RW 30.85kB` 里绝大部分是 `.bss`：FFT 工作区 20KB（`re[2048]+im[2048]+power[1025]`）
+ 屏缓冲 8KB（`irBuf[2048]+redBuf[2048]`）。F407VET6 有 128KB SRAM，余量充足。
**注意：`spo2_alg_t` 约 9KB，必须声明成 `static`/全局（`Main.c` 里就是 `static spo2_alg_t
s_spo2Alg;`）；若改成局部变量，默认 1KB 栈会立刻 HardFault。**

---

## 8. 上机测试步骤（每步都有明确判据）

串口助手设置：**115200 8N1，十六进制发送**。所有命令帧已按打包规则算好，直接发即可
（帧格式 = 模块ID + 数据头 + 二级ID + 6 字节数据 + 校验和，非首字节的最高位被数据头借走）。

### T1 上电自检（判据：PLL lock 必须为 1）

上电后串口应打印（`SPO2_OUT_MODE=0` 的默认配置下，这段是上电唯一的文本）：

```
==============================================
 SPO2 algorithm on STM32F407VET6 (LCKFB Firebase-Tx)
 SYSCLK = 168000000 Hz, PLL lock = 1
 APB1 TIM clk = 84000000 Hz, APB2(PCLK2) = 84000000 Hz
 SPO2 sample rate = 500 SPS/ch, screen = 2048 points (4096 ms)
 H4: VSIG=PA5(CH5) DAI=PA4(DAC1) IR_CS=PC4 RED_CS=PC5 CAP=PC6
 SPO2_CAP: en=0 active_low=1 pulse=200us delta=2
 FE settle = 100us, invert = 0 (light stronger -> code smaller)
 IR central = 2000, RED central = 1700, off threshold = 150
 [accept] PR range = 30-200 bpm, SpO2 range = 90-99%, PI expect = 3.00%
 [accept] ADC oversampling = 4 (random noise /sqrt(4))
 [accept] off detect: probe(no light) code >= 3900, finger(too bright) code < 150, en = 1
 [accept] R cal: gain = 1000/1000, offset = 0
 Wave output: PCT protocol frames (0x33/0x02 wave, 0x33/0x03 int, 0x33/0x04 result)
 UART1 baudrate = 115200
==============================================
```

判据：`PLL lock = 1`。若为 0，说明 8MHz 晶振没起振、芯片在跑 16MHz 内部 RC，采样率与波特率
都会偏，先解决硬件再看后面的数据。

> 想看波形但手边没有 MATLAB 上位机？把 `app/Main/Main.c` 的 `SPO2_OUT_MODE` 改成 `1`，
> 重新编译，即可用 **串口波形小工具 / VOFA+** 直接看双波形，右侧参数面板显示
> `PI / PR / SpO2 / DAC / OFF`（通道选择见 `SPO2_WAVE_CH1/CH2`）。

### T2 前端极性确认（本板已定为 `SPO2_FE_INVERT = 0`，此步只做复核）

探头（DB9 指夹）**不要夹手指**，发查询状态命令：

```
查询运行状态 : 33 81 84 80 80 80 80 80 80 B8
```

看回显里的 `irAd=`（正确极性下，不夹手指 = LED 直射光电管 = 最亮 = 码值最小）：
- **≈ 95**（几十~一百多）→ 极性正确，继续；
- ≈ 3300 左右（偏暗侧）→ 极性反了：发 `33 81 86 81 80 80 80 80 80 BB`（取反）在线翻一下，
  或把 `SPO2_FE_INVERT` 改成 `1u` 重新编译（详见 1.2 的实测依据）。

### T3 光强闭环自检（判据：`adjDone` 变 1、`irDAC` 从 10 附近往上爬）

夹上手指（或用白纸/手指遮住探头），每秒发一次 `33 81 84 ...B8`，观察：

```
 finger=1 adjDone=0 irDA= 20 redDA= 20 adjTick= 12     ← 正在粗调，DA 每 0.3s 加 2
 finger=1 adjDone=1 irDA=178 redDA=156 adjTick= 60     ← 粗调完成，进入细调
 irAd=1980 redAd=1690 ...（IR中线=2000 RED中线=1700）   ← 说明直流电平落入标定区间
```

判据：`adjDone` 能从 0 变 1；`irAvg/redAvg` 收敛到中线附近（±300 内会被细调继续修）。
若 `irDA` 一路走到 250 还在 `adjDone=0`，说明 LED 不够亮或前端增益偏低 → 检查 H4 接线/5V 供电，
或放宽 `SPO2_IR_CENTRAL_LINE`。
若一直 `finger=0`，检查极性（T2）。

> 粗调节拍沿用了 MATLAB 的 `RealDraw` 定时器（10Hz）× `MAX_CNT_ADJ_*_DELAY(3)` = **每 0.3s 调一次 DA**，
> 所以从 20 爬到 250 最坏情况要 34.5s。嫌慢就把 `spo2_config.h` 里
> `SPO2_MAX_CNT_ADJ_IR_DELAY` / `MAX_CNT_ADJ_RED_DELAY` 改成 1（→ 每 0.1s 调一次，最坏 11.5s）。
> 本板是 MCU 直接驱动 DAC，不需要等模块响应，改小是安全的。

### T4 波形与结果（判据：R 值在 500~900、SpO2 落在 90~99）

- 直接看上位机 MATLAB 的 4 个坐标轴：坐标1/2 = 红外/红光原始，坐标3 = 20Hz 低通后的脉搏波，
  坐标4 = 0~10Hz 归一化谱（应与脉搏基频一致）；
- 或者发 `33 81 82 80 80 80 80 80 80 B6`（查血氧结果）看回显：

```
SPO2 result: spo2=97 pr=72 finger=1 adj=1 irDA=178 redDA=156 rVal=643 fund=120
```

判据：
- `rVal` 落在 500~900；`spo2` 落在 90~99（无效时是 -100，对应上位机显示 `---`）；
- `pr` 与 `fund`（主频×100 Hz）互相印证：`pr ≈ fund × 60`；
- 屏缓冲每 4.096s 才更新一次血氧（与 MATLAB 的“收满一屏才算”一致），不要按秒期待它跳变。

### T5 算法与统计命令（判据：都回 ACK 且状态行字段同步变化）

| 命令 | 帧（十六进制） | 作用 |
|---|---|---|
| 开始采样 | `33 81 80 81 80 80 80 80 80 B5` | 波形上传开 + 算法复位重标定（= 上位机“开始采样”） |
| 停止采样 | `33 81 80 80 80 80 80 80 80 B4` | 停止上传波形（= 上位机“停止采样”） |
| 查询血氧结果 | `33 81 82 80 80 80 80 80 80 B6` | 回一帧 `0x33/0x04` + 打印一行 |
| 算法复位 | `33 81 83 80 80 80 80 80 80 B7` | 清屏 + 复位 R 值/脉率缓冲 + 光强交还自动 |
| 查询运行状态 | `33 81 84 80 80 80 80 80 80 B8` | 打印多行详细状态 |
| 建立延时=200us | `33 83 85 C8 80 80 80 80 80 83` | 前端通道建立延时（μs，小端） |
| 建立延时=100us | `33 81 85 E4 80 80 80 80 80 9D` | 同上 |
| 极性=取反 | `33 81 86 81 80 80 80 80 80 BB` | 前端极性归一在线试 |
| 极性=不取反 | `33 81 86 80 80 80 80 80 80 BA` | 同上 |
| ADC 通道=5(PA5) | `33 81 87 85 80 80 80 80 80 C0` | 切回 SPO2_VSIG |
| ADC 通道=17(VREFINT) | `33 81 87 91 80 80 80 80 80 CC` | **不用接线的自检**：内部 1.2V 基准 |
| 手动调光 IR=100,RED=50 | `33 81 81 E4 80 B2 80 80 80 CB` | 上位机接管光强（= `SendAdjCmd`） |
| 波形上传开/关 | `71 81 80 81 80 80 80 80 80 F3` / `...80... F2` | 同心电工程的调试命令 |
| 清零统计量 | `33 81 88 80 80 80 80 80 80 BC` | 清 ADC 统计 |
| **R 值标定：直通** | `33 83 89 E8 83 80 80 80 80 AA` | gain=1000, offset=0（回到不标定，与 MATLAB 数值行为一致） |
| **R 值标定：增益 1.05** | `33 81 89 9A 84 80 80 80 80 DB` | R 值整体 ×1.05（把偏低的血氧读数往上拉） |
| **R 值标定：偏置 −10** | `33 9B 89 E8 83 F6 FF 80 80 B7` | R 值整体 −10（精细调整） |
| **导出 1024 点原始 AD** | `33 81 8A 80 80 80 80 80 80 BE` | 验收前置条件“≥1000 点真实 AD”，约 1.28s 发完纯文本 |
| **查询 PI / 脱落 / R 值** | `33 81 8B 80 80 80 80 80 80 BF` | 回一帧 `0x33/0x05` + 打印两行（PI、脱落状态、R 值） |
| **SPO2_CAP：断开(正常)** | `33 81 8C 80 80 80 80 80 80 C0` | 前端基线复位开关置"正常测量"（见 1.5） |
| **SPO2_CAP：复位(导通)** | `33 81 8C 81 80 80 80 80 80 C1` | 把 C8 后的节点持续拉到 VREF（试极性用） |
| **SPO2_CAP：打一个复位脉冲** | `33 81 8C 82 80 80 80 80 80 C2` | 200µs 脉冲，改光强后手动补复位 |
| **SPO2_CAP：强驱电平** | `33 81 8C 83 80 80 80 80 80 C3`（低）/ `...83 81...C4`（高） | 固定住引脚电平，一边看波形一边判极性 |

应答帧格式：`01 84 <二级ID> <被应答模块ID> <被应答命令> 00 80 80 80 <校验>`，`00` 表示成功。

> 表中帧的校验和由 `app/PackUnpack/PackUnpack.c` 的 `PackWithCheckSum()` 规则算得，
> 已用“开始采样/查询状态”两条已知帧反向自检通过。
> **注意 `0x89`/`0x8A`/`0x8B` 帧的数据字节在传输时最高位被搬到数据头里了**，
> 所以看到的是 `E8 83`（= gain 低字节 0xE8 + 高字节 0x03|0x80），
> 解包后才是 `0x03E8 = 1000`，这是协议本身的设计，不是笔误。

**VREFINT 自检判据**：切到通道 17 后 `irAd` 应在 1489 附近（1.2V/3.3V×4095）。
注意此时 `irAd` 是**归一后**的值（`4095 - raw`）：raw≈1489 → `irAd ≈ 2606`。
测完记得切回通道 5。

### T6 动态响应（判据：无超时、无溢出、无丢帧）

连续观察 `33 81 84 ...B8` 的 `conv=` 与 `timeout=`/`ovr=`：
- `conv` 每秒增加 **1000**（IR/RED 各 500 次转换）；
- `timeout` 与 `ovr` **恒为 0**。

2026-10-09 起，同一命令还会打印一行 `tick:`（见第 14 节），判据更硬：
- `beat` 每秒增加 **500**（硬件时基，含被丢弃的拍）；
- `miss` **恒为 0** —— 这才是"无丢帧"的直接证据（`conv/timeout/ovr` 发现不了整拍被拖长）；
- `tickMax` 应明显小于 **2000 us**（一拍采集+算法+上报的预算）；
- `heavyMax` 是一次屏级重算（屏统计 + 2048 点 FFT）的真实耗时，它现在落在采样拍之外；
- `lostScreen` **恒为 0**（非 0 说明主循环长期取不走重算活，要减慢串口输出而不是改算法）。

### T7 LED

板载 LED(PB2) 每 500ms 翻转一次，证明主循环没被卡死。

---

## 9. 调参入口速查

| 想改什么 | 改哪里 |
|---|---|
| 采样率、屏幕窗长度 | `Alg/spo2_config.h`：`SPO2_FS`、`SPO2_SCREEN_LEN`（改屏幕长度后 FFT 点数与屏缓冲要同步） |
| 低通截止/阶数/系数 | `Alg/spo2_coeffs.h/.c`（用 `scipy.signal.butter` 或 MATLAB `butter` 重新生成，见第 10 节脚本） |
| 工频陷波 | `spo2_config.h`：`SPO2_FILT_NOTCH_EN`（默认 0，与 MATLAB 一致；现场干扰大再开） |
| 脉率门限/不应期 | `spo2_config.h`：`SPO2_PR_THR_ALPHA`、`SPO2_PR_ABS_MIN`、`SPO2_PR_MIN_DIST`(135=0.27s→222bpm)、`SPO2_PR_MIN/MAX`(30/200，验收区间) |
| 随机噪声 | `spo2_config.h`：`SPO2_ADC_OS`（每通道过采样次数，默认 4；降噪 = 1/√N） |
| 基线漂移 | `spo2_config.h`：`SPO2_FILT_HP_EN` / `SPO2_FILT_FHP`（0.5Hz 高通，已默认开启） |
| 血氧查表 | `Alg/spo2_coeffs.c` 的 `SPO2_RR_TABLE`（原样照抄 `CalcSPO2.m`，按实际探头标定后应替换） |
| **血氧标定（±2%）** | **首选运行时命令 `0x89`**（见第 12.6 节），不影响已编译固件；编译期默认值在 `spo2_config.h` 的 `SPO2_R_CAL_GAIN_DEF/OFFSET_DEF` |
| R 值是否做直流归一 | `spo2_config.h`：`SPO2_R_DC_NORM`（**默认 0 保持与 MATLAB/`RR_TABLE` 一致**，改成 1 必须重标表） |
| 灌注指数 PI | `spo2_config.h`：`SPO2_PI_VALID_MIN_X100`（PI 有效性门限，×100 口径，20 = 0.20%）、`SPO2_PI_MAX_X100` |
| 光强中线/手指脱落门限 | `spo2_config.h`：`SPO2_IR_CENTRAL_LINE`、`SPO2_RED_CENTRAL_LINE`、`SPO2_*_OFF_MAX_VAL`、`SPO2_RE_ADJ_*` |
| **探头脱落判别** | `spo2_config.h`：`SPO2_PROBE_OFF_EN`(总开关)、`SPO2_PROBE_OFF_CODE_MIN`(无光门限)、`SPO2_PROBE_OFF_CNT`(防抖) |
| 原始 AD 导出 | `spo2_config.h`：`SPO2_DUMP_POINTS`(默认 1024)、`SPO2_DUMP_PER_LINE`(8)、`SPO2_DUMP_TICK_DIV`(5)；**必须满足 PER_LINE > TICK_DIV**（有编译期 `#error` 兜底） |
| 调光速度 | `spo2_config.h`：`SPO2_ADJ_PERIOD_SMP`、`SPO2_MAX_CNT_ADJ_*_DELAY`、`SPO2_ROUGH_ADJ_*_STEP` |
| 前端引脚 | `HW/ADC/ADC.h`（VSIG 通道）、`HW/SPO2FE/SPO2FE.h`（DAI 与两个 CS） |
| 通道建立延时 | `spo2_config.h`：`SPO2_FE_SETTLE_US`（也可用命令在线改） |
| 串口输出模式 | `app/Main/Main.c`：`SPO2_OUT_MODE`（0=上位机 PCT 协议帧，1=串口波形小工具/VOFA+ 双波形+参数面板） |
| **双波形画哪两路** | `app/Main/Main.c`：`SPO2_WAVE_CH1` / `SPO2_WAVE_CH2`（0=IR原始 1=IR脉搏波×10 2=RED原始 3=IR直流 4=脉率 5=血氧）|
| 波形帧降采样 | `app/Main/Main.c`：`SPO2_WAVE_SEND_DIV`（仅协议模式）、`SPO2_VOFA_DIV`（仅文本模式） |
| **前端极性** | `spo2_config.h`：`SPO2_FE_INVERT`（**本板实测 = 0**，详见 1.2） |
| **脱落门限** | `spo2_config.h`：`SPO2_IR/RED_OFF_MAX_VAL`（手指脱落，实测 150）、`SPO2_PROBE_OFF_CODE_MIN`（探头脱落，3900）、`SPO2_PROBE_OFF_EN` |

### 串口带宽核算

`SPO2_OUT_MODE=0`（500 帧/s × 10 字节 = 5000 B/s）+ 光强帧 2 帧/s + 结果帧 1 帧/s ≈ **5.03 KB/s**，
占 115200(≈11.5KB/s) 的 **44%**，安全。若以后要把采样率提到 1000SPS，请同时把波特率改成
921600 或把 `SPO2_WAVE_SEND_DIV` 改成 2。

---

## 10. 系数是怎么算出来的（可复现）

`Alg/spo2_coeffs.c` 的系数由等价于 MATLAB `butter` 的 `scipy.signal.butter` 生成，
并用“与 `scipy.signal.lfilter`（= MATLAB `filter`）逐样本对拍”验证过 DF2T 递推的正确性：

```
HP  (butter(2, 0.5/250,'high'))    max|DF2T - lfilter| = 4.7e-10
LP  (butter(5, 20/250 ,'low'))     max|DF2T - lfilter| = 2.8e-09
NOTCH (50Hz, beta=0.96)            max|DF2T - lfilter| = 5.5e-12
级联后的直流增益 = -5.6e-14（直流被彻底扣掉）
频响：HP 0.5Hz -3.0dB ｜ LP 20Hz -3.0dB / 40Hz -30.8dB / 50Hz -41.0dB
```

所以 **C 侧输出与 MATLAB `filter(b,a,x)` 逐样本等价**，差异只有 single 舍入（1e-5 量级），
可以直接用“MATLAB 激励 vs C 输出对拍”的方式做金标准验收。

FFT 侧的算法原型也用等价的 Python 实现与 `numpy.fft` 对拍过：相对最大偏差 **2.4e-18**。

---

## 11. 已知限制与后续可做的事

1. **`RR_TABLE` 未按实物探头标定**：它逐字来自 `CalcSPO2.m` 的常量表。同款血氧板 + 同款指夹
   探头的 R-SpO2 曲线会与 LEYUTEK 模块的略有差异，正式使用前应用标准血氧模拟器/血氧仪
   做两点以上标定，再替换 `SPO2_RR_TABLE`。
2. **`irADRng` 默认不做直流归一**：MATLAB 的 `R = redADRng*1000/irADRng` 直接用了峰峰值之比，
   严格定义应是 `R = (AC_red/DC_red)/(AC_ir/DC_ir)`。两者相差一个固定的 `redAvg/irAvg` 因子
   （由两条调光中线决定）。本工程为保持与原型数值一致**默认走原式**，但已提供开关
   `SPO2_R_DC_NORM`（改成 1 即切到标准定义式）；**切过去之后必须用实测数据重新整定
   `SPO2_RR_TABLE`**，否则查表会系统性偏移。
3. **未做 SpO2 时间平均**：目前只有 R 值的 6 点中值（与 MATLAB 一致）。监护场景通常还要
   对 `spo2` 再做一次滑动平均/变化率限制，属于产品化增强。
4. **探头脱落判别的门限是占位值**：`SPO2_PROBE_OFF_CODE_MIN = 3900` 是按“归一后码值趴在
   满量程一侧 = 完全无光”这一物理约定给的，必须用实测值回填（三种状态各看一次 `irAd`）。
   如果实测发现探头是**反射式**（无手指时反而收不到光），两种脱落会落在同一侧，
   此时把 `SPO2_PROBE_OFF_EN` 置 0，退化为“只报一种脱落”。
5. **PI 的交流幅度取的是峰峰值/2**：与 `R` 值的峰峰值口径一致，便于交叉核对；
   若要与商用监护仪的 PI 读数严格对齐，需要按 RMS 口径重算（`CalcPiX100` 里改一行）。
6. **`CalcAmpSpec` 输出的是功率谱**（见 6.3），只影响上位机那一路谱线的纵轴标尺。
7. **`HW/ADC` 没有 DMA**：一拍的两次转换是阻塞的（约 384us，含 4 次过采样）。若以后要加呼吸/体温
   等更多通道，应改为“TIM 触发 + DMA 双缓冲 + 主循环批处理”，`stm32f4xx_dma.c` 已经挂在工程里。
8. `Alg/spo2_light.c` 里的 `ResetAdj()` 会同时解除上位机的手动接管（`manualMode`），
   这是为了不让人卡在手动模式；如果希望“上位机接管后一直有效”，把那一行删掉即可。
9. **`SPO2_ADC_OS` 与 `SPO2_FE_SETTLE_US` 是此消彼长的**：两者共同吃 2ms 节拍预算
   （当前 ≈19%）。若现场需要更长的建立时间，先把过采样降到 2。

---

## 12. 五参数验收标准（血氧部分）对照与操作流程

依据：`五参数验收标准.pdf` → 「血氧验收标准」一节。

**通用前置条件（验收原文）**：测试环境 15~30℃、900~1000kPa；实验报告需记录
**各参数至少 1000 个真实 AD 数据** → 本工程用命令 `0x33/0x8A` 一次导出 1024 对原始码值（见 12.8）。

### 12.1 逐条对照表

| # | 验收条文 | 本工程实现（文件/函数） | 配置项 / 命令 | 判据 |
|---|---|---|---|---|
| 1 | 调光要求：**具有自动调光** | `spo2_light.c`：`spo2_light_rough_step()` + `spo2_light_judge_step()`（粗调/细调/重调/复位四件套） | `SPO2_ADJ_EN=1`；`SPO2_IR/RED_CENTRAL_LINE`；`SPO2_ADJ_PERIOD_SMP` | `adjDone` 由 0→1，`irDA/redDA` 稳定不再爬升 |
| 2 | 噪声要求：**滤除随机噪声、基线漂移** | 基线漂移：`spo2_filter.c` 的 0.5Hz 二阶高通；随机噪声：`ADC.c::ADCConvertVsigAvg()` 4 次过采样平均 + 20Hz 五阶低通 | `SPO2_FILT_HP_EN=1`；`SPO2_ADC_OS=4`；`SPO2_FILT_ORDER=5`/`SPO2_FILT_FPASS=20` | 静置时滤波后波形无明显基线漂移；对同一稳定信号连测，`rVal` 的逐屏标准差小 |
| 3 | 脉率范围 **30~200BPM，误差 ±5%** | `spo2_pulse.c`（自适应门限 + 不应期 + `median(RR)` + 6 点中值） | `SPO2_PR_MIN=30`、`SPO2_PR_MAX=200`、`SPO2_PR_MIN_DIST=135`（0.27s→222bpm 上限） | 模拟器设 30/60/120/180/200，显示值误差 ≤5%；`pr ≈ fund×60` 互相印证 |
| 4 | 灌注值数：**3%**（模拟器固定输出） | `spo2_calc.c::CalcPiX100()` | `SPO2_PI_SCALE=100`；`SPO2_PI_VALID_MIN_X100=20` | `0x8B` 回显 `PI=3.00%` 附近（原始整数 300） |
| 5 | 血氧范围 **90~99%** | `spo2_coeffs.c` 的 `SPO2_RR_TABLE`（index 1..11 → 100..90，100 钳到 99） | `SPO2_R_TABLE_LEN=11`、`SPO2_SAT_MAX=99` | 查表结果天然落在 90~99，不会越界 |
| 6 | 血氧误差 **±2%** | `spo2_calc.c::UpdateSPO2()` 的 R 值现场标定 + `SPO2_ADC_OS` 过采样保分辨率 | 运行时命令 `0x89`；`SPO2_R_CAL_GAIN_DEF/OFFSET_DEF` | 模拟器设 90/95/99，显示值落在 ±2 以内（见 12.6 标定流程） |
| 7 | 导联脱落：**自动判别手指脱落与探头脱落** | `spo2_light.c::DetectProbeOff()`（探头）+ `spo2_light_rough_step()`（手指）+ `spo2_alg.c` 的 PI 二级判据 | `SPO2_PROBE_OFF_EN=1`；`SPO2_PROBE_OFF_CODE_MIN=3900`；`SPO2_IR/RED_OFF_MAX_VAL=80` | `0x8B` 的 `offState`：0 正常 / 1 手指脱落 / 2 探头脱落，三者可复现切换 |

### 12.2 第 1 条 · 自动调光

移植自 MATLAB 的 `RoughAdj/FineAdj/JudgeReAdj/ResetAdj/CheckRoughAdj`，控制律未改：

```
每 0.1s(SPO2_ADJ_PERIOD_SMP=50 样本) 调一次:
    若 irAd ≥ IR_CENTRAL_LINE(2000) → DA += 2（每 0.3s 因 MAX_CNT_ADJ_IR_DELAY=3 才动一次）
    连续 3 次"到位"(irAd < 中线) → 粗调完成
每满一屏(4.096s)、且粗调完成:
    直流跑出 [BOT_LINE, TOP_LINE] 连续 3 屏 → 重新粗调
    仍在窗内但有偏差 → 细调 ±1
```

验收判据：夹上手指后 `adjDone` 从 0 变 1，且 `irAvg/redAvg` 收敛到 2000/1700 附近；
之后长时间不动，`irDA/redDA` 不再持续爬升。

> **调光与 `SPO2_CAP` 的配合（新版硬件必读）**：前端 C8 是交流耦合，每次改光强都会在
> C8 上留一个直流台阶。所以本工程在"光强变化 ≥ `SPO2_CAP_ADJ_DELTA`"时，会在下一个采样
> 节拍里自动补一个 200µs 的基线复位脉冲（见 1.5）。**没有这一步，粗调期间 `irAvg` 会
> 一直漂，`JudgeReAdj` 的越界判断会误触发、反复重新粗调。**
> 如果实测发现前端其实是**直流耦合**（`SPO2_CAP` 怎么变 `irAvg` 都不动），
> 说明这块板子的 `Q1`/`C8` 与你手上的版本不同，把 `SPO2_CAP_EN` 置 0 即可。

### 12.3 第 2 条 · 噪声（随机噪声 + 基线漂移）

| 噪声类型 | 手段 | 位置 |
|---|---|---|
| 基线漂移（呼吸、体动、慢漂） | 0.5Hz 二阶 Butterworth 高通，0.5Hz 处 -3dB | `spo2_filter.c`，系数在 `spo2_coeffs.c` |
| 随机噪声（量化噪声 + 白噪声） | **ADC 每通道连转 4 次取平均**，噪声降到 1/2 | `ADC.c::ADCConvertVsigAvg()`，由 `SPO2FE_ReadPair()` 调用 |
| 高频噪声（工频、肌电） | 20Hz 五阶 Butterworth 低通（20Hz -3dB / 40Hz -30.8dB / 50Hz -41.0dB） | `spo2_coeffs.c` 的 `SPO2_LP_*` |
| 工频干扰（可选） | 50Hz 陷波（默认关，与 MATLAB 一致） | `SPO2_FILT_NOTCH_EN` 改成 1 |

> 为什么必须做过采样：验收要求模拟器灌注指数固定 3%，交流峰峰值只有直流的 3%
> （直流 2000 码时约 120 码），1LSB 量化就有 ~0.8% 的相对误差，不做平均会直接吃掉
> 第 6 条 ±2% 的预算。这是“第 2 条”和“第 6 条”互相咬合的地方。

### 12.4 第 3 条 · 脉率 30~200BPM ±5%

**移植时最容易踩的坑**：MATLAB 原不应期 250 样本(0.5s) 隐含“最大只能测 120bpm”，
验收要 200bpm（RR = 150 样本），必须同步把 `SPO2_PR_MIN_DIST` 改到 135（0.27s）。
只改 `SPO2_PR_MAX` 不改不应期的话，读数会周期性减半。

| 项目 | 值 | 依据 |
|---|---|---|
| 有效范围 | 30 ~ 200 bpm | `SPO2_PR_MIN` / `SPO2_PR_MAX`（验收区间） |
| 不应期 | 135 样本 = 0.27s | → 可测上限 222bpm，对 200bpm 留 11% 裕量；同时 > 0.2s 能压住重搏波 |
| RR 合法窗 | 150 ~ 1000 样本 | `SPO2_PR_MIN_DIST` ~ `60*Fs/SPO2_PR_MIN` |
| 门限 | 自适应 `winMin + 0.6×(winMax-winMin)`，4.096s 滑动窗 | MATLAB 的 `minpeakheight=100` 在交流域无意义，已换成自适应 |
| 后端 | `median(RR)` → `60*Fs/medRR` → 6 点中值 → 越界置 -100 | 与 `CalcPulseRate.m` 逐行一致 |

验收操作：模拟器依次设 30 / 60 / 120 / 180 / 200 bpm，每次等 ≥ 3 屏（约 12s）稳定，
用 `0x8B` 读 `pr`。判据：|显示值 − 设定值| / 设定值 ≤ 5%，
且 `pr ≈ fund(主频)×60`。

### 12.5 第 4 条 · 灌注指数 3%

```c
PI(%) = (交流幅度 / 直流) × 100 = (irRng/2) / irAvg × 100
      → 整数输出 PI×100 = irRng × 5000 / irAvg      // 3.00% → 300
```

- 位置：`spo2_calc.c::CalcPiX100()`，每满一屏（4.096s）算一次；
- 输出：`0x33/0x05` 指标帧的 `arrData[0..1]`，以及 `0x8B` 命令的打印行；
- 顺带用途：`PI < SPO2_PI_VALID_MIN_X100(0.20%)` 连续 2 屏 → 判“光路通但无脉动”，
  血氧/脉率一并置 -100（这一层是第 7 条的补充判据）。

验收操作：模拟器接入后读 `0x8B`，判据 `PI` 打印为 **3.00%** 附近（原始值 300±30）。

> 若显示值系统性是设定值的一半（1.50%），说明模拟器的 PI 定义用的是“半幅”而非“峰峰值/2”，
> 把 `CalcPiX100()` 里的 `50` 改成 `100` 即可（已留注释）。

### 12.6 第 5、6 条 · 血氧 90~99%、误差 ±2%

**先说清楚精度由什么决定**（这是最容易背锅的地方）：

| 环节 | 影响 | 本工程对策 |
|---|---|---|
| AC 幅度分辨率 | PI=3% 时 AC 峰峰值 ≈120 码，1LSB ≈0.8% | `SPO2_ADC_OS=4` 过采样，白噪声降到 1/2 |
| 峰峰值对噪声敏感 | 单点毛刺会同时抬高峰和谷 | 屏统计用原始码值（与 MATLAB 一致），但喂给统计的是过采样平均后的值 |
| R 的直流归一化 | 原式 `R=redRng/irRng` 与标准式差一个 `redAvg/irAvg` 常数因子 | 默认保持原式（与 `RR_TABLE` 配套）；可切 `SPO2_R_DC_NORM`（切了必须重标表） |
| R→SpO2 曲线 | MATLAB 的表是 LEYUTEK 模块整定的，与你的探头有差异 | **`0x89` 运行时 R 值标定**，不动表、可复现 |

**查表区间**（`spo2_coeffs.c`，实际生效的门限）：

```
R_used < 620          → 99 %      620 ≤ R < 650 → 98 %     650 ≤ R < 670 → 97 %
670 ≤ R < 700         → 96 %      700 ≤ R < 730 → 95 %     730 ≤ R < 760 → 94 %
760 ≤ R < 780         → 93 %      780 ≤ R < 810 → 92 %     810 ≤ R < 840 → 91 %
R_used ≥ 840          → 90 %
```

**两点标定流程（把模拟器读数压进 ±2%）**

标定作用在 R 值上：`R_used = R_raw × gain/1000 + offset`。

1. 模拟器设 **PI=3%、SpO2=99%**，夹好探头，等粗调完成 + 至少 3 屏稳定（约 15s）；
   发 `33 81 8B 80 80 80 80 80 80 BF`，记下打印的 `raw=` → 记为 **R99**。
2. 模拟器改 **SpO2=90%**，同样等稳定，记下 `raw=` → 记为 **R90**。
3. 按公式解标定参数（把两个实测点映射到查表区间中段）：
   ```
   R_match_low  = 500   (99% 档的中段，落在 <620 区间内，稳)
   R_match_high = 860   (90% 档的中段，落在 ≥840 区间内，稳)

   gain/1000 = (R_match_high - R_match_low) / (R90 - R99)
   offset    = R_match_low - R99 × gain/1000
   ```
   例：实测 R99 = 480、R90 = 700 →
   `gain/1000 = 360/220 = 1.636 → gain = 1636`，`offset = 500 − 480×1.636 = −285`。
4. 发 `0x89` 把 gain/offset 写进去（帧的算法见本节末尾），**立即生效、且跨复位保留**；
   再发 `0x8B` 读回 `rVal`，确认它已经落在期望区间、`spo2` 显示 90/99。
5. 用 95% 档复核（应显示 95±2）。三点都过关即完成标定，把 gain/offset 记进实验报告。

**`0x89` 帧怎么算**：数据 6 字节 = `gain 低, gain 高, offset 低, offset 高, 0, 0`（小端，
都按 uint16 二进制补码写），再套用 `PackWithCheckSum` 规则。已算好的常用帧：

| 标定 | 帧（十六进制） |
|---|---|
| 直通 gain=1000, offset=0 | `33 83 89 E8 83 80 80 80 80 AA` |
| gain=1050, offset=0 | `33 81 89 9A 84 80 80 80 80 DB` |
| gain=950, offset=0 | `33 83 89 B6 83 80 80 80 80 F8` |
| gain=1000, offset=−10 | `33 9B 89 E8 83 F6 FF 80 80 B7` |

> ⚠️ 帧里 `E8 83` 看起来像“gain=0x83E8”，其实不是：协议把每个数据字节的最高位搬进了
> “数据头”（第 2 字节），解包后才是 `0x03E8 = 1000`。这是原协议设计，不是笔误。
> 需要其它档位时用下面这段 Python 算（已验证与固件的 `PackWithCheckSum` 一致）：

```python
def frame(module_id, second_id, data6):
    p = [module_id, 0, second_id] + list(data6) + [0]
    cs, head = p[0], 0
    for i in range(8, 1, -1):
        head = (head << 1) & 0xFF | ((p[i] & 0x80) >> 7)
        p[i] |= 0x80
        cs = (cs + p[i]) & 0xFF
    p[1] = head | 0x80
    p[9] = ((cs + p[1]) & 0xFF) | 0x80
    return " ".join("%02X" % x for x in p)

gain, offset = 1636, -285
print(frame(0x33, 0x89, [gain & 0xFF, gain >> 8,
                         offset & 0xFF, (offset >> 8) & 0xFF, 0, 0]))
```

### 12.7 第 7 条 · 手指脱落 vs 探头脱落

两种脱落必须在电学上可分，依据是“光电管到底有没有收到 LED 的光”
（归一后约定：**光越强 → 码值越小**）：

| 状态 | LED 电流 | PD 收光 | 归一后码值 | 脉动 PI | 判别 |
|---|---|---|---|---|---|
| 正常 | 有 | 有（穿过手指/经反射） | 在调光窗内（≈中线） | 3% 左右 | `offState = 0` |
| **手指脱落** | 有 | 直射/漏光，光很强 | 逼近 **0**（下饱和） | ≈0 | `offState = 1` |
| **探头脱落** | **无**（开路） | 无，全黑 | 逼近 **4095**（上饱和） | ≈0 | `offState = 2` |

实现：`spo2_light.c::DetectProbeOff()`（两个通道**同时** ≥ `SPO2_PROBE_OFF_CODE_MIN=3900`
连续 5 次 → 探头脱落，优先级最高，一旦确认就停止一切调光动作），之后才走原来的
“任一通道 < `SPO2_IR/RED_OFF_MAX_VAL` 判手指脱落”分支。

> 门限的实测依据（`SPO2_FE_INVERT=0` 下）：手指拿掉时 IR 原始 ADC ≈ **95**，
> MATLAB 原值 80 刚好卡在它之下会漏判，所以放宽到 **150**（见 1.2 的实测表）。
> 探头判定用的 3900 则对应“光电管一点光都没收到”，两侧相差 4095−95≈4000 码，
> 不会互相误判。

验收操作（三步，判据明确）：

1. **探头拔掉**（或 H4 排针拔开）→ 发 `0x8B`，应打印 `offState=2 (PROBE-OFF)`；
2. **插回探头、不夹手指** → 应打印 `offState=1 (FINGER-OFF)`；
3. **夹上手指** → `offState=0 (OK)`、`adjDone=1`、`PI≈3.00%`。

> 若第 1、2 步的结果反了，说明你的前端极性是反的（`SPO2_FE_INVERT` 要改成 0）；
> 若两步都报同一种脱落，说明探头是反射式（见第 11 节第 4 条的说明，把 `SPO2_PROBE_OFF_EN` 置 0）。

### 12.8 前置条件 · 导出 1024 点真实 AD 数据

1. 手指夹好、等 `adjDone=1`（保证光强已标定，数据才有意义）；
2. 发 `33 81 8A 80 80 80 80 80 80 BE`；
3. 把串口收到的 `===SPO2 AD DUMP BEGIN===` 与 `===SPO2 AD DUMP END===` 之间的内容
   存成 `.csv`：每行 8 组 `ir,red`（共 1024 组，约 1.28s 发完）；
4. 导出结束后固件会打印 `dropped=N bytes`，**N 必须为 0**（有丢字节就降低发送速率：
   把 `SPO2_DUMP_TICK_DIV` 调大或把 `SPO2_DUMP_PER_LINE` 调小，但务必保持
   `PER_LINE > TICK_DIV`，否则倒序读取会被写指针追上、数据出现撕裂）。

> 顺带：`0x33/0x02` 波形帧本身就是原始 AD（红外/红光各 24bit），用上位机录 2 秒也有 1000 点，
> 且能同时看到波形，做实验报告更直观。

### 12.9 验收建议顺序（一次跑完，避免反复插拔）

```
⓪ 接线自查：H4-1 对 GND、H4-2 对 +5V；5 根信号线按第 1 节表接好（别接反，见 1 节的警告）
① 上电 → 看 PLL lock=1、打印的 [accept] 行与配置是否符合（第 8 节 T1）
② 极性标定：探头空置看 irAd ≈ 0（第 8 节 T2）
③ SPO2_CAP 定常态：发 0x8C 的动作 0/1 各看一次波形（见 1.5），确定"常态电平"
④ 第 7 条：探头拔掉 / 不夹手指 / 夹上手指 三态各读一次 0x8B   →  判 offState 0/1/2
⑤ 第 1 条：看 adjDone 0→1、irDA/redDA 稳定                   →  判自动调光
⑥ 第 4 条：读 0x8B 的 PI                                     →  判 3.00%
⑦ 第 2 条：导出 1024 点 AD，看静态段无明显漂移、毛刺小        →  判噪声
⑧ 第 6 条：模拟器 99/90 两点标定（12.6），再用 95 复核        →  判 ±2%
⑨ 第 3 条：模拟器 30/60/120/180/200 逐档读 0x8B 的 pr        →  判 ±5%
⑩ 第 5 条：确认读数始终在 90~99（不会出现 100 或 <90）        →  判范围
```

---

## 13. 一句话总结

算法本体（滤波 / 脉率 / SpO2 / 光强自动调节 / 幅度谱）全部搬到 `Alg/`，与硬件零耦合、
与 MATLAB 逐样本等价；硬件差异（单路 VSIG 分时复用 IR/RED）收在新增的 `HW/SPO2FE` 里；
数据源、协议命令表、上传帧按血氧重新实现；Keil 工程只动了 IncludePath、Groups 和一个
DAC 驱动源文件。

面向《五参数验收标准》的血氧条目，本轮又补了 6 件事：

1. **脉率放宽到 30~200bpm**，并把不应期从 250 样本同步改到 135（只改上限不改不应期会减半）；
2. **灌注指数 PI** 计算与上报（验收要求模拟器固定 3%），并顺带用作“有光路无脉动”的判据；
3. **手指脱落 / 探头脱落双判别**：用“码值趴在 0 侧还是 4095 侧”区分两种截然不同的物理状态；
4. **ADC 4 次过采样平均**，把随机噪声降到 1/2，保住 PI=3% 时血氧 ±2% 的分辨率预算；
5. **R 值运行时标定命令 `0x89`**（跨复位保留）+ **1024 点原始 AD 导出命令 `0x8A`**
   （对应验收前置条件“≥1000 点真实数据”）；
6. 修正了血氧**模块 ID**（必须 `0x33`，与 MATLAB 上位机的 `unpackRslt(1)==51` 一致；
   写成 `0x13` 会让整条链路静默失效）。

**30 个源文件 + 启动文件 0 error 0 warning 编译链接通过**，
RAM 31.90kB / ROM 22.00kB（F407VET6 有 128+64kB SRAM、512kB Flash，余量充足）。

---

## 14. 2026-10-09 补强：实时性与可观测性（模仿心电工程的写法）

心电工程（`D:\SZU\心电\算法\STM\心率采集算法final\STM32F407_ProjectTemplate`）与本工程共用
同一副骨架：`UART1.c / Queue.c / Timer.c / RCC.c / LED.c / SysTick.c / NVIC.c / PackUnpack.c /
DataType.h` 经 MD5 校验**逐字节相同**。问题在于：心电在演进中修掉的几个坑，血氧这边还留着；
而血氧的"主循环自己踩采样节拍"让这些坑从无害变成致命。本轮按心电的写法补齐六件事。

### 14.1 `DelayNus()` 改用 DWT 周期计数（`ARM/SysTick/SysTick.c`）

旧实现是 `for(i=0;i<7;i++)` 空循环，注释自己就写着"按 72MHz 标定，168MHz 下偏短约一半"，
且循环变量无 `volatile`——提高优化等级后整个内层循环可能被删掉，`SPO2_FE_SETTLE_US=100us`
的通道建立时间会趋近 0（后果是 IR 串进 RED 的长尾）。

新实现 `InitCycleTimer()` 使能 `DWT->CYCCNT`（`InitSysTick()` 里最先调用），按
`SystemCoreClock` 标定每微秒周期数；计数器异常时有 `guard` 上限兜底退出，绝不无限死等。
**影响**：建立延时与 ADC 时序现在是真值，第 1.1 节的"2ms 预算占 19%"应按实测复核。

### 14.2 屏级重算挪出 2ms 采样拍（`Alg/spo2_alg.c` + `app/Main/Main.c`）

2048 点 FFT + 屏统计在 168MHz 下要 1~4ms。原来它跑在 `spo2_alg_step()` 里，等于把
"IR 采完到 RED 采完"之间拉开好几毫秒——**两个通道不再成对同时刻，R 值与 PI 的口径直接被
污染**，而这正是血氧 ±2% 最吃不起的误差。

现在：`spo2_alg_step()` 满屏只置 `screenPending`；主循环空闲时调用
`ProcSPO2HeavyTask()` → `spo2_alg_screen_run()` 取走。数值路径（屏统计→光强均值回灌→
结果快照→PI 判据→请求重调→FFT）与调用顺序**一字未改**，只是换了执行时机。

副作用与边界：重算期间采样照跑，刚收满那一屏的缓冲会有 1~2 个点被新屏覆写
（= 重算耗时/2ms），与 MATLAB"收满一屏再算"的数值行为一致，误差在 2/2048 量级。
若主循环长时间取不走（`lostScreen` 增长）才需要处理。

### 14.3 丢拍统计（`HW/Timer/Timer.c`）

时基是"单位标志位 + 主循环轮询"，慢过 2ms 就少采一对 IR/RED 且**原本毫无痕迹**。
现在 ISR 里记账：`Get2msTickCnt()`（到期拍数，每秒 +500）与 `Get2msMissCnt()`（丢拍数）。

### 14.4 峰值耗时统计（`app/Main/Main.c`）

`SPO2GetTickMaxUs()`（单拍峰值）、`SPO2GetHeavyMaxUs()`（屏级重算峰值 = FFT 真实开销）、
`SPO2GetHeavyRunCnt()`、`SPO2GetScreenLostCnt()`。拍内只多两次 DWT 读数与一次比较。

### 14.5 日志改走发送队列（`app/SendDataToHost/*`、`app/ProcHostCmd/*`）

`printf` 经 `fputc` 是"直写 DR + 死等 TC"，115200 下每字节约 87us；`0x84` 状态查询连打
10 行（≈600 字节）会堵住 2ms 任务约 50ms（丢约 25 对采样），还会与 TXE 中断并发争抢串口，
破坏正在发送的协议帧/CSV 字节间隔。**心电工程早已改走队列**（其 `Main.c` 状态行注释），
血氧这边新增 `SendTextToHost()` / `LogPrintf()`（static 缓冲，不压 2KB 栈；整条一次入队），
`ProcHostCmd.c` 27 处 `printf` 全部切换，`Main.c` 的 AD 导出 BEGIN/END 头也切换。
上电自检 `PrintBootInfo()` **仍用 printf**（那时队列与中断都还没跑，与心电一致）。

### 14.6 修正 `rCalGainPm` 的 u8 截断（`Alg/spo2_alg.h`）

原 `uint8_t` 存 int16 标定增益：按第 12.6 节例子 `gain=1636` 会被截成 **116**，
`0x84/0x8B` 的 `calGain` 回显失真，标定时极易误判"参数没生效"。改为 `int16_t` 并去掉强转。

### 14.7 命令与统计口径

| 命令 | 变化 |
|---|---|
| `33 81 84 ... B8` 查状态 | 新增一行 `tick: beat= miss= | tickMax= heavyMax= heavyRun= lostScreen=`，字段含义见 T6 |
| `33 81 88 ... BC` 清统计 | 除 ADC 外，一并清 `ClearTickStatic()`（拍数/丢拍）与 `SPO2ClrTickStat()`（耗时峰值/丢屏） |

### 14.8 尺寸与构建

MicroLIB 真实构建（`UV4.exe -r`）：**0 Error / 0 Warning**，
`Code=21812 RO=1772 RW=148 ZI=32132`（改动前 `Code=18376 RO=1608 RW=120 ZI=31928`）。
+3.4kB Flash 主要来自 `vsnprintf` 与新增函数；RAM +0.2kB。ROM/RAM 余量仍极充足。

命令行校验脚本已按心电的 `tools/cc_check.ps1` 移植为 **`tools/cc_check.ps1`**
（30 个源文件 + 启动文件，加 `-Link` 走 armlink 出尺寸）。注意命令行链的是标准 C 库，
`vsnprintf` 之类能否解析**必须以 UV4 的 MicroLIB 构建为准**。

### 14.9 本轮的改动方式（GBK 源文件必读）

本工程 56 个源文件里 **55 个是 GBK(代码页 936)**，只有 `app/ProcHostCmd/ProcHostCmd.c` 是
UTF-8。用按 UTF-8 解码的编辑工具直接改 GBK 文件会把中文注释写成乱码，因此本轮改动全部通过
**`_patch/apply_hunks.ps1` + `_patch/h_*.txt`** 完成：锚点取纯 ASCII 代码行（或已核实的中文原文），
脚本用 `GetEncoding(936)` 读写，并强制"锚点必须唯一命中"才落盘。改动前的 10 个文件已备份到
`_backup_20261009/`。

> 后续若把源文件统一成一种编码（第 11 节之外的建议），这套补丁流程可以退休，改用常规编辑。
