# SpO2（血氧饱和度 + 脉率）STM32F407 嵌入式实现

手指式血氧探头的**采集 · 滤波 · 脉率 · 血氧 · 光强闭环 · 脱落判别**全套 MCU 固件，算法逐条对齐 MATLAB 原型工程 `SPO2DynamicPrj`（`RealAnalyzeData.m` / `CalcPulseRate.m` / `CalcSPO2.m` / `RoughAdj.m` / `FineAdj.m`），并针对"五参数血氧验收标准"补齐了 PI、三态脱落、R 值现场标定与原始 AD 导出。

| 项 | 值 |
|---|---|
| MCU | STM32F407VET6（Cortex-M4F @168MHz，硬件 FPU） |
| 工具链 | Keil MDK，ARMCC V5.06 update 5 (build 528)，`-O1` |
| 采样率 | 每通道 **500 SPS**（2ms 一拍，每拍采一对 IR+RED） |
| 处理窗口 | 2048 点 = **4.096 s** / 屏（一屏出一次 R 值、PI、血氧） |
| ROM / RAM | 28 204 B（27.5 KB） / **11 880 B（11.6 KB）** |
| 源码规模 | 43 个 `.c/.h`，8 186 行（其中纯代码约 4 400 行） |
| 编译校验 | `tools/cc_check.ps1 -Link -All` → **0 error / 0 warning**（命令行全量编译 + 链接） |
| 串口 | UART1 `PA9/PA10` @115200，VOFA+ FireWater 波形 + 二进制命令帧 |

---

## 1. 硬件接口（血氧板 H4 排针）

| 信号 | MCU 引脚 | 外设 | 作用 |
|---|---|---|---|
| `SPO2_VSIG` | PA5 | ADC1 通道（放大滤波后的脉搏波） | 模拟输入，ADCCLK = PCLK2/4 = 21MHz |
| `SPO2_DAI` | PA4 | DAC 通道 1 | 发光电流（光强），算法侧用 0~250 的 DA 值，映射 `code = da × 4095 / 250` |
| `SPO2_IR_CS` | PC4 | GPIO 推挽 | 选通红外 LED |
| `SPO2_RED_CS` | PC5 | GPIO 推挽 | 选通红光 LED |
| 状态 LED | PB2 | GPIO | 500ms 翻转，证明主循环在跑 |

**IR 与 RED 共用一路 DAC，但光强是分别控制的**：`SPO2FE_Select()` 在切通道的同时把该通道对应的光强码写进 DAC（`HW/SPO2FE/SPO2FE.c`）。所以"红外亮时给红外光强、红光亮时给红光光强"，两路 AGC 互不干扰，而不需要第二个 DAC。

一拍的采集时序（`SPO2FE_ReadPair()`）：

```
选通 IR → DelayNus(settle=100) → ADCConvertVsigAvg(过采样 4 次取均值) → 选通 RED → 建立延时 → 过采样平均
```

建立延时与过采样次数都是现场标定项：延时太短会让 IR 通道的余尾串进 RED（直接污染 R 值），太长会吃掉 2ms 预算。

---

## 2. 目录结构

```
Alg/        算法本体，零外设访问，可整体搬走（13 文件 / 2 619 行）
  spo2_config.h   ★ 所有算法指标、门限、标定系数的唯一定义处（对应 MATLAB global 变量）
  spo2_filter.*   扣直流 → 0.5Hz 高通 → [50Hz 陷波] → 5 阶 20Hz 低通（DF2T，逐系数对齐 scipy/MATLAB）
  spo2_coeffs.*   滤波器系数（gen_spo2_coeffs.m 生成）+ 11 档 R-SpO2 标定表
  spo2_pulse.*    逐点寻峰（块极值自适应阈值 + 迟滞确认 + 不应期）→ RR 中值 → 6 点脉率平滑
  spo2_calc.*     屏级统计（max/min/rng/avg）→ PI → R 值 → 查表血氧；R 值现场标定
  spo2_light.*    光强自动调节状态机（粗调 / 细调 / 重调 / 复位）+ 手指/探头脱落判别
  spo2_alg.*      上述模块的唯一聚合入口：step(每拍) + screen_run(每屏，挪到主循环空闲时)
app/        业务层：Main（2ms/1s 任务、VOFA 上报、AD 导出）、PackUnpack、ProcHostCmd、SendDataToHost
HW/         RCC / Timer(TIM4 1ms→2ms 时间片 + 丢拍统计, TIM5→1s) / UART1(中断收发队列) / ADC / SPO2FE / LED
ARM/        NVIC / SysTick（DelayNus 走 DWT 周期计数，不靠指令空转）
module/     stm32f4xx_it.c / stm32f4xx_conf.h
project/MDK(V5)/Project.uvprojx     Keil 工程
tools/cc_check.ps1                  命令行全量编译 + 链接校验（改动后的回归闸门）
docs/MIGRATION_NOTES.md             移植与设计长文（992 行，含 MATLAB 逐行对应关系）
```

---

## 3. 快速开始

**编译**：用 Keil 打开 `project/MDK(V5)/Project.uvprojx`。首次打开请先读 `docs/MIGRATION_NOTES.md` §5.5——本机若默认 ARMCLANG（V6），必须把 TIF 换成 **ARMCC V5.06**，否则工程名旁的编译器下拉框会与源码里的 ARMCC 语法不符。

**不开 Keil 做回归校验**：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools\cc_check.ps1 -Link -All
# 期望输出：==== compile: 0 failed / 0 warning / 29 compiled ====  且 link exit = 0
```

**接串口**：115200-8-N-1。上电先打一段自检信息（时钟是否 PLL lock、采样率、屏长、极性、脱落门限、R 标定、`[accept]` 验收口径）。波形用 **VOFA+ FireWater**（默认 250 帧/s，约占串口带宽 26%），5 个参数槽位为 `PI / PR / SpO2 / DAC / OFF`。

**主要命令（MODULE_SPO2 = 0x33）**

| 二级 ID | 命令 | 用途 | 与验收的关系 |
|---|---|---|---|
| 0x80 | 采样开关 | 1 开始 / 0 停止 | 上位机原生命令 |
| 0x81 | 手动光强 | IR/RED 的 DA 值 | 自动调光失效时的旁路 |
| 0x82 | 查询结果 | 回血氧 + 脉率 + 主频 | 血氧/脉率 |
| 0x83 | 算法复位 | 等价 `InitData + ResetAdj` | 换手指/换探头后重标定 |
| 0x84 | 查询状态 | 打印一拍耗时峰值、丢拍、ADC 超时/溢出 | 实时性证据 |
| 0x85 | 设建立延时 | μs，钳 [5,1000] | 通道串扰标定 |
| 0x86 | 设极性 | `SPO2_FE_INVERT` 运行时切换 | 脱落判据方向的前提 |
| 0x87 | 切 ADC 通道 | 含内部 VREFINT 自检 | 前端排查 |
| 0x88 | 清零统计 | ADC / 丢拍 / 耗时峰值 | 换一项测试前复位证据链 |
| 0x89 | **R 值标定** | gain(千分比) + offset，跨复位保留 | **血氧 ±2% 的唯一调节手段** |
| 0x8A | **导出 AD** | 倒序 1024 点真实原始码值，非阻塞 ~1.28s | 实验报告要求的 ≥1000 点数据 |
| 0x8B | **查询指标** | PI×100 + 脱落状态 + R 值 | **灌注指数、脱落判别** |

---

## 4. 数据流

```
2ms 拍（Proc2msTask）
  ├─ SPO2FE_ReadPair()      → irAd, redAd（500 SPS/通道）
  ├─ spo2_alg_step()
  │    ├─ 仅 IR 通道 → spo2_filt_step() → spo2_pulse_step() → 脉率
  │    ├─ 原始 IR/RED 写入 2048 点环形屏缓冲；满一屏只置 screenPending 标志
  │    └─ spo2_light_*()    → 每 50 拍(0.1s) 跑一次粗调/重调/细调，算出 irDa/redDa
  └─ SPO2FE_SetIntensityDa() → 下一拍生效；VOFA 文本帧压入发送队列（非阻塞）

主循环空闲（ProcSPO2HeavyTask，绝不在 2ms 拍内）
  └─ spo2_alg_screen_run()
       屏统计(max/min/rng/avg) → PI → R 值中值(6 点) → 查表血氧 → 光强回看 → 请求重调
```

**为什么屏级重算要挪出采样拍**：2048 点统计在 168MHz 下要数百 µs，若卡在 2ms 拍内会把"IR 采完到 RED 采完"的间隔拉开，红外与红光就不再是同一时刻的血氧，R 值和 PI 的口径直接被污染。挪出后用三个指标自证：`tickMax`（单拍最长耗时，预算 2000µs）、`heavyMax`（屏级重算耗时）、`lostScreen`（重算没被取走就被覆写的屏数，**应恒为 0**）。

**脱落三态**（`spo2_light.c`）：

| 状态 | LED | PD 收到 | 归一码值 | 判据 | 确认时间 |
|---|---|---|---|---|---|
| 正常 | 亮 | 透过手指 | 电窗口内 | 有搏动 + PI 达标 | — |
| 手指脱落 | 亮 | 直射/漏光，光很强 | 贴 0 一侧 | 任一路 < 150 连续 20 拍，或 PI < 0.20% 连续 2 屏 | ~2 s |
| 探头脱落 | 灭（断路） | 全黑 | 贴 4095 一侧 | 两路同时 ≥ 3900 连续 5 拍 | ~0.5 s |

任一脱落态下血氧与脉率一律置 `-100`（上位机显示 `---`），不会保留上一次的数值。

---

## 5. 验收对照（当前状态）

| 验收项 | 实现 | 状态 |
|---|---|---|
| 自动调光 | 粗调(0.1s 节拍 × 每 3 次动一步) → 重调判据 → 细调(±1/屏)，IR/RED 独立 DA | ✅ 已实现，收敛时间待实测 |
| 滤随机噪声 | ADC 过采样 4 次 + 5 阶 20Hz 低通 + 10 点…（脉率通道）；50Hz 陷波已备好，默认直通 | ✅ 脉率通道到位 |
| 滤基线漂移 | 扣 1.65V 中点 + 0.5Hz 高通 | ✅ |
| 脉率 30~200BPM ±5% | `PR_MIN=30/PR_MAX=200`，`MIN_DIST=135 点`(0.27s)，RR 8 点中值 + 脉率 6 点中值，越界报 -100 | ⚠️ 覆盖到位，存在 −0.5bpm 截断偏差与 2 个未标定门限 |
| 灌注指数 3% | `PI = rng × 50 / avg`（×100 定点），走 0x8B 帧上报，并兼作手指在位判据 | ✅ 已实现，口径待与模拟器核对 |
| 血氧 90~99%，±2% | 11 档 `RR_TABLE` + 6 点中值 + `spo2 = 101 − index`；`R_used = R_raw × gain/1000 + offset` 由 0x89 现场写入且跨复位保留 | ⚠️ 机制完整，**gain/offset 未标定前无法保证** |
| 自动判别手指/探头脱落 | 上表三态，判据相互独立 | ✅ 已实现，极性前提见下 |

**尚未完成的 5 个 `[实测]` 参数**（代码里明确标注，不是遗漏而是需要上机）：`SPO2_PR_ABS_MIN`、`SPO2_PR_THR_ALPHA`、`SPO2_FE_INVERT` 及其派生脱落门限、R 值 `gain/offset`、`SPO2_FE_SETTLE_US`；外加 PI 的定义口径（峰-峰/2/DC vs rms/DC，模拟器厂商口径）。

**已知缺口（建议按此顺序修）**：

1. `spo2_calc.c` 的 R 值与 PI 建立在**未滤波原始码值的整屏 max−min** 上，单个毛刺就会撑大摆幅 → 直接吃 ±2% 预算。改法：百分位统计或 3 点限幅中值。
2. 脉率 `prRaw = (int16)(60·Fs/medRR)` 向下截断，低心率端系统性偏低约 0.5 bpm。改法：`+0.5f` 四舍五入。
3. `spo2_light_apply_manual()`（手动光强）会把 `fingerOn/probeOn/offState` 强制成正常 → 手动模式下脱落判别被屏蔽。改法：只冻结调光动作，保留脱落检测。

---

## 6. 2026-10-10 精简记录

| 动作 | 内容 | 收益 |
|---|---|---|
| 移出主树 | `app/main.c`（原厂示例，含重复 `main()`）、`board/`、`bsp/uart/`、`_backup_*/`、`_patch/` | 仓库少 393 行误导源码 |
| 移除 `SPO2_CAP` 整条链 | 基线复位钳位（`SPO2_CAP_EN` 原为 0，代码从未生效）；板级结论保留为 `spo2_config.h` 的 10 行注释 | −334 行，2ms 拍内少一段分支 |
| 移除死波形帧路径 | `SendSPO2WaveToHost` / `SendSPO2IntensityToHost` 及其分频计数 | −76 行 |
| 移除 FFT 模块 | `spo2_fft.c/.h`（每屏 2048 点幅度谱；算出的 `fundHz` 从未参与判定）；结果帧的 `fundHz` 两字节**保留且恒为 0**，上位机解析无需改动 | −394 行、**RAM −20.0 KB**、每屏少 1~4ms |
| 调试命令保留 | `SET_SETTLE / SET_INV / SET_CH / CLR_CNT` 全部保留 | 它们是上面 5 个待标定参数的现场工具，删了就自断验收路 |

源码总量 8 708 → **8 186 行**；RAM 32 368 → **11 880 B**。

---

## 7. 开发约定（务必先读）

- **源码编码是混的**：`Alg/ app/ HW/ ARM/ module/` 的 `.c/.h` 全部是 **GBK(CP936) 无 BOM + CRLF**，唯独 `app/ProcHostCmd/ProcHostCmd.c` 是 UTF-8；`*.md` 用 UTF-8（GitHub 渲染需要）。**用默认按 UTF-8 读写的编辑器直接改 GBK 源文件会把中文注释永久损坏**——改前先确认编码，保存时选"GBK/简体中文"。Keil 里若某文件中文显示乱码，多半就是编码选错。
- 算法参数只准写在 `Alg/spo2_config.h`，算法模块内不允许出现硬编码数字或外设访问。
- 每处偏离 MATLAB 原型的实现选择都要留注释说明"为什么"（历史上已踩过的坑见 `docs/MIGRATION_NOTES.md`）。
- 改完必须过 `tools/cc_check.ps1 -Link -All`，再进 Keil 出 hex；`Objects/ Listings/ *.hex *.o` 已由 `.gitignore` 排除，不进仓库。
- 数值上报一律定点（PI×100、R 取整、主频×100），不用 `%f`：既规避 MicroLIB 限制，也省掉浮点格式化的时间。

---

## 8. 文档

- `docs/MIGRATION_NOTES.md` —— 移植与设计长文：MATLAB ↔ C 的逐行对应、系数是怎么算出来的（可复现）、T1~T7 上机测试步骤与判据、串口带宽核算、以及 2026-10-09 那轮实时性补强。开头有**时效声明**，列出 2026-10-10 精简后已与代码不符的章节，读的时候先看它。

---

## 9. 许可

见根目录 [`LICENSE`](LICENSE)：**Copyright © 2018 - 2026 Jaziel.Jiang，保留所有权利（All rights reserved）**。
本仓库公开，仅授予"阅读与学习"的事实性可访问性，**不构成**使用、修改、再发布或商用的任何授权；
若需用于产品、实验申报或其他用途，请先取得 Jaziel.Jiang 的书面许可。

> 本实现的 SpO2 / 脉率输出用于算法研究与实验验证，**不构成医疗诊断依据**，不能替代已注册的医疗器械与专业判断。

Copyright © 2018 - 2026 Jaziel.Jiang. All rights reserved.
