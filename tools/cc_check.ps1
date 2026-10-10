# ---------------------------------------------------------------------------
# cc_check.ps1 : command-line full compile + link check for the SpO2 project.
#   Ported from the ECG project's tools/cc_check.ps1 ("same style, same gates").
#   Usage:  powershell -NoProfile -ExecutionPolicy Bypass -File tools\cc_check.ps1
#           (add -Link to also run armlink, -All to rebuild every source)
# ---------------------------------------------------------------------------
param(
  [switch]$Link,
  [string[]]$Only = @()     #e.g. -Only SysTick.c,Timer.c for a quick subset check
)

$ErrorActionPreference = "Continue"
$root = Split-Path $PSScriptRoot -Parent
$tool = "D:\Keil_v5\ARM\ARMCC\bin"
$out  = Join-Path $env:TEMP "spo2_objs"

if (-not (Test-Path (Join-Path $tool "armcc.exe"))) { throw "armcc not found at $tool" }

if (Test-Path $out) { Remove-Item $out -Recurse -Force }
New-Item -ItemType Directory -Path $out | Out-Null

$incDirs = @("app\Main","app\DataType","app\PackUnpack","app\ProcHostCmd","app\SendDataToHost",
             "HW\RCC","HW\Timer","HW\UART1","HW\ADC","HW\SPO2FE","HW\LED","ARM\NVIC","ARM\SysTick",
             "Alg","module",
             "libraries\CMSIS\Include",
             "libraries\CMSIS\Device\ST\STM32F4xx\Include",
             "libraries\STM32F4xx_StdPeriph_Driver\inc")
$incArgs = @(); foreach ($d in $incDirs) { $incArgs += @("-I" + (Join-Path $root $d)) }

$srcs = @(
  # APP
  "app\Main\Main.c","app\ProcHostCmd\ProcHostCmd.c","app\PackUnpack\PackUnpack.c",
  "app\SendDataToHost\SendDataToHost.c",
  # ALG
  "Alg\spo2_alg.c","Alg\spo2_filter.c","Alg\spo2_coeffs.c","Alg\spo2_pulse.c",
  "Alg\spo2_calc.c","Alg\spo2_light.c",
  # HW
  "HW\RCC\RCC.c","HW\Timer\Timer.c","HW\UART1\UART1.c","HW\UART1\Queue.c",
  "HW\ADC\ADC.c","HW\SPO2FE\SPO2FE.c","HW\LED\LED.c",
  # ARM / MODULE / CMSIS
  "ARM\NVIC\NVIC.c","ARM\SysTick\SysTick.c","module\stm32f4xx_it.c",
  "libraries\CMSIS\Device\ST\STM32F4xx\Source\Templates\system_stm32f4xx.c",
  # Driver (only what the Keil group actually holds)
  "libraries\STM32F4xx_StdPeriph_Driver\src\misc.c",
  "libraries\STM32F4xx_StdPeriph_Driver\src\stm32f4xx_rcc.c",
  "libraries\STM32F4xx_StdPeriph_Driver\src\stm32f4xx_gpio.c",
  "libraries\STM32F4xx_StdPeriph_Driver\src\stm32f4xx_usart.c",
  "libraries\STM32F4xx_StdPeriph_Driver\src\stm32f4xx_tim.c",
  "libraries\STM32F4xx_StdPeriph_Driver\src\stm32f4xx_adc.c",
  "libraries\STM32F4xx_StdPeriph_Driver\src\stm32f4xx_dac.c",
  "libraries\STM32F4xx_StdPeriph_Driver\src\stm32f4xx_dma.c"
)

$objs = @(); $err = 0; $warn = 0
foreach ($s in $srcs) {
  $leaf = Split-Path $s -Leaf
  if (($Only.Count -gt 0) -and ($Only -notcontains $leaf)) { continue }

  $fi = Join-Path $root $s
  if (-not (Test-Path -LiteralPath $fi)) { Write-Output "NOT FOUND: $s"; $err++; continue }

  $o = Join-Path $out ($leaf -replace '\.c$','.o')
  $cargs = @("--cpu","Cortex-M4.fp","-c","--c99","-O1","--apcs=/interwork",
             "-DUSE_STDPERIPH_DRIVER","-DSTM32F40_41xxx") + $incArgs + @("-o",$o,$fi)

  $res = & "$tool\armcc.exe" @cargs 2>&1
  if ($LASTEXITCODE -ne 0) {
    $err++; Write-Output "FAIL: $s"
    $res | ForEach-Object { Write-Output ("     " + (($_ -split "`n")[0])) }
  }
  elseif (((($res | Out-String).Trim()).Length) -gt 0) {
    $warn++; Write-Output "WARN: $s"
    $res | ForEach-Object { Write-Output ("     " + (($_ -split "`n")[0])) }
  }
  if (Test-Path $o) { $objs += $o }
}

Write-Output ("==== compile: {0} failed / {1} warning / {2} compiled ====" -f $err, $warn, $objs.Count)

# startup: armasm must get the same __MICROLIB define uVision passes, otherwise the
# two-region-memory branch is taken and armlink reports L6218E (__initial_sp).
$sa = Join-Path $root "libraries\CMSIS\Device\ST\STM32F4xx\Source\Templates\arm\startup_stm32f40xx.s"
$so = Join-Path $out "startup.o"
& "$tool\armasm.exe" --cpu Cortex-M4.fp --pd "__MICROLIB SETA 1" -o $so $sa 2>&1 |
  ForEach-Object { Write-Output ("armasm: " + (($_ -split "`n")[0])) }
$objs += $so

if ($Link) {
  $axf = Join-Path $out "SPO2_Precheck.axf"
  $largs = @("--cpu","Cortex-M4.fp","--strict","--entry","Reset_Handler","--first","startup.o(RESET)",
             "--ro-base","0x08000000","--rw-base","0x20000000","--info=sizes","-o",$axf) + $objs
  & "$tool\armlink.exe" @largs 2>&1 |
    Select-String -Pattern "Error|Warning|Total RO  Size|Total RW  Size|Total ROM Size" |
    ForEach-Object { Write-Output $_.ToString().Trim() }
  Write-Output "link exit = $LASTEXITCODE"
  if (Test-Path $axf) {
    & "$tool\fromelf.exe" --i32 -o (Join-Path $out "SPO2_Precheck.hex") $axf | Out-Null
    Write-Output ("hex -> " + (Join-Path $out "SPO2_Precheck.hex"))
  }
}

Write-Output ("UV4 running now = " + (Get-Process UV4 -ErrorAction SilentlyContinue | Measure-Object).Count)
