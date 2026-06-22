# TriVital-Monitor

TriVital-Monitor 是一个生命体征监测示例工程，仓库包含两部分：

- `嵌入式软件部分`：基于 STM32F103RC 的下位机采集、处理和串口发送工程。
- `上位机部分/ParamMonitorHost`：基于 PyQt5 的 PC 端监护界面，用于串口接收、波形绘制、参数显示和报警提示。

项目面向学习、课程设计和原理验证，重点展示 ECG、RESP、SpO2 三类信号从嵌入式采集处理到上位机显示的完整链路。

## 功能概览

### 下位机

- 使用 Keil MDK 工程 `嵌入式软件部分/Project/STM32KeilPrj.uvprojx`。
- 目标芯片为 `STM32F103RC`，工程使用 ARMCC V5。
- 初始化 RCC、NVIC、SysTick、Timer、UART1、ADC、DAC、OLED、LED 等外设。
- 通过 ADC 获取 ECG、RESP、SpO2 相关信号。
- ECG 模块完成滤波、平滑、R 波/心率计算和导联状态判断。
- RESP 模块完成低频呼吸波处理、呼吸率计算和导联状态判断。
- SpO2 模块完成红光/红外 LED 控制、采样、滤波、R 值分析、血氧计算和光强调节。
- OLED 本地显示 ECG、RESP、SpO2 参数。
- UART1 以 115200 波特率向上位机发送波形、参数和状态数据包。

### 上位机

- 入口文件：`上位机部分/ParamMonitorHost/main.py`。
- 使用 PyQt5 构建主窗口，使用 pyserial 读取串口数据。
- 显示 ECG、SpO2、RESP 三路实时波形。
- 显示心率、血氧、呼吸率和各通道导联状态。
- 支持串口选择、波形暂停、清屏、报警静音。
- 提供协议调试面板，显示接收字节、包计数、校验/同步错误等信息。
- 内置报警阈值判断：心率过高/过低、呼吸过高/过低、血氧过低、导联异常。

## 目录结构

```text
TriVital-Monitor/
├── README.md
├── LICENSE
├── 嵌入式软件部分/
│   ├── App/
│   │   ├── ECG/              # ECG 采集处理与心率计算
│   │   ├── RESP/             # 呼吸信号处理与呼吸率计算
│   │   ├── SPO2/             # 血氧采样、滤波、血氧计算和调光
│   │   ├── Main/             # 系统初始化与周期任务调度
│   │   ├── OLED/             # OLED 显示
│   │   ├── PackUnpack/       # 串口协议打包/解包
│   │   ├── SendDataToHost/   # 下位机到上位机的数据发送
│   │   └── ProcHostCmd/      # 上位机命令解析预留
│   ├── ARM/                  # 启动文件、系统文件、SysTick、NVIC
│   ├── FW/                   # STM32F10x 标准外设库
│   ├── HW/                   # ADC、DAC、RCC、Timer、UART1 等驱动
│   └── Project/              # Keil 工程
└── 上位机部分/
    └── ParamMonitorHost/
        ├── main.py           # 上位机入口
        ├── ParamMonitor.py   # 主窗口、串口接收、波形绘制、数据解析
        ├── PackUnpack.py     # Python 端协议解包
        ├── form_setuart.py   # 串口设置窗口
        ├── monitor_alarm.py  # 报警阈值与报警判断
        ├── ui_theme.py       # 上位机界面样式
        └── requirements.txt  # Python 依赖
```

## 数据链路

下位机主循环中包含两个主要周期任务：

- `Proc2msTask`：每 2 ms 检查一次标志，约每 4 ms 执行 ECG、RESP、SpO2 实时处理，并发送一包三路波形数据。
- `Proc1SecTask`：每 1 s 刷新 OLED，并发送一包参数数据和一包状态数据。

上位机通过串口接收定长数据包，完成解包后按模块 ID 分发：

| 模块 ID | 含义 | 上位机处理 |
| --- | --- | --- |
| `0x10` | 波形数据 | `analyzeWaveData`，绘制 ECG/RESP/SpO2 波形 |
| `0x11` | 参数数据 | `analyzeParamData`，显示心率、呼吸率、血氧 |
| `0x12` | 状态数据 | `analyzeStatusData`，显示导联/传感器状态 |

## 串口协议

当前协议为 10 字节定长包：

```text
Byte0   模块 ID，最高位为 0
Byte1   数据头，用于保存后续字节原始最高位
Byte2   二级 ID
Byte3-8 6 字节有效数据
Byte9   校验和
```

协议打包会将 `Byte1-Byte9` 的最高位置 1，因此接收端可以通过“模块 ID 小于 `0x80`、后续字节大于等于 `0x80`”进行同步。C 端实现位于 `嵌入式软件部分/App/PackUnpack`，Python 端实现位于 `上位机部分/ParamMonitorHost/PackUnpack.py`。

三类有效数据格式如下：

```text
波形包 0x10:
data[0:1] ECG  int16，高字节在前
data[2:3] RESP int16，高字节在前
data[4:5] SpO2 int16，高字节在前

参数包 0x11:
data[0:1] 心率 bpm
data[2:3] 呼吸率 bpm
data[4:5] 血氧 %

状态包 0x12:
data[0] ECG 导联状态，0 异常，1 正常
data[1] ECG 报警状态，当前下位机发送 0
data[2] RESP 导联状态，0 异常，1 正常
data[3] RESP 报警状态，当前下位机发送 0
data[4] SpO2 状态，0 异常，1 正常
data[5] SpO2 报警状态，当前下位机发送 0
```

## 运行上位机

进入上位机目录后安装依赖并运行：

```powershell
cd 上位机部分\ParamMonitorHost
python -m pip install -r requirements.txt
python main.py
```

打开程序后，在工具栏点击“串口”，选择下位机对应的串口号。下位机代码默认 UART1 波特率为 `115200`，数据位、停止位、校验位按串口设置窗口选择。

## 编译下位机

1. 使用 Keil MDK 打开 `嵌入式软件部分/Project/STM32KeilPrj.uvprojx`。
2. 确认目标设备为 `STM32F103RC`。
3. 确认已安装 STM32F1 设备包。
4. 编译生成 `Objects/STM32KeilPrj.hex`。
5. 通过调试器或下载工具烧录到目标板。

工程注释中提示：若使用 `printf` 串口输出，需要在 Keil 中启用 `Use MicroLIB`。

## 上位机打包

仓库保留了 `main.spec`，可使用 PyInstaller 打包：

```powershell
cd 上位机部分\ParamMonitorHost
pyinstaller main.spec
```

`build/` 和 `dist/` 属于构建产物，不建议提交到仓库。

## 注意事项

- 本项目仅用于学习、教学和工程原理验证，不用于医疗诊断或临床用途。
- 生命体征算法和阈值未经过医疗器械级验证。
- 上位机日志默认写入 `上位机部分/ParamMonitorHost/logs/host_monitor.log`。
- Keil 的 `Objects/`、`Listings/`、`.uvguix.*`，Python 的 `__pycache__/`、PyInstaller 的 `build/`/`dist/` 都属于本地生成物。

## License

本项目使用 MIT License，详见 `LICENSE`。
