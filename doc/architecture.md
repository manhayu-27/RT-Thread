# 项目架构图

> 本文档用 Mermaid 图表展示整个项目的模块结构，面向非技术人员，只需看图即可理解各模块如何协作。

---

## 一、系统运行架构（数据是怎么流动的）

> 一句话理解：**传感器感知身体 → STM32 采集翻译 → ESP32 无线转发 → 电脑/手机查看**。

```mermaid
flowchart TB
    %% ========== 第 1 层：物理世界 ==========
    subgraph SENSORS["🧬 传感器 · 感知身体的信号"]
        ADS["ADS1194 芯片<br/>心电 ECG / 肌电 EMG"]
        BMI["BMI088 芯片<br/>姿态角度 / 跌倒检测"]
        GPSM["GPS 定位模块<br/>经纬度"]
    end

    %% ========== 第 2 层：STM32 采集板 ==========
    subgraph STM["🎛️ STM32F407 采集板 · 信号的翻译官"]
        RT["RT-Thread 实时操作系统<br/>（管理各个采集任务）"]
        T1["心电/肌电采集线程<br/>读取传感器数据"]
        T2["姿态线程<br/>跌倒判断 / 角度计算"]
        FILT["数字滤波器<br/>去除噪声"]
        RT --- T1
        RT --- T2
    end

    %% ========== 第 3 层：ESP32 网关 ==========
    subgraph ESP["📶 ESP32-S3 网关 · 信号的快递员"]
        UART["串口接收<br/>STM32 发来的数据"]
        WSRV["WiFi WebSocket 服务<br/>把数据广播出去"]
        LCD["板上小屏幕<br/>本地实时波形"]
        ASR["语音模块 ASRPRO<br/>语音问答"]
        GPSP["GPS 数据解析"]
    end

    %% ========== 第 4 层：查看端 ==========
    subgraph PC["💻 电脑上的 BioScope 监控台"]
        WEB["网页界面<br/>实时波形 · 异常报警"]
        CSV["CSV 数据文件<br/>采集记录 / 历史回放"]
        AI["豆包 AI<br/>生成健康观察"]
    end

    subgraph PHONE["📱 Android 手机 App"]
        APP["ESP32Monitor<br/>随身查看数据"]
    end

    %% ========== 数据流向 ==========
    ADS -- "心电/肌电电信号" --> T1
    BMI -- "加速度/角速度" --> T2
    T1 -- "原始信号" --> FILT
    T2 -- "姿态结果" --> FILT
    FILT -- "整理好的数据 · 串口 UART" --> UART
    GPSM -- "定位数据" --> GPSP

    UART --> LCD
    UART --> WSRV
    GPSP --> WSRV
    ASR <-- "语音指令 / 语音播报" --> WSRV

    WSRV -- "WiFi 无线" --> WEB
    WEB -- "保存" --> CSV
    CSV -- "统计分析" --> AI
    WSRV -- "WiFi 无线" --> APP
```

### 图例

| 图形 | 含义 |
| --- | --- |
| 🧬 传感器 | 贴在身上的设备，负责感知信号 |
| 🎛️ 采集板 | 把传感器信号翻译成数据（本项目核心） |
| 📶 网关 | 负责把数据无线发送出去 |
| 💻 / 📱 | 查看数据的终端 |
| 实线箭头 | 数据从一个模块流向另一个模块 |

---

## 二、仓库目录地图（代码放在哪里）

```mermaid
flowchart LR
    ROOT["📂 项目根目录"]

    ROOT --> STM32["STM32/<br/>🎛️ 采集板固件（当前开发主线）"]
    STM32 --> M1["Mycode/<br/>❤️ 自己写的代码<br/>采集线程·滤波·IMU"]
    STM32 --> M2["Core/ Drivers/<br/>📦 芯片官方代码<br/>（CubeMX 生成）"]
    STM32 --> M3["MDK-ARM/<br/>🔨 Keil 工程文件"]

    ROOT --> ESP32["ESP32/<br/>📶 网关固件"]
    ESP32 --> E1["main/<br/>❤️ 主程序 + 引脚定义"]

    ROOT --> WEBM["web-monitor/<br/>💻 电脑监控网站"]
    WEBM --> W1["server.py<br/>本地服务器 + CSV 存储"]
    WEBM --> W2["index.html / app.js<br/>网页界面"]
    WEBM --> W3["data/<br/>📁 采集的 CSV 记录"]
    WEBM --> W4["PROJECT_SPEC.md<br/>📋 需求与协议文档"]

    ROOT --> AND["AndroidStudioProjects/<br/>📱 手机监控 App"]

    ROOT --> VOICE["voice/<br/>🗣️ 语音模块工程"]

    ROOT --> OLD["0509_GOOD/<br/>🗄️ 旧版固件备份<br/>（假肢电机控制）"]

    ROOT --> DOC["doc/<br/>📚 项目文档与报告"]
```

---

## 三、每个模块一句话说明

| 模块 | 扮演的角色 | 一句话说明 |
| --- | --- | --- |
| `STM32/` | 信号的翻译官 | F407 采集板，用 RT-Thread 系统实时读取心电、肌电、姿态传感器数据，滤波后发给 ESP32 |
| `ESP32/` | 信号的快递员 | S3 网关，接收 STM32 数据，通过 WiFi 无线发给电脑和手机；同时驱动小屏幕、GPS 和语音模块 |
| `web-monitor/` | 电脑监控台 | 本地网站，实时显示波形、触发报警、把数据存成 CSV、回放历史、生成 AI 健康观察 |
| `AndroidStudioProjects/` | 随身监视器 | 手机 App，连上 ESP32 也能看数据 |
| `voice/` | 语音助手 | ASRPRO 语音模块固件，支持语音问答 |
| `0509_GOOD/` | 旧版存档 | 早期"正常步态"固件备份（含假肢电机 CAN 控制），现已归档 |
| `doc/` | 资料库 | 竞赛报告、设计文档、电路原理图 |

---

## 四、一次完整的数据旅程

1. 🧬 身上的传感器（心电/肌电/姿态）持续产生信号
2. 🎛️ STM32 采集板上的 RT-Thread 系统分线程读取这些信号，滤波去噪
3. 📶 数据通过串口送到 ESP32-S3 网关
4. 📶 ESP32 把数据打包，通过 WiFi 无线广播（网页端/手机端都能连）
5. 💻 电脑网页实时画出波形，超阈值就报警，同时存成 CSV 文件；历史记录可回放，还可让 AI 生成观察
6. 📱 手机上也能实时查看；板上小屏幕和语音模块则提供本地交互
