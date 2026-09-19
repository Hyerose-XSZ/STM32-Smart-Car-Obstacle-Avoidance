# 面向极低算力边缘终端的 TinyML 辅助驾驶系统设计与实车验证
### TinyML-Assisted Autonomous Driving System for Ultra-Low Compute Edge Microcontrollers

[![Platform](https://img.shields.io/badge/MCU-STM32F103%20(Cortex--M3%20%4072MHz)-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)
[![Architecture](https://img.shields.io/badge/Driver-Pure%20Register%20Level%20(No%20HAL)-orange.svg)](https://github.com/Hyerose-XSZ/STM32-Smart-Car-Obstacle-Avoidance)
[![TinyML](https://img.shields.io/badge/TinyML-39%20Lines%20Pure%20C%20Bitwise-green.svg)](https://github.com/Hyerose-XSZ/STM32-Smart-Car-Obstacle-Avoidance)
[![Verification](https://img.shields.io/badge/Unicorn%20ARM-2272%2F2272%20(100%25%20Match)-brightgreen.svg)](https://github.com/Hyerose-XSZ/STM32-Smart-Car-Obstacle-Avoidance)
[![Competition](https://img.shields.io/badge/2026%E5%A4%A9%E6%B4%A5%E5%B8%82%E6%96%B0%E5%B7%A5%E7%A7%91%E5%A4%A7%E8%B5%9B-%E7%94%B5%E5%AD%90%E4%BF%A1%E6%81%AF%E4%B8%8E%E4%BA%BA%E5%B7%A5%E6%99%BA%E8%83%BD%E7%BB%84-red.svg)](http://eeecxds.mh.chaoxing.com)

---

## 📌 项目概述

本项目为 **2026年天津市大学生新工科实践创新大赛（电子信息与人工智能组）** 参赛项目。

针对工业物联网、微型仓储 AGV 及低成本微型移动机器人在极低算力端侧微控制器上难以部署高开销复杂智能算法的工程痛点，本项目以 **STM32F103 (ARM Cortex-M3 @ 72MHz, 20KB SRAM, 64KB Flash)** 为核心主控，采用 **纯底层寄存器级无阻塞驱动**，构建了 **“确定性规则控制底座 + 纯 C 位运算端侧 TinyML 决策树时序辅助 + ML-Shadow 双轨影子容错门控”** 的软硬件软实时一体化闭环控制系统。

### 🌟 核心工程指标突破
* **极轻量端侧 TinyML**：仅 **39 行纯 C 语言无浮点位运算** 代码，零动态内存分配，占用 **Flash 仅 ~200 字节，SRAM 仅 5 字节**，单次推理耗时 **< 2 微秒**。
* **跨平台指令级硬件等价验证**：基于 Unicorn ARM Cortex-M3 指令级模拟器，在 **2272 组连续实测时序输入** 下，端侧纯 C 位运算输出与 Python 高维算法仿真达成 **100% 绝对一致**。
* **超低成本与绿色低功耗**：整机物料成本 **< 100 元**，运行功耗 **< 0.5W**，实车综合闭环路测通过率达 **98.2%**。
* **严密工程证据链**：采集并公开 **13 次真实实车脱机路测数据集**，经历 21 个高质量片段人工严格标定与镜像数据增强，软硬件全链条可闭环复现。

---

## 👥 团队信息与分工

* **申报单位**：天津大学 微电子学院（卫津路校区）
* **指导教师**：**武依**（天津大学微电子学院 英才副教授 / 博士生导师，研究方向：高频集成电路与微系统集成技术）
* **参赛成员**：
  * **肖索真（负责人 / 组长）**：系统总体架构设计、纯寄存器级驱动开发、自适应 PD 控制律、6步状态机避障、实车软硬件系统级集成联调。
  * **刘泽昊**：实车遥测日志清洗管道、5帧时序 TinyML 决策树离线训练与剪枝、纯 C 位运算自动生成器、Unicorn ARM 指令级模拟验证。
  * **许清源**：微型底盘机械空间 3D 装配与重心调谐、L298N 动力与主控双电源隔离供电拓扑、高频钽电容滤波去耦网络、电气故障排查。
  * **黄光昱**：全要素综合实验跑道设计与标定铺设、13轮闭环路测数据采集与切片初筛、实车影像采集、技术报告与答辩路演材料编制。

---

## 🏗️ 系统技术架构

### 1. 软硬件系统框图

```text
┌────────────────────────────────────────────────────────────────────────┐
│                        STM32F103 主控平台 (72MHz)                      │
│                                                                        │
│  ┌─────────────────┐      ┌──────────────────┐    ┌─────────────────┐ │
│  │ 4路红外光电传感器 │───▶ │ 16态误差连续映射   │───▶│ 自适应分层PD律   │ │
│  │ (PC0~PC3, 2ms)  │      │ Tracking_GetError│    │ KP=42.0 KD=75.0 │ │
│  └─────────────────┘      └──────────────────┘    └────────┬────────┘ │
│                                                            │          │
│  ┌─────────────────┐      ┌──────────────────┐    ┌────────▼────────┐ │
│  │ HC-SR04超声波模块│───▶ │ 10周期分频测距   │───▶│ L298N 双H桥电机 │ │
│  │ (PB10/PB11)     │      │ TIM4 1μs计时     │    │ TIM3 1kHz PWM   │ │
│  └─────────────────┘      └──────────────────┘    └─────────────────┘ │
│                                                            ▲          │
│  ┌─────────────────┐      ┌──────────────────┐             │          │
│  │ 5帧滑动窗口特征  │───▶ │ 39行纯C位运算     │             │          │
│  │ (20维时序二值向量│      │ CarML_Predict()  │             │          │
│  └─────────────────┘      └────────┬─────────┘             │          │
│                                    ▼                       │          │
│                           ┌──────────────────┐             │          │
│                           │ ML-Shadow 双轨   │─────────────┘          │
│                           │ 门控仲裁状态机   │ (一致信任/异常回退)      │
│                           └──────────────────┘                        │
└────────────────────────────────────────────────────────────────────────┘
```

### 2. 双轨安全容错机制（ML-Shadow 架构）
为了杜绝机器学习黑盒模型在边缘单片机上偶发“跑飞”失控，系统提出双轨门控机制：
* **第一轨（确定性规则底座）**：纯寄存器级编写的断崖降速 PD 控制律与避障状态机，保障 100% 独立脱困能力与确定性安全底线；
* **第二轨（端侧 TinyML 辅助）**：39 行决策树对连续 5 帧时序数据进行滑动推演，提供前瞻性同向微调；
* **门控仲裁（Gate Arbiter）**：设计 11 种工作状态码（`READY`, `DISAGREE`, `STALE`, `CHANGED`, `LOST` 等），当 ML 输出与物理规则一致时生效微调，一旦产生分歧瞬间无缝回退至物理规则底座，兼顾自适应能力与绝对安全性。

---

## 💻 硬件引脚分配与电气拓扑

### 1. L298N 电机驱动与动力接口
| 接口引脚 | STM32 物理引脚 | 寄存器配置模式 | 功能说明 |
|:---|:---|:---|:---|
| **ENA** | `PA6` | 复用推挽 (`TIM3_CH1`, 0xB) | 左轮硬件 PWM 速度调节 (1kHz, ARR=999) |
| **ENB** | `PA7` | 复用推挽 (`TIM3_CH2`, 0xB) | 右轮硬件 PWM 速度调节 (1kHz, ARR=999) |
| **IN1** | `PA0` | 通用推挽 (`0x3`, 50MHz) | 左电机正反转方向逻辑 1 |
| **IN2** | `PA1` | 通用推挽 (`0x3`, 50MHz) | 左电机正反转方向逻辑 2 |
| **IN3** | `PA4` | 通用推挽 (`0x3`, 50MHz) | 右电机正反转方向逻辑 1 |
| **IN4** | `PA5` | 通用推挽 (`0x3`, 50MHz) | 右电机正反转方向逻辑 2 |
| **GND** | `GND` | 系统单点星型共地 | 消除电机反峰电流对逻辑地干扰 |

### 2. 传感器与外设接口
| 传感器模块 | STM32 物理引脚 | 模式配置 | 工作机制 |
|:---|:---|:---|:---|
| **红外 IN4 (最左 L2)** | `PC0` | 上拉输入 (`CNF=10, MODE=00`) | 1: 检测到黑线 (灯灭)；0: 白地 (灯亮) |
| **红外 IN3 (中左 L1)** | `PC1` | 上拉输入 (`CNF=10, MODE=00`) | 1: 检测到黑线 (灯灭)；0: 白地 (灯亮) |
| **红外 IN2 (中右 R1)** | `PC2` | 上拉输入 (`CNF=10, MODE=00`) | 1: 检测到黑线 (灯灭)；0: 白地 (灯亮) |
| **红外 IN1 (最右 R2)** | `PC3` | 上拉输入 (`CNF=10, MODE=00`) | 1: 检测到黑线 (灯灭)；0: 白地 (灯亮) |
| **超声波 Trig** | `PB10` | 通用推挽输出 (50MHz) | 软件触发 20μs 高电平脉冲启动声波发射 |
| **超声波 Echo** | `PB11` | 浮空/下拉输入 | 接收回波高电平，TIM4 硬件 1μs 计数 |
| **状态指示 LED0** | `PE5` | 通用推挽输出 | 低电平点亮，启动自检与异常报警 |
| **板载蜂鸣器** | `PB8` | 通用推挽输出 | 启动初始化强制拉低，杜绝悬空杂音 |
| **微秒定时器 TIM4** | 内部计数器 | PSC=71 (1MHz), ARR=0xFFFF | 自由计数，提供微秒级确定性延时基准 |

---

## ⚡ 核心算法与参数标定

### 1. 动量抑制分层自适应 PD 控制律
针对高速直道（PWM 450）冲刺进入小半径急弯（直角弯）由于离心惯性甩尾冲出赛道的难题，采用阶梯力矩分配算法：

```c
// 1. 连续偏航误差映射与自适应降速
Error = f(Sensor_State);
Current_Base = max(SPEED_BASE - abs(Error) * SPEED_DROP_RATE, SPEED_CURVE_MIN);

// 2. PD 微调量运算
PID_Output = Kp * Error + Kd * (Error - Last_Error);
```

#### 四级分层控制工况响应策略：
* **工况 A：直道与微弯（`|Error| < 2.0`）**
  * `Left_Motor  = Current_Base + PID_Output`
  * `Right_Motor = Current_Base - PID_Output`
  * 纯正向差速平稳巡航，精准微调偏航。
* **工况 B：中度弯道（`2.0 <= |Error| < 4.0`）**
  * 偏右：`Left_Motor = Current_Base + 120`, `Right_Motor = -90`（内侧轮施加负向阻尼刹车）
  * 偏左：`Left_Motor = -90`, `Right_Motor = Current_Base + 120`
* **工况 C：极限急弯（`|Error| >= 4.0`）**
  * 启动向心反拉力矩：外侧轮给定 `380` 大扭矩推进，内侧轮施加 **`-180` 负向反转**，强制将车头拉向内侧，克服离心甩尾。
* **工况 D：全白脱轨（`0b0000`）**
  * 读取脱线瞬间锁存的 `memory_dir` 历史偏航记忆，启动原地高速差速自转搜线自救。

### 2. 复合闭环避障状态机
超声波动态测距阈值设为 `12.0 cm`，采用七阶段防剐蹭与平滑重捕时序：
1. **制动停顿（150ms）**：吸收直线前冲动量；
2. **强制倒车缓冲（150ms, -450/-450）**：主动拉开与障碍物纵向物理裕度；
3. **大差速右转借道（320ms, 460/80）**：形成充裕横向外展净距（>35cm）；
4. **侧向平稳超越（460ms, 300/300）**：安全越过障碍物侧方投影面；
5. **小锐角回切（430ms, 0/460）**：以 25° ~ 35° 平缓小夹角靠近黑线，避免直冲脱轨；
6. **动态二次滤波搜线**：连续 2 次采样命中黑线确认，彻底消除虚假光电抖动；
7. **反向打舵冲平（180ms, 260/80）**：顺平入轨，平滑切换回闭环 PD 循迹。

---

## 🤖 TinyML 训练与验证管道

本项目 TinyML 算法工程完整保存在 `car_ml/` 目录中：

```text
car_ml/
├── data/                               # 13次实车真实路测原始与清洗数据
│   ├── run_001.txt ~ run_013.txt       # 串口遥测原始日志 (含CRC与时间戳)
│   ├── run_001_clean.csv ~ ...         # 格式化结构时序数据
│   └── segment_labels.csv              # 21个经人工审定的有效路测切片
├── outputs/                            # 训练模型与导出交付物
│   ├── five_frame_candidate.joblib     # 训练好的5帧决策树模型
│   ├── car_ml_candidate.c              # 自动导出的39行纯C位运算代码
│   └── recovery_experiment_results.csv # 丢线恢复能力评测表
├── preview_mirror.py                   # 1帧 vs 5帧时序模型对比训练主脚本
├── export_candidate.py                 # sklearn 决策树转纯 C 位运算导出器
├── test_ml_shadow_arm.py               # Unicorn ARM Cortex-M3 指令级仿真验证
├── check_robustness.py                 # 传感器单路翻转注入抗噪鲁棒性测试
└── train_noise_experiment.py           # 噪声数据增强对比实验
```

### 39行纯 C 决策树核心实现（节选）
```c
// car_ml_candidate.c - 纯位运算、无浮点、无外部库依赖
#include <stdint.h>

int CarML_Predict(const uint8_t s[5]) {
    uint32_t f = ((uint32_t)s[0] << 16) | ((uint32_t)s[1] << 12) |
                 ((uint32_t)s[2] << 8)  | ((uint32_t)s[3] << 4)  | (uint32_t)s[4];
    if (!(f & (1 << 0))) {
        if (!(f & (1 << 1))) {
            if (!(f & (1 << 2))) return 0; // Class 0: 大左偏
            return 1;                      // Class 1: 轻左偏
        }
        return 4;                          // Class 4: 直行居中
    } else {
        if (!(f & (1 << 2))) return 3;     // Class 3: 轻右偏
        return 2;                          // Class 2: 大右偏
    }
}
```

---

## 📁 仓库代码结构

```text
.
├── README.md                           # 本项目全局技术与申报文档
├── 源码/                               # 生产级全闭环固件工程 (Keil MDK 完整工程)
│   ├── main.c                          # 纯寄存器底层驱动、PD控制与避障状态机主逻辑
│   ├── SmartCar.uvprojx                # Keil uVision5 工程配置文件
│   └── ML/                             # 端侧 TinyML 集成模块
│       ├── car_ml_candidate.c          # 39行纯C端侧决策树
│       ├── ml_shadow.c                 # ML-Shadow 双轨门控仲裁状态机实现
│       └── ml_shadow.h                 # 11种门控状态码与API定义
├── car_ml/                             # TinyML 模型离线训练、导出与仿真管道 (Python)
└── Stages/                             # 阶梯式递进开发与单项验证历史源码
    ├── 1. 基础驱动验证/                 # 四轮前进、PWM 差速转向测试
    ├── 2. 传感器独立诊断/               # 4路红外状态机、超声波测距静态校验
    ├── 3. 功能集成与单项验证/           # 动态循迹停车、独立绕障姿态标定
    └── 4. 生产级全闭环系统/             # 循迹+避障 1.0/2.0 正式交付版
```

---

## 🚀 快速复现与上手指南

### 1. 嵌入式固件编译与烧录（STM32）
* **开发工具**：Keil MDK 5 (ARM Compiler 6 / armclang)
* **芯片支持包**：`Keil.STM32F1xx_DFP.2.4.1.pack`
* **编译流程**：
  1. 打开 `源码/SmartCar.uvprojx`；
  2. 在 Project Options 中确认配置：Target: STM32F103ZE, Compiler: ARM Compiler 6, Optimization: `-O2`；
  3. 点击 **Build (F7)** 编译生成二进制固件；
  4. 连接 DAP-Link / J-Link，点击 **Download (F8)** 烧录到单片机。

### 2. TinyML 模型训练与 ARM 指令级仿真（Python）
* **依赖环境**：Python 3.10+
* **安装依赖**：
  ```bash
  pip install numpy pandas scikit-learn joblib matplotlib unicorn
  ```
* **一键运行对比训练**：
  ```bash
  cd car_ml
  python preview_mirror.py
  ```
* **执行 2272 组 ARM 指令级一致性验证**：
  ```bash
  python test_ml_shadow_arm.py
  ```
  终端将逐样本比对 Python 端与 Unicorn 模拟器执行 ARM 指令的输出，验证一致率达到 `100.00%`。

---

## 📚 参考文献与学术支撑

1. **[1] 期刊 [J]**：韩强, 杨晓华, 于如兴. 基于STM32单片机的智能小车设计[J]. 汽车实用技术, 2026(02): 21-26.
2. **[2] 期刊 [J]**：丘斯远, 曾翔, 张鉴隆. 基于STM32的多功能自主巡航与避障智能小车控制系统设计[J]. 电脑知识与技术, 2026, 22(10): 26-29.
3. **[3] 官方手册 [R]**：STMicroelectronics. STM32F101xx...STM32F107xx advanced Arm-based 32-bit MCUs Reference Manual (RM0008)[R]. Rev 21, 2021.
4. **[4] 芯片手册 [R]**：STMicroelectronics. L298: Dual Full-Bridge Driver Datasheet[R]. DocID 1773 Rev 6, 2000.
5. **[5] 传感器手册 [R]**：ElecFreaks Technology. Ultrasonic Ranging Module HC-SR04 User Manual[R]. 2011.
6. **[6] 专著 [M]**：ÅSTRÖM K J, HÄGGLUND T. Advanced PID Control[M]. ISA, 2006.
7. **[7] 专著 [M]**：WARDEN P, SITUNAYAKE D. TinyML: Machine Learning with TensorFlow Lite on Arduino and Ultra-Low-Power Microcontrollers[M]. O'Reilly Media, 2019.

---

## 📄 开源许可与参赛声明

* 本项目源码遵循 **MIT License** 开源协议；
* 欢迎交流研讨与学术引用！
