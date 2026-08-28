# 差速小车下位机工作原理详解

> 本文档解释 ESP32 固件（下位机）与 PC（上位机）如何通过 USB 串口协作，
> 实现从键盘遥控到电机转动的完整链路。

---

## 目录

1. [系统总览](#1-系统总览)
2. [硬件层](#2-硬件层)
3. [ESP32 固件详解](#3-esp32-固件详解)
4. [串口通信协议](#4-串口通信协议)
5. [差速运动学](#5-差速运动学)
6. [ROS2 上位机（serial_bridge.py）](#6-ros2-上位机serial_bridgepy)
7. [端到端数据流](#7-端到端数据流)
8. [接线参考](#8-接线参考)

---

## 1. 系统总览

```
┌──────────────────────────────────────────────────────────────────────┐
│                           笔记本电脑 (Ubuntu)                          │
│                                                                       │
│  ┌──────────┐    /cmd_vel     ┌────────────────┐                     │
│  │ teleop_   │───(Twist msg)──→│ serial_bridge   │  ← ROS2 节点       │
│  │ gui.py    │                 │ /odom, /tf →    │                     │
│  └──────────┘                 └───────┬──────────┘                     │
│                                        │  USB 串口 (/dev/ttyUSB0)       │
│                                        │  115200 baud, 文本协议         │
│  ════════════════════════════════════════════════════════════════      │
│                                        │                               │
│  ┌────────────────────────────────────┐                                │
│  │          ESP32 开发板               │  ← 下位机                      │
│  │  ┌──────────────────────────────┐ │                                │
│  │  │         主循环  (loop)        │ │                                │
│  │  │  ┌──────────┐ ┌───────────┐  │ │                                │
│  │  │  │ 读串口    │ │ 发送编码器 │  │ │  50Hz                         │
│  │  │  │ "M 100.."│ │ "E 0 0"   │  │ │                                │
│  │  │  └────┬─────┘ └─────┬─────┘  │ │                                │
│  │  │       │              │        │ │                                │
│  │  │  ┌────▼──────────────▼─────┐  │ │                                │
│  │  │  │    PWM 生成 + 方向控制    │  │ │                                │
│  │  │  │    ledc + digitalWrite  │  │ │                                │
│  │  │  └────────┬───────────────┘  │ │                                │
│  │  └──────────┼──────────────────┘ │                                │
│  ├─────────────┼────────────────────┤                                │
│  │             │   PWM / DIR       │                                │
│  └─────────────┼────────────────────┘                                │
│                │                                                       │
│  ┌─────────────▼────────────────────┐                                │
│  │         L298N 电机驱动板           │                                │
│  │  ┌──────┐ ┌──────┐ ┌──────┐     │                                │
│  │  │ 逻辑  │ │ H桥A │ │ H桥B │     │  12V 电源                        │
│  │  │ 转换  │ │      │ │      │     │                                │
│  │  └──────┘ └──┬───┘ └──┬───┘     │                                │
│  └──────────────┼─────────┼─────────┘                                │
│                 │         │                                            │
│        ┌────────▼──┐  ┌──▼────────┐                                  │
│        │ 左 N20 电机 │  │ 右 N20 电机 │                                  │
│        │ + 编码器    │  │ + 编码器    │                                  │
│        └────────────┘  └────────────┘                                  │
└──────────────────────────────────────────────────────────────────────┘
```

**核心思路**：

- **上位机 (PC)** 跑 ROS2，负责高层逻辑：遥控、路径规划、SLAM
- **下位机 (ESP32)** 只做实时硬件驱动：读编码器、发 PWM、串口通信
- 上下位机通过 **USB 串口 + 文本协议** 通信，不依赖 WiFi/蓝牙（更可靠、低延迟）

---

## 2. 硬件层

### 2.1 硬件清单

| 组件 | 型号 | 用途 |
|------|------|------|
| 主控 | ESP32 DevKit V1 (CH340) | 运行固件，控制系统 |
| 驱动 | L298N 双路 H 桥 | 放大 PWM 信号驱动电机 |
| 电机 ×2 | N20 12GA 150RPM 减速电机 | 差速驱动左右轮 |
| 编码器 ×2 | 霍尔传感器 (电机自带) | 测量轮子转速/转角 |
| 电源 | 12V 锂电池 + 降压模块 | 分别供电机和 ESP32 |

### 2.2 ESP32 引脚分配

```
ESP32 DevKit V1 (30-pin)

          ┌────────────────────────┐
     EN  ─┤○ ○                    ├── GND
    VP   ─┤○ ○                    ├── GPIO23
    VN   ─┤○ ○ L: ENA=26          ├── GPIO22
   GPIO34─┤○ ○ L: IN1=27          ├── TX0(1)
   GPIO35─┤○ ○ L: IN2=14          ├── RX0(3)
   GPIO32─┤○ ○ R: IN3=32          ├── GPIO21
   GPIO33─┤○ ○ R: IN4=33          ├── GND
   GPIO25─┤○ ○ R: ENB=25          ├── GPIO19 ← R: ENC_B
   GPIO26─┤○ ○  ENA               ├── GPIO18 ← R: ENC_A
   GPIO27─┤○ ○  IN1               ├── GPIO5
   GPIO14─┤○ ○  IN2               ├── GPIO17
    GPIO12─┤○ ○                   ├── GPIO16
     GND  ─┤○ ○                   ├── GPIO4
    VIN   ─┤○ ○                   ├── GPIO0
    EN    ─┤○ ○                   ├── GPIO2
    3V3   ─┤○ ○                   ├── GPIO15
          └────────────────────────┘
    USB 口在这边（CH340 串口芯片）
```

### 2.3 L298N 工作原理

```
             ┌─────────────┐
             │   L298N     │
    ESP32 ──→│ IN1  IN2    │──→ 左电机 (ENA 控制速度)
    ESP32 ──→│ IN3  IN4    │──→ 右电机 (ENB 控制速度)
    12V ────→│ VCC         │
    GND ────→│ GND         │
             └─────────────┘

H 桥原理（以左电机为例）：

   IN1=HIGH, IN2=LOW  →  电流 A→B  →  电机正转（前进）
   IN1=LOW,  IN2=HIGH →  电流 B→A  →  电机反转（后退）
   IN1=LOW,  IN2=LOW  →  断路        →  电机自由停转（滑行）
   IN1=HIGH, IN2=HIGH →  短路        →  刹车（能耗制动）

   ENA 输入 PWM  →  调节电机两端等效电压  →  控制转速
```

### 2.4 编码器原理

```
N20 电机尾部有一个霍尔传感器编码器：

  磁铁盘（随电机轴旋转）
    │
    ├── 7 对 N/S 磁极
    │
    ▼
  霍尔传感器 A ──→ 脉冲信号 A (方波)
  霍尔传感器 B ──→ 脉冲信号 B (方波, A 的相位差 90°)

电机转一圈（电机端） → 7 个脉冲 × 减速比 100 → 输出轴 700 个脉冲

所以 ENCODER_CPR = 700（轮子每转一圈，编码器产生 700 个脉冲）

通过 A/B 相位判断方向：
  A 先于 B 上升 → 正转 → enc_left++
  B 先于 A 上升 → 反转 → enc_left--
```

---

## 3. ESP32 固件详解

固件文件：`src/main.ino`

### 3.1 整体架构

```
┌─────────────────────────────────────────┐
│  setup() — 只执行一次                     │
│  ├─ Serial.begin(115200)     配置串口     │
│  ├─ pinMode(OUTPUT)          配置 GPIO    │
│  ├─ ledcSetup + ledcAttach   配置 PWM     │
│  ├─ attachInterrupt          配置编码器中  │
│  └─ stop()                   初始停车      │
└─────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────┐
│  loop() — 无限循环                        │
│  ┌─────────────────────────────────────┐ │
│  │  1) 检查串口是否有数据                 │ │
│  │     Serial.available()?              │ │
│  │     ├─ 有 → 读取 "M xxx xxx"        │ │
│  │     │       解析左右 PWM             │ │
│  │     │       set_motor(L, pwm)       │ │
│  │     │       set_motor(R, pwm)       │ │
│  │     └─ 无 → 跳过                    │ │
│  ├─────────────────────────────────────┤ │
│  │  2) 定时发送编码器（50Hz）             │ │
│  │     millis() - last >= 50ms?        │ │
│  │     └─ 是 → "E left_enc right_enc" │ │
│  └─────────────────────────────────────┘ │
└─────────────────────────────────────────┘
              │
    ┌─────────┴──────────┐
    ▼                    ▼
 中断服务（后台）     编码器计数
 on_enc_left()       enc_left volatile
 on_enc_right()      enc_right volatile
```

### 3.2 关键代码解读

#### PWM 生成（ESP32 专用 ledc）

```cpp
ledcSetup(0, PWM_FREQ, PWM_RES);  // 通道0: 5000Hz, 8位精度(0-255)
ledcAttachPin(ENA, 0);             // 通道0 输出到 GPIO26
```

> 注：ESP32 不能用 `analogWrite()`，那是 AVR 的函数。必须用 `ledc`（LED PWM Controller）。

#### 编码器中断（IRAM_ATTR）

```cpp
void IRAM_ATTR on_enc_left() {
  if (digitalRead(ENC_L_B)) enc_left++; else enc_left--;
}
```

- `IRAM_ATTR`：将函数放在 **IRAM**（指令 RAM），而不是 Flash。中断服务函数必须放在 RAM 里，因为 Flash 可能正在被其他操作占用，导致延迟。
- `volatile long enc_left`：`volatile` 告诉编译器"这个变量会被中断随时修改，不要缓存到寄存器里"。
- `CHANGE` 触发：A 相每次边沿变化（上升+下降）都触发中断，配合 B 相电平判断方向。

#### 电机控制 set_motor()

```cpp
void set_motor(Motor side, int pwm) {
  pwm = constrain(pwm, -255, 255);     // 限幅

  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);   // 正转
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);  digitalWrite(in2, HIGH);  // 反转
  } else {
    digitalWrite(in1, LOW);  digitalWrite(in2, LOW);   // 刹车
  }
  ledcWrite(en, abs(pwm));    // PWM 占空比
}
```

**pwm 值含义**：

| pwm | IN1 | IN2 | 效果 |
|-----|-----|-----|------|
| +150 | HIGH | LOW | 正转，速度 150/255 = 59% |
| -100 | LOW | HIGH | 反转，速度 100/255 = 39% |
| 0 | LOW | LOW | 刹车（能耗制动） |

---

## 4. 串口通信协议

### 4.1 物理层

- **接口**：USB 转串口 (CH340)
- **波特率**：115200 bps
- **数据位**：8 bit
- **停止位**：1 bit
- **校验**：无
- **流控**：无

### 4.2 数据链路层

采用**文本协议，换行符 `\n` 分隔**。类似 GPS NMEA 0183 的简化版。

好处：
- 人可读，方便 debug（用串口助手直接看）
- 不需要打包/解包，不需要考虑字节序
- 缺点是带宽利用率低（不在乎，115200 完全够用）

### 4.3 帧格式

| 方向 | 格式 | 示例 | 频率 |
|------|------|------|------|
| PC → ESP32 | `M <左PWM> <右PWM>\n` | `M 100 -50\n` | 按需 (有新的 cmd_vel) |
| ESP32 → PC | `E <左编码器> <右编码器>\n` | `E 1234 -567\n` | 50 Hz |
| ESP32 → PC | `I <ax> <ay> <az> <gx> <gy> <gz>\n` | `I 0.01 0.02 9.81 ...\n` | 预留 (IMU) |

**协议状态机（ESP32 侧）**：

```
串口收到字符 → readStringUntil('\n')
                   │
                   ▼
            第一个字符 = ?
            ├─ 'M' → 解析左右 PWM → set_motor()
            ├─ 其他 → 忽略 (暂不支持上行指令)
            └─ 空行 → 跳过
```

**协议状态机（PC 侧）**：

```
串口收到一行 → 第一个字符 = ?
               ├─ 'E' → 解析编码器脉冲 → 计算里程计
               ├─ 'I' → 解析 IMU 数据（预留）
               └─ 其他 → 跳过
```

---

## 5. 差速运动学

### 5.1 逆运动学：cmd_vel → 轮速

```
给定机器人线速度 v (m/s) 和角速度 ω (rad/s)，
计算左右轮各自的角速度：

    轮距 L = 0.20m（左右轮间距）

    ω_left  = (2v - ω·L) / (2·R)      R = 轮子半径 0.04m
    ω_right = (2v + ω·L) / (2·R)

例：直行 v=0.3 m/s, ω=0
    ω_left = ω_right = (2×0.3) / (2×0.04) = 7.5 rad/s

例：原地左转 v=0, ω=1.57 rad/s (90°/s)
    ω_left  = (0 - 1.57×0.20) / 0.08 = -3.93 rad/s (反转)
    ω_right = (0 + 1.57×0.20) / 0.08 = +3.93 rad/s (正转)
```

### 5.2 正运动学：编码器 → 里程计

```
给定左右轮转动角度 Δθ_left, Δθ_right（从编码器增量计算）：

    Δ距离 = (Δθ_right + Δθ_left) / 2 × R     (轮子平均位移)
    Δ偏航  = (Δθ_right - Δθ_left) × R / L     (航向角变化)

    x  += Δ距离 × cos(偏航)
    y  += Δ距离 × sin(偏航)
    yaw += Δ偏航
```

### 5.3 轮速 → PWM 映射

```
N20 12GA 150RPM 空载转速：
    ω_max = 150 RPM = 150 × 2π/60 = 15.7 rad/s

PWM = (需求转速 / 15.7) × 255                          ← 线性近似
PWM = clamp(PWM, -255, 255)
```

> 这是**开环控制**，实际转速会受负载、电池电量影响。
> 工业级产品会用 **PID 闭环控制**：编码器反馈实际转速 → 计算误差 → 调整 PWM。

---

## 6. ROS2 上位机（serial_bridge.py）

### 6.1 节点接口

```
             ┌───────────────────┐
             │   serial_bridge   │
             │   (ROS2 Node)     │
             │                   │
  /cmd_vel ──→│ _on_cmd()       │──→ 串口 "M xxx xxx"
  (Twist)    │                   │
             │ _read_serial()   │←── 串口 "E xxx xxx"
             │ _on_encoder()    │
             │                   │
             ├──→ /odom         │  (Odometry msg)
             └──→ /tf           │  (TransformStamped)
            └───────────────────┘
```

### 6.2 数据流

```
/cmd_vel 收到消息 (v, ω)
    │
    ├── 差速解算 → v_left, v_right (rad/s)
    ├── 转速→PWM 映射 → pwm_left, pwm_right
    └── 串口发送 "M 100 -50\n"
            │
            ▼
        ESP32 接收 → set_motor()
            │
            ▼
        电机转动 + 编码器计数
            │
            ▼
        ESP32 发送 "E 1234 -567\n"
            │
            ▼
        _read_serial() 50Hz
            │
            ├── 解析编码器增量
            ├── 计算里程计 (x, y, yaw)
            ├── 发布 /odom
            └── 发布 /tf (odom → base_footprint)
```

### 6.3 可配置参数

```yaml
# config/motor_params.yaml
serial_bridge:
  ros__parameters:
    port: /dev/ttyUSB0          # 串口设备
    baud: 115200                # 波特率（须与 ESP32 一致）
    encoder_cpr: 700.0          # 编码器每转脉冲数
    wheel_radius: 0.04          # 轮子半径 (m)
    wheel_base: 0.20            # 左右轮距 (m)
```

### 6.4 启动命令

```bash
# 单步启动
ros2 run vehicle_hardware serial_bridge.py

# launch 文件启动（含 robot_state_publisher + teleop_gui）
ros2 launch vehicle_hardware diff_drive_real.launch.py

# 指定不同端口
ros2 launch vehicle_hardware diff_drive_real.launch.py port:=/dev/ttyUSB1
```

---

## 7. 端到端数据流

以"键盘遥控前进"为例，追踪一次完整的指令路径：

```
时刻 T0：用户按下 W 键
    │
    ▼  teleop_gui.py 发布
    │
    ▼  /cmd_vel: linear.x=0.3, angular.z=0.0
    │
    ▼  serial_bridge 收到 Twist 消息
    │   _on_cmd():
    │     差速解算: v_left=7.5 rad/s, v_right=7.5 rad/s
    │     PWM 映射:  pwm_left=122, pwm_right=122
    │     串口发送:  b"M 122 122\n"
    │
    ▼  USB 线传输 (~1ms)
    │
    ▼  ESP32 loop() 读到串口数据
    │   Serial.readStringUntil('\n') → "M 122 122"
    │   解析: left=122, right=122
    │   set_motor(LEFT,  122):
    │     IN1=HIGH, IN2=LOW, ledcWrite(0, 122)
    │   set_motor(RIGHT, 122):
    │     IN3=HIGH, IN4=LOW, ledcWrite(1, 122)
    │
    ▼  L298N 输出 PWM 到电机
    │
    ▼  电机转动 → 编码器产生脉冲
    │
    ▼  编码器中断 →
    │   on_enc_left:  enc_left  += 1  (每 175μs 左右)
    │   on_enc_right: enc_right += 1
    │
    ▼  loop() 50Hz 定时 →
    │   Serial.print("E 10234 10256\r\n")
    │
    ▼  USB 线传输 (~1ms)
    │
    ▼  serial_bridge _read_serial() 50Hz →
    │   _on_encoder("E 10234 10256"):
    │     Δleft=10, Δright=12
    │     里程计更新: x+=0.0005, yaw+=0.001
    │     发布 /odom, /tf

总延迟：约 2-3ms（串口传输） + 50ms（编码器上报周期） ≈ 50-60ms
```

---

## 8. 接线参考

### 8.1 ESP32 ↔ L298N

| L298N | 线色建议 | ESP32 引脚 | 说明 |
|-------|---------|-----------|------|
| ENA | 黄色 | GPIO 26 | 左电机速度 (PWM) |
| IN1 | 橙色 | GPIO 27 | 左电机方向 A |
| IN2 | 红色 | GPIO 14 | 左电机方向 B |
| IN3 | 棕色 | GPIO 32 | 右电机方向 A |
| IN4 | 紫色 | GPIO 33 | 右电机方向 B |
| ENB | 蓝色 | GPIO 25 | 右电机速度 (PWM) |
| GND | 黑色 | GND | 共地 |

### 8.2 ESP32 ↔ 编码器

| 编码器 | ESP32 引脚 | 说明 |
|--------|-----------|------|
| 左电机 A 相 | GPIO 34 | 输入 (仅支持输入) |
| 左电机 B 相 | GPIO 35 | 输入 (仅支持输入) |
| 右电机 A 相 | GPIO 18 | 输入, 中断 |
| 右电机 B 相 | GPIO 19 | 输入, 中断 |
| 编码器 VCC | 3.3V 或 5V | 供电 (看编码器规格) |
| 编码器 GND | GND | 共地 |

### 8.3 电源

```
12V 电池/适配器
    ├──→ L298N VCC (12V) — 电机供电
    └──→ 降压模块 (5V)
         └──→ ESP32 VIN 或 USB 口 — 控制电路供电

共地：L298N GND ↔ 降压模块 GND ↔ ESP32 GND
```

> **重要**：ESP32 的 GPIO34/35 是**纯输入引脚**，没有内部上拉！
> 编码器接这两个脚需要用 `INPUT_PULLUP`（固件已配置）。

---

## 附录 A：常用调试命令

```bash
# 查看串口是否识别
ls -la /dev/ttyUSB*

# 原始串口输出（波特率须一致）
stty -F /dev/ttyUSB0 115200 raw; cat /dev/ttyUSB0

# 发送测试指令
echo -e "M 100 100\r\n" > /dev/ttyUSB0

# ROS2 查看话题
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /cmd_vel

# 查看 TF 树
ros2 run tf2_tools view_frames
```

## 附录 B：性能指标

| 指标 | 值 |
|------|-----|
| 固件大小 | ~280KB Flash |
| 内存占用 | ~22KB RAM (6.7%) |
| 编码器上报频率 | 50 Hz |
| 串口延迟 | ~1-2ms |
| PWM 频率 | 5000 Hz |
| PWM 精度 | 8-bit (0-255) |
| 编码器精度 | 700 脉冲/轮圈 |
| 最小可测位移 | 2π×0.04/700 ≈ 0.36 mm |
