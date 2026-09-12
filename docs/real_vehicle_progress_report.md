# 实车差速小车阶段报告

日期：2026-09-12  
Git 分支：`feat/real-robot-wifi-slam`

## 1. 项目目标

将两轮差速小车的激光雷达、MPU-6050 和轮端霍尔编码器接入 ROS 2。
电脑负责 SLAM 建图、Nav2 导航和运动控制；ESP32 负责传感器采集、
电机 PWM 和 Wi-Fi 通信。

## 2. 当前可用架构

```text
激光雷达 ─原装USB转换器→ Linux电脑 ─→ /scan

霍尔编码器 ─┐
MPU-6050      ─┼→ ESP32 ─Wi-Fi/UDP→ Linux电脑 ─┼→ /odom + odom→base_footprint TF
L298N+电机    ─┘                                  └→ /imu

/scan + odom TF ─→ slam_toolbox ─→ /map + map→odom TF
/cmd_vel ─→ WiFi桥接/PID ─→ ESP32 PWM ─→ L298N ─→ 左右轮
```

`slam_toolbox` 直接使用的是 `/scan` 和里程计 TF，不直接订阅 `/imu`。
当前 `/imu` 已发布，后续可用 `robot_localization` 与轮式里程计融合。

## 3. 已完成工作

### 3.1 工程分层

已按职责拆分为：

- `vehicle_description`：URDF/Xacro 和固定 TF。
- `vehicle_simulation`：Gazebo 仿真。
- `vehicle_hardware`：ESP32、电机、编码器和 IMU。
- `lidar_pkg`：雷达串口/TCP 输入与 `/scan`。
- `vehicle_mapping`：`slam_toolbox` 参数、地图和 RViz 配置。
- `vehicle_navigation`：Nav2、AMCL 和地图定位。
- `vehicle_control`：手动控制和监视工具。
- `vehicle_bringup`：组合上述包的场景启动文件。

### 3.2 ESP32 固件

- 建立 `car-esp32` Wi-Fi AP，地址 `192.168.4.1`。
- UDP `8888` 接收电机 PWM，回传编码器和 IMU。
- TCP `8889` 保留雷达原始字节透传。
- 加入命令超时停车保护。
- 编码器改为 A 相上升沿计数，避免 `CHANGE` 两个边沿相互抵消。
- 编码器报文加入启动 ID、序号和 ESP32 采样时间，用于识别重启、
  丢包和 Wi-Fi 成批到达。
- GY-521/MPU-6050 通过 I2C 读取，上报 m/s² 和 rad/s。

### 3.3 电脑端硬件桥接

- `/cmd_vel` 转换为左右轮目标角速度和 PWM。
- 根据 ESP32 采样时间计算轮速，进行 PID 调节。
- 处理编码器计数器溢出、重复/乱序数据和 ESP32 重启。
- 发布 `/odom`、`odom → base_footprint` TF 和 `/imu`。
- 加入 10 Hz 心跳，重新连网后可恢复遥测，断线时停车。

### 3.4 车体和编码器参数

- 轮子直径：`0.04 m`（半径 `0.02 m`）。
- 左右轮中心距：`0.13 m`。
- 轮端每圈计数：`360`个 A 相上升沿。
- 实测左轮约 `357.8`、右轮约 `359.0` 次/圈。
- 右轮软件方向系数为 `-1`，使小车前进时左右里程同为正。

### 3.5 激光雷达驱动

- 支持两种输入：ESP32 TCP 透传和 Linux 本地串口。
- 按实际协议解析：

  ```text
  AA 55 | CT | LSN | FSA(2) | LSA(2) | CS(2) |
  [intensity(1) | distance(2)] * LSN
  ```

- 加入数据包长度检查和校验码校验。
- 修正距离字节偏移和 `LaserScan.angle_max`。
- 原装 USB 转换器模式下连续发送3次 `A5 60`，提高启动可靠性。
- 新增 `lidar_serial_probe.py`，可单独显示原始 HEX 并统计合法包。

### 3.6 建图启动

`real_mapping.launch.py` 现可通过参数选择雷达来源：

- `lidar_transport:=tcp`：雷达经 ESP32。
- `lidar_transport:=serial`：原装 USB 转换器直连 Linux。

新增 `vehicle_mapping/config/real_mapping.rviz`，默认显示 `/map`、`/scan`和 TF。

## 4. 已完成的实车测试

| 项目 | 结果 |
|---|---|
| 左右电机开环 PWM | 两边均可转动 |
| 左右编码器 | 均有方向和计数反馈 |
| MPU-6050 | I2C `0x68` 识别成功，加速度/角速度有数据 |
| 雷达原装 USB 转换器 | `150000 baud, 8N1` 正常 |
| USB 雷达6秒原始测试 | `47187` 字节，`1421/1421` 个包校验通过 |
| `/scan` | 约 `4.84 Hz` |
| `/odom` | 约 `18.5 Hz` |
| `/imu` | 约 `46 Hz` |
| `odom → lidar_link` TF | 可连通 |
| `slam_toolbox` | 已生成 `/map`；静止测试地图为 `43×49` 栅格、`0.05 m` 分辨率 |

上述地图测试只证明数据链和 SLAM 已跑通，不代表地图几何精度已验收。

代码提交前已完成以下检查：

- ROS 2 的 8 个包全部构建成功。
- ROS 2 测试共 13 项，0 错误、0 失败、1 项按环境跳过。
- ESP32 PlatformIO 固件编译成功。
- Python 语法、C++ 格式和 Git 空白检查通过。

## 5. 尚未解决/需要 Linux 复测的问题

### 5.1 雷达裸 TTL 直连 ESP32 尚未成功

预期接线是雷达 `TX → ESP32 GPIO16(RX2)`、
`RX ← GPIO17(TX2)`、公共 GND、VCC 5V。

实测 ESP32 可收到约 `52 KB/6 s`，但内容类似 `6B 93 77 ...`，
`AA 55` 包头数为0。已尝试 D16/D35、极性反相、上拉、TX/RX 核对和
`100000–250000` 波特率扫描，仍无合法包。

当前只能确认问题在“雷达 TTL 电气信号/ESP32 UART 解码”附近，
还不能断言是电平、波形还是 ESP32 UART 配置。原生 Linux 实验阶段先用
已验证的原装 USB 转换器。

### 5.2 参数和融合待完成

- 在原生 Linux 中核对 RViz 的雷达方向、地图墙体和真实环境。
- WSLg 的 RViz Map 插件出现 GLSL 渲染错误，不能用于地图质量验收。
- WSL 测试中 `slam_toolbox` 偶尔报扫描/TF 队列丢弃，需在原生 Linux 确认是否消失。
- 标定雷达相对 `base_link` 的安装位置和方向。
- 通过实际直行和原地旋转进一步标定轮径、轮距和 PID。
- 如需要 IMU 改善位姿，增加 `robot_localization` 并填写正确的 IMU 协方差。

## 6. 原生 Linux 实验步骤

### 6.1 拉取和编译

```bash
git fetch origin
git switch feat/real-robot-wifi-slam
git pull --ff-only

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 6.2 单独验证 USB 雷达

确认雷达原装 USB 转换器的设备名：

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
python3 src/lidar_pkg/scripts/lidar_serial_probe.py --port /dev/ttyACM0
```

预期结论为看到 `AA 55` 且有校验正确数据包。

### 6.3 启动实车建图

1. 给 ESP32 和小车上电。
2. Linux 电脑连接 Wi-Fi `car-esp32`。
3. 雷达经原装 USB 转换器接 Linux 电脑。
4. 启动：

```bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 launch vehicle_bringup real_mapping.launch.py \
  lidar_transport:=serial \
  lidar_port_name:=/dev/ttyACM0
```

### 6.4 分项监视

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /imu
ros2 run tf2_ros tf2_echo odom lidar_link
```

原生 Linux 中启动 RViz：

```bash
rviz2 -d "$(ros2 pkg prefix vehicle_mapping)/share/vehicle_mapping/config/real_mapping.rviz"
```

先手推或低速控制小车，检查红色激光点、墙体和里程计方向是否一致。
确认无误后再扩大地图范围。

### 6.5 保存地图

```bash
ros2 run nav2_map_server map_saver_cli \
  -f src/vehicle_mapping/maps/real_map
```

## 7. 当前主要接线

```text
MPU-6050: SDA→GPIO21, SCL→GPIO22, VCC→3V3, GND→GND
左编码器: A→GPIO13, B→GPIO23
右编码器: A→GPIO18, B→GPIO19
L298N: ENA→26, IN1→27, IN2→14, IN3→32, IN4→33, ENB→25
```

完整供电、电平转换和电机线色说明见 `docs/hardware_wiring.md`。

## 8. 下一阶段验收标准

- RViz 中激光点与真实墙体方向一致。
- 手推直行时 `/odom` 直行，原地右转时 yaw 方向正确。
- 小车慢速绕房间一圈后，地图墙体不重影、不明显拉伸。
- 回到起点时回环能够闭合。
- 保存地图后可由 Nav2 `map_server` 重新加载。
