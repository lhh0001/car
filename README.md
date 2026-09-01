# Differential-Drive Robot Workspace

ROS2 Humble differential-drive robot: simulation, Wi-Fi hardware, mapping and navigation.

## 工程结构

```
vehicle_ws/
├── src/
│   ├── lidar_pkg/              # ESP32 TCP 雷达转发与 /scan 解析
│   ├── vehicle_description/    # 差速小车 URDF/XACRO
│   ├── vehicle_simulation/     # Gazebo world、插件与仿真基础启动
│   ├── vehicle_hardware/       # ESP32 Wi-Fi、IMU、编码器和电机接口
│   ├── vehicle_mapping/        # slam_toolbox 参数、建图启动与地图
│   ├── vehicle_navigation/     # AMCL、map_server、Nav2 参数与启动
│   ├── vehicle_control/        # 遥控与话题监控
│   └── vehicle_bringup/        # 只组合上述功能包的场景启动
├── .github/workflows/          # CI/CD (GitHub Actions)
├── Dockerfile                  # 容器化构建
├── docker-compose.yml          # 一键启动
└── .pre-commit-config.yaml     # 代码质量检查
```

**设计原则**: 每个包只负责一种能力；包之间只经 ROS topic、TF、service/action 协作。
详见 [架构说明](docs/architecture.md)。

## 机器人模型

| 模型 | 包 | 类型 | 用途 |
|------|-----|------|------|
| `diff_drive_robot.xacro` | vehicle_description | 两轮差速 | SLAM + Nav2 自主导航 |

## 快速开始

### 环境

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11

### 安装依赖

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-xacro ros-humble-teleop-twist-keyboard ros-humble-rviz2 \
  python3-tk
```

### 编译

```bash
cd ~/vehicle_ws
colcon build --symlink-install
source install/setup.bash
```

### 启动

```bash
# 差速小车 + SLAM 建图
ros2 launch vehicle_bringup diff_drive_sim.launch.py

# 差速小车 + SLAM + Nav2 自主导航
ros2 launch vehicle_bringup diff_drive_nav.launch.py

# 真车建图（电脑连接 ESP32 的 car-esp32 热点后）
ros2 launch vehicle_bringup real_mapping.launch.py

# 手动遥控（独立启动）
ros2 run vehicle_control teleop_gui.py --ros-args -p max_linear:=5.0 -p max_angular:=3.0
```

## 测试

```bash
# 运行测试
colcon test --packages-select vehicle_control
colcon test-result --verbose

# 运行 lint
pip install pre-commit && pre-commit run --all-files
```

## Docker

```bash
# 构建镜像
docker build -t vehicle-sim:humble .

# 启动（需 X11 转发）
xhost +local:docker
docker compose up

# 在容器内运行仿真
docker compose run --rm vehicle-sim bash -c "ros2 launch vehicle_bringup diff_drive_sim.launch.py"
```

## CI/CD

Push 到 main/PR 时自动触发：
- 构建三个包
- 运行单元测试
- 代码风格检查

详见 `.github/workflows/ci.yml`

## 参数说明

| 包 | 参数 | 默认值 | 说明 |
|-----|------|--------|------|
| vehicle_control | `max_linear` | 2.0 | 手动控制最大线速度 (m/s) |
| vehicle_control | `max_angular` | 8.0 | 手动控制最大角速度 (rad/s) |
| vehicle_simulation | `world` | test_yard.world | Gazebo 世界文件 |
| vehicle_bringup | `x_pose/y_pose/z_pose` | 0/0/0.05 | 初始位姿 |

## 架构

```
/scan (LiDAR) ──┐
                ├→ slam_toolbox ──→ /map + map→odom TF
/odom TF ───────┘
                     ↓
planner_server → /plan (全局路径)
                     ↓
controller_server (DWB) → /cmd_vel → diff_drive → 车轮
    ↓ 读取
local_costmap (障碍物代价地图)
```

## 开发指南

```bash
# 代码质量
pip install pre-commit && pre-commit install

# 添加新机器人模型
# 1. 在 vehicle_description/urdf/ 添加真实车共同模型
# 2. 仿真插件/世界放 vehicle_simulation/
# 3. SLAM 参数放 vehicle_mapping/config/；Nav2 参数放 vehicle_navigation/config/
# 4. 在 vehicle_bringup/launch/ 组合成用户命令
```

## License

Apache-2.0
