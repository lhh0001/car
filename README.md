# Vehicle Simulation Workspace

ROS2 Humble + Gazebo Classic 11 

## 工程结构

```
vehicle_ws/
├── src/
│   ├── vehicle_description/    # 纯描述: URDF/XACRO 模型 + 网格资源
│   ├── vehicle_bringup/        # 启动 + 配置: launch files, worlds, YAML
│   └── vehicle_control/        # 控制脚本: 转向控制器, 遥控 GUI
├── .github/workflows/          # CI/CD (GitHub Actions)
├── Dockerfile                  # 容器化构建
├── docker-compose.yml          # 一键启动
└── .pre-commit-config.yaml     # 代码质量检查
```

**设计原则**: 描述 / 启动 / 控制 三层分离，每个包职责单一。

## 机器人模型

| 模型 | 包 | 类型 | 用途 |
|------|-----|------|------|
| `diff_drive_robot.xacro` | vehicle_description | 两轮差速 | SLAM + Nav2 自主导航 |
| `ferrari.urdf` | vehicle_description | 四轮 Ackermann | 仿真驾驶、转向控制 |

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

# Ferrari 阿克曼驾驶仿真
ros2 launch vehicle_bringup ferrari_sim.launch.py

# 带参数启动
ros2 launch vehicle_bringup ferrari_sim.launch.py world:=my_world.world x_pose:=5.0 use_slam:=false

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
| vehicle_control | `wheelbase` | 2.64 | 轴距 (m) |
| vehicle_control | `max_steer_angle` | 0.6 | 最大转向角 (rad) |
| vehicle_control | `deadband` | 0.02 | 速度死区 |
| vehicle_bringup | `world` | test_yard.world | Gazebo 世界文件 |
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
# 1. 在 vehicle_description/urdf/ 添加 URDF/XACRO
# 2. 在 vehicle_bringup/launch/ 添加 launch 文件
# 3. 在 vehicle_bringup/config/ 添加对应 YAML 配置
```

## License

Apache-2.0
