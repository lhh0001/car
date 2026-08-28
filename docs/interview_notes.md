# 面试速记 —— 差速小车导航项目

---

## 系统架构（自上而下）

```
┌─────────────────────────────────────────────────────────────┐
│  用户层                                                      │
│  teleop_gui.py ──► /cmd_vel          ← 手动遥控              │
│  RViz "2D Goal Pose" ──► /goal_pose  ← 设定导航目标          │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│  导航层 (Nav2)                                               │
│                                                              │
│  bt_navigator          ← 行为树，协调整体导航逻辑             │
│      │                                                       │
│      ├── planner_server  ← 全局路径规划 (NavfnPlanner / A*)  │
│      │      输入: global_costmap + 目标位置                   │
│      │      输出: /plan (路过点序列)                          │
│      │                                                       │
│      ├── controller_server ← 局部轨迹跟踪 (DWB)               │
│      │      输入: /plan + local_costmap + /odom              │
│      │      内部: 20×20=400条轨迹采样 → 7个critic评分 → 选最优 │
│      │      输出: /cmd_vel (Twist)                           │
│      │                                                       │
│      ├── behavior_server ← 异常恢复 (旋转/后退/等待)          │
│      └── velocity_smoother ← 速度平滑，防急刹急加速           │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│  代价地图层 (Costmap 2D)                                      │
│                                                              │
│  global_costmap (991×1004, 0.05m分辨率)                      │
│      ├── static_layer    ← 预建地图                          │
│      ├── obstacle_layer  ← /scan 标记障碍物                  │
│      └── inflation_layer ← 膨胀障碍物 (0.6m半径)             │
│                                                              │
│  local_costmap (4m×4m滚动窗口)                               │
│      ├── voxel_layer     ← /scan 标记障碍物                  │
│      └── inflation_layer ← 膨胀障碍物                        │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│  感知层                                                      │
│  Gazebo LiDAR ──► /scan (LaserScan, 10Hz)                   │
│  轮式里程计 ────► /odom (Odometry)                           │
└─────────────────────────────────────────────────────────────┘
                           │
┌─────────────────────────────────────────────────────────────┐
│  驱动层                                                      │
│  /cmd_vel ──► Gazebo diff_drive 插件 ──► /joint_states       │
│  (真车: /cmd_vel → serial_bridge.py → ESP32 → N20电机)      │
└─────────────────────────────────────────────────────────────┘
```

## 数据流（一帧激光到车轮转动）

```
/scan → costmap标记障碍 → planner算全局路径(/plan)
                        → DWB生成400条轨迹 → 7维评分 → 选最优
                        → /cmd_vel → 差速驱动 → /joint_states
```

---

## 排查过的四个核心问题

### 1. costmap 注册了 topic 但 0 条消息
- **现象**: `/global_costmap/costmap` 存在，`ros2 topic hz` 返回 0Hz
- **排查链**:
  1. `ros2 topic info` → publisher 有，subscriber 有 ✓
  2. `ros2 lifecycle get` → 节点卡在 `activating [13]` 而非 `active [3]` ✗
  3. 原因：AMCL 没收初始位姿 → `map` 帧不存在 → costmap 的 `on_activate()` 无法完成
  4. 解决：在 RViz 里点 "2D Pose Estimate" 后 costmap 立即激活到 active
- **学到的**: ROS2 lifecycle node 的 activation 不是无条件的，依赖 TF 帧存在才能完成转换

### 2. DWB 不走规划路径，直接冲正前方
- **现象**: `/plan` 路径正确，但车不转弯，沿当前航向直走
- **排查链**:
  1. 确认 `/plan` 有数据，planner 正常工作 ✓
  2. 确认 `/scan` 有数据，costmap active ✓
  3. 查 DWB 源码：`short_circuit_trajectory_evaluation=True` → 找到第一条"还行"的轨迹就停
  4. 正前方那条刚好过门槛，后面对齐路径的轨迹根本未被评估
  5. 解决：改 `False`，强制评估全部 400 条轨迹 → 立即贴路径
- **学到的**: DWB 的轨迹评分是竞争机制，捷径评估可能导致次优解

### 3. DDS 多播失败 (Network is unreachable)
- **现象**: Gazebo 日志疯狂刷 "Exception sending a multicast message: Network is unreachable"
- **原因**: 系统默认路由走 PPP 拨号接口 (USB 热点)，该接口不支持多播
- **排查**: `ip route` 看到 `default via 115.200.0.1 dev ppp0`
- **解决**: 写 FastDDS profile，设 `initialPeersList` 为 `127.0.0.1`，禁用多播发现
- **学到的**: ROS2 的 DDS 通信对网络拓扑敏感，默认多播不适用于所有网络环境

### 4. `/map` 话题 QoS 不匹配
- **现象**: global_costmap 日志显示 "incompatible QoS"
- **原因**: `map_server` 发布 `/map` 使用 `TRANSIENT_LOCAL` durability，但 `nav_activator` relay 用的是默认 `VOLATILE`
- **解决**: 让 activator 的 publisher/subscriber 都使用 `TRANSIENT_LOCAL`
- **学到的**: ROS2 QoS 配置决定 publisher/subscriber 是否能匹配，durability 不一致会导致通信失败

---

## DWB 关键参数速记

| 参数 | 作用 | 我们用的值 |
|---|---|---|
| `short_circuit_trajectory_evaluation` | False=评估全部轨迹 | **False** |
| `PathAlign.scale` | 轨迹方向与路径方向对齐程度 | 80 |
| `PathDist.scale` | 轨迹终点离路径的距离 | 60 |
| `BaseObstacle.scale` | 避障权重 | 0.5 |
| `RotateToGoal.scale` | 转向目标的优先度 | 50 |
| `vx_samples × vtheta_samples` | 轨迹采样数量 | 20×20=400 |
| `sim_time` | 前向模拟时长 (秒) | 1.7 |
| `inflation_radius` | 障碍物膨胀半径 (m) | 0.6 |

---

## TF 树

```
map → odom → base_footprint → base_link → pole → lidar_link
                                       ├── caster_link
                                       └── left_wheel / right_wheel
```

- `map→odom`: AMCL 发布的定位修正
- `odom→base_footprint`: 里程计发布的增量位姿
- `base_footprint→base_link`: URDF 静态变换
