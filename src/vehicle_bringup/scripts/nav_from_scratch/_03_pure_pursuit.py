#!/usr/bin/env python3
"""
=== 导航基础 03：Pure Pursuit 路径跟踪 ===

几何路径跟踪 —— 不需要优化、不需要采样，直接几何计算"怎么转弯才能追上前面那个点"。

核心思想:
  1. 沿路径往前找一个"看前点"(lookahead point)，距离机器人 L (lookahead distance)
  2. 计算从机器人到那个点的圆弧曲率 κ
  3. ω = κ × v  →  角速度 = 曲率 × 线速度

纯几何推导（差速驱动）:
  设看前点在机器人坐标系下的坐标为 (lx, ly)
  曲率 κ = 2 × ly / (lx² + ly²)

  推导:
    圆弧中心在机器人左/右侧, 距离机器人 R = 1/κ
    看前点到圆弧中心距离也是 R
    (lx)² + (ly - R)² = R²
    → lx² + ly² - 2·ly·R = 0
    → R = (lx² + ly²) / (2·ly)
    → κ = 1/R = 2·ly / (lx² + ly²)
"""

import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
from _01_occupancy_grid import OccupancyGrid
from _02_astar_planner import AStarPlanner


class PurePursuit:
    """Pure Pursuit 路径跟踪控制器"""

    def __init__(self, lookahead_distance: float = 0.5,
                 max_angular_vel: float = 2.0):
        self.lookahead = lookahead_distance
        self.max_omega = max_angular_vel

    def find_lookahead_point(self, path: list, robot_wx: float, robot_wy: float):
        """
        在路径上找到距离机器人 lookahead 距离的那个点。

        从路径起点往后扫描，找第一个距离 ≥ lookahead 的点。
        如果所有点都太近（比如快到终点了），返回最后一个点。
        """
        closest_dist = float('inf')
        closest_idx = 0

        for i, (wx, wy) in enumerate(path):
            dist = np.sqrt((wx - robot_wx)**2 + (wy - robot_wy)**2)
            if dist >= self.lookahead:
                return i, (wx, wy)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i

        # 所有点都太近 → 返回最后一个
        return closest_idx, path[closest_idx]

    def compute_curvature(self, robot_wx, robot_wy, robot_yaw,
                          lookahead_wx, lookahead_wy):
        """
        计算到达看前点需要的曲率。

        关键步骤: 把看前点从世界坐标系转到机器人坐标系
        旋转矩阵: [cosθ  sinθ ] [dx]
                  [-sinθ cosθ] [dy]
        """
        dx = lookahead_wx - robot_wx
        dy = lookahead_wy - robot_wy

        # 转到机器人坐标系
        lx = np.cos(robot_yaw) * dx + np.sin(robot_yaw) * dy
        ly = -np.sin(robot_yaw) * dx + np.cos(robot_yaw) * dy

        # 曲率: κ = 2 × ly / (lx² + ly²)
        L_sq = lx * lx + ly * ly
        if L_sq < 1e-6:
            return 0.0

        curvature = 2.0 * ly / L_sq
        return curvature

    def compute_cmd_vel(self, path: list, robot_wx: float, robot_wy: float,
                        robot_yaw: float, desired_linear_vel: float):
        """
        输入: 路径 + 机器人当前位姿 + 期望线速度
        输出: (v, ω) 速度指令
        """
        if not path:
            return 0.0, 0.0

        # 1. 找看前点
        lookahead_idx, (lx, ly) = self.find_lookahead_point(
            path, robot_wx, robot_wy)

        # 2. 计算曲率
        kappa = self.compute_curvature(robot_wx, robot_wy, robot_yaw, lx, ly)

        # 3. 曲率 → 角速度
        omega = kappa * desired_linear_vel

        # 4. 限幅
        omega = np.clip(omega, -self.max_omega, self.max_omega)

        # 5. 接近终点时减速
        dist_to_goal = np.sqrt((path[-1][0] - robot_wx) ** 2 +
                               (path[-1][1] - robot_wy)**2)
        if dist_to_goal < self.lookahead:
            desired_linear_vel *= dist_to_goal / self.lookahead

        return desired_linear_vel, omega


class SimulatedRobot:
    """简化的差速驱动机器人运动学仿真"""

    def __init__(self, x=0, y=0, yaw=0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def update(self, v: float, omega: float, dt: float):
        """差速运动学模型积分一次"""
        self.x += v * np.cos(self.yaw) * dt
        self.y += v * np.sin(self.yaw) * dt
        self.yaw += omega * dt


# ======================================================================
#  Demo: A* 规划 + Pure Pursuit 跟踪的完整闭环
# ======================================================================
if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    maps_dir = os.path.join(os.path.dirname(script_dir), 'maps')

    # --- 1. 加载地图 ---
    grid = OccupancyGrid(
        os.path.join(maps_dir, 'mymap.pgm'),
        os.path.join(maps_dir, 'mymap.yaml'),
    )

    # --- 2. A* 规划路径 ---
    planner = AStarPlanner(grid)
    start_wx, start_wy = -5.0, -5.0
    goal_wx, goal_wy = 10.0, 8.0
    path = planner.plan(start_wx, start_wy, goal_wx, goal_wy)

    if not path:
        print("No path! Pick different start/goal.")
        sys.exit(1)

    print(f"Path: {len(path)} waypoints")

    # --- 3. Pure Pursuit 跟踪仿真 ---
    controller = PurePursuit(lookahead_distance=0.5, max_angular_vel=2.0)
    robot = SimulatedRobot(start_wx, start_wy, yaw=np.pi/4)  # 初始航向 45°
    dt = 0.05  # 50Hz
    desired_v = 0.5

    history_x, history_y = [], []

    for step in range(2000):
        history_x.append(robot.x)
        history_y.append(robot.y)

        # 到达？
        dist_to_goal = np.sqrt((goal_wx - robot.x)**2 + (goal_wy - robot.y)**2)
        if dist_to_goal < 0.1:
            print(f"Reached goal at step {step}!")
            history_x.append(robot.x)
            history_y.append(robot.y)
            break

        v, omega = controller.compute_cmd_vel(path, robot.x, robot.y,
                                              robot.yaw, desired_v)
        robot.update(v, omega, dt)

        if step % 200 == 0:
            print(f"  step {step}: pos=({robot.x:.2f},{robot.y:.2f}) "
                  f"yaw={np.degrees(robot.yaw):.0f}° v={v:.2f} ω={omega:.2f}")

    # --- 4. 画图: 路径 + 实际轨迹 ---
    fig, ax = plt.subplots(figsize=(10, 10))
    grid.plot(ax=ax, title="Pure Pursuit: Path Following")

    # A* 路径
    if path:
        px, py = zip(*path)
        ax.plot(px, py, 'r--', linewidth=2, alpha=0.7, label='A* Path')

    # 实际轨迹
    ax.plot(history_x, history_y, 'b-', linewidth=2, label='Robot Trajectory')

    ax.plot(start_wx, start_wy, 'go', markersize=10, label='Start')
    ax.plot(goal_wx, goal_wy, 'ro', markersize=10, label='Goal')
    ax.legend()
    plt.tight_layout()
    plt.show()
