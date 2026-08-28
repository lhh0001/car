#!/usr/bin/env python3
"""
=== 导航基础 02：A* 路径规划 ===

在占据栅格地图上实现 A* (A-star) 搜索算法 —— 全局路径规划的核心。

核心思想:
  f(n) = g(n) + h(n)
    g(n): 从起点到当前节点的实际代价
    h(n): 从当前节点到终点的启发式估计 (用欧几里得距离/曼哈顿距离)
    f(n): 总估计代价，A* 每次扩展 f 最小的节点

为什么 A* 能找到最优路径:
  只要 h(n) ≤ 真实代价 (admissible)，A* 保证找到最短路径。
  欧几里得距离满足这个条件。
"""

import numpy as np
import heapq
import matplotlib.pyplot as plt
from collections import deque
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
from _01_occupancy_grid import OccupancyGrid


# 8方向移动的代价和偏移量
#   cost: sqrt(2) 表示对角线走 1.414 格的距离
#   4方向用 1.0，保证 admissible
MOVES_8 = [
    (-1,  0, 1.0),       # 上
    (1,  0, 1.0),        # 下
    (0, -1, 1.0),        # 左
    (0,  1, 1.0),        # 右
    (-1, -1, 1.414),     # 左上
    (-1,  1, 1.414),     # 右上
    (1, -1, 1.414),      # 左下
    (1,  1, 1.414),      # 右下
]


class AStarPlanner:
    """A* 全局路径规划器"""

    def __init__(self, grid: OccupancyGrid):
        self.grid = grid

    def heuristic(self, row1, col1, row2, col2):
        """启发函数：欧几里得距离 (admissible → 保证最优)"""
        return np.sqrt((row1 - row2)**2 + (col1 - col2)**2)

    def plan(self, start_wx: float, start_wy: float,
             goal_wx: float, goal_wy: float) -> list:
        """
        A* 搜索主函数。

        参数:
          start_wx, start_wy: 起点世界坐标
          goal_wx, goal_wy:   终点世界坐标

        返回:
          list of (wx, wy): 路径点序列，空列表表示无法到达
        """
        start = self.grid.world_to_map(start_wx, start_wy)
        goal = self.grid.world_to_map(goal_wx, goal_wy)

        start_row, start_col = start
        goal_row, goal_col = goal

        # 边界检查
        if not self.grid.is_in_bounds(start_row, start_col):
            print(f"ERROR: start {start} out of bounds")
            return []
        if not self.grid.is_in_bounds(goal_row, goal_col):
            print(f"ERROR: goal {goal} out of bounds")
            return []
        if self.grid.is_occupied(goal_row, goal_col):
            print(f"ERROR: goal {goal} is occupied!")
            return []

        # --- A* 数据结构 ---
        # open_set: 优先队列 (f, 计数, (row, col)) —— 计数用于打破平局
        open_set = []
        heapq.heappush(open_set,
                       (self.heuristic(start_row, start_col, goal_row, goal_col),
                        0, start_row, start_col))
        counter = 1

        # 记录每个节点的 g 值 和 来源
        g_score = {start: 0.0}
        came_from = {start: None}

        # 已探索节点（用于可视化）
        self.explored = set()

        while open_set:
            f_val, _, row, col = heapq.heappop(open_set)
            current = (row, col)
            self.explored.add(current)

            # 到达终点
            if current == goal:
                return self._reconstruct_path(came_from, current)

            # 8方向展开
            for dr, dc, move_cost in MOVES_8:
                nr, nc = row + dr, col + dc
                neighbor = (nr, nc)

                # 出界或碰障碍
                if not self.grid.is_in_bounds(nr, nc):
                    continue
                if self.grid.is_occupied(nr, nc):
                    continue

                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(nr, nc, goal_row, goal_col)
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f, counter, nr, nc))
                    counter += 1

        print("A*: NO PATH FOUND!")
        return []

    def _reconstruct_path(self, came_from: dict, current: tuple) -> list:
        """回溯得到从起点到终点的路径点序列"""
        path = []
        while current is not None:
            row, col = current
            wx, wy = self.grid.map_to_world(row, col)
            path.append((wx, wy))
            current = came_from[current]
        path.reverse()
        return path

    def plot_search(self, ax, start, goal):
        """在图上画出 A* 搜索过程 (展开的节点)"""
        if not self.explored:
            return
        rows, cols = zip(*self.explored)
        wxs = [self.grid.origin_x + (c + 0.5) * self.grid.resolution
               for c in cols]
        wys = [self.grid.origin_y + (self.grid.height - 1 - r + 0.5) * self.grid.resolution
               for r in rows]
        ax.scatter(wxs, wys, c='lightblue', s=1, alpha=0.5, label='Explored')
        ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
        ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')


# ======================================================================
#  Demo
# ======================================================================
if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    maps_dir = os.path.join(os.path.dirname(script_dir), 'maps')

    grid = OccupancyGrid(
        os.path.join(maps_dir, 'mymap.pgm'),
        os.path.join(maps_dir, 'mymap.yaml'),
    )

    planner = AStarPlanner(grid)

    # 在地图上选起点和终点
    start_wx = 0.0
    start_wy = 0.0
    goal_wx = 8.0
    goal_wy = 6.0

    print(f"Planning: ({start_wx}, {start_wy}) → ({goal_wx}, {goal_wy})")
    path = planner.plan(start_wx, start_wy, goal_wx, goal_wy)

    if path:
        print(f"Path found: {len(path)} waypoints")
        px, py = zip(*path)
    else:
        px, py = [], []

    # 画图
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # 左: 仅地图 + 搜索过程
    grid.plot(ax=axes[0], title="A* Search (blue=explored nodes)")
    planner.plot_search(axes[0], (start_wx, start_wy), (goal_wx, goal_wy))

    # 右: 路径
    grid.plot(ax=axes[1], title="A* Path Result")
    if px:
        axes[1].plot(px, py, 'r-', linewidth=2, label=f'Path ({len(px)} pts)')
    axes[1].plot(start_wx, start_wy, 'go', markersize=10, label='Start')
    axes[1].plot(goal_wx, goal_wy, 'ro', markersize=10, label='Goal')
    axes[1].legend()

    plt.tight_layout()
    plt.show()
