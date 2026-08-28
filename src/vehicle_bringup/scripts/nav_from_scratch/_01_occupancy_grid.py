#!/usr/bin/env python3
"""
=== 导航基础 01：占据栅格地图 (Occupancy Grid) ===

从零实现占据栅格地图的核心数据结构，不依赖任何 Nav2 代码。
这是所有导航算法的地基 —— A*、Dijkstra、代价地图、碰撞检测全都建在这上面。

关键概念：
  - 占据栅格：把连续世界离散化成 grid[x][y] 的格子
  - 三值状态：FREE(0) / OCCUPIED(100) / UNKNOWN(-1)
  - 坐标转换：世界坐标 <-> 栅格索引 互转是规划器的入口
  - 射线投射：模拟激光雷达扫描，判断哪些格子被障碍物挡住
"""

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import yaml
import os


class OccupancyGrid:
    """占据栅格地图 —— 导航算法的地基"""

    # 三值状态（ROS2 OccupancyGrid 标准）
    FREE = 0        # 空闲（可通行）
    OCCUPIED = 100  # 占据（障碍物）
    UNKNOWN = -1    # 未知（未探索）

    def __init__(self, pgm_path: str, yaml_path: str = None):
        """
        从 PGM 图片加载占据栅格地图。

        PGM (Portable GrayMap) 是 SLAM 建图的标准输出格式，
        每个像素的灰度值代表该位置的占据概率：
          - 0 (黑色)   → OCCUPIED (墙/障碍物)
          - 254 (白色) → FREE (可通行区域)
          - 205 (灰色) → UNKNOWN (未探索)
        """
        # --- 加载 YAML 元数据 ---
        if yaml_path is None:
            yaml_path = pgm_path.replace('.pgm', '.yaml')
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        self.resolution = meta['resolution']      # 米/像素
        self.origin_x = meta['origin'][0]          # 地图左下角世界 X
        self.origin_y = meta['origin'][1]          # 地图左下角世界 Y

        # --- 加载 PGM 图片 ---
        img = Image.open(pgm_path)
        self.width = img.width     # 栅格列数 (x方向)
        self.height = img.height   # 栅格行数 (y方向)
        raw = np.array(img)

        # --- 灰度 → 三值状态 ---
        # PGM: 黑色=障碍, 白色=空闲, 灰色=未知
        # ROS2 OccupancyGrid 存储约定: 0=Free, 100=Occupied, -1=Unknown
        self.data = np.full((self.height, self.width), self.UNKNOWN, dtype=np.int8)
        self.data[raw > 250] = self.FREE           # 白→空闲
        self.data[raw < 5] = self.OCCUPIED         # 黑→占据

        # 预计算世界坐标范围（方便画图）
        self.world_min_x = self.origin_x
        self.world_min_y = self.origin_y
        self.world_max_x = self.origin_x + self.width * self.resolution
        self.world_max_y = self.origin_y + self.height * self.resolution

        print(f"Grid: {self.width} x {self.height} cells @ {self.resolution:.3f}m")
        print(f"World: [{self.world_min_x:.2f}, {self.world_max_x:.2f}] x "
              f"[{self.world_min_y:.2f}, {self.world_max_y:.2f}]")

    # ==================================================================
    #  核心操作 1: 坐标转换（世界坐标 ↔ 栅格索引）
    #  这是规划器的入口 —— 机器人知道自己的世界坐标 (x,y)，
    #  要知道它在地图 grid 的哪个格子里。
    # ==================================================================

    def world_to_map(self, wx: float, wy: float) -> tuple:
        """
        世界坐标 → 栅格索引 (row, col)
        注意: 行号从上往下 (Y轴), 列号从左往右 (X轴)

        推导:
          col = (wx - origin_x) / resolution
          row = (wy - origin_y) / resolution

        因为 PGM 图片 row=0 在顶部 而世界坐标 y 越大越靠上，
        所以 row = height - 1 - (wy - origin_y) / resolution
        """
        col = int((wx - self.origin_x) / self.resolution)
        row = self.height - 1 - int((wy - self.origin_y) / self.resolution)
        return row, col

    def map_to_world(self, row: int, col: int) -> tuple:
        """栅格索引 → 该格子中心的世界坐标"""
        wx = self.origin_x + (col + 0.5) * self.resolution
        wy = self.origin_y + (self.height - 1 - row + 0.5) * self.resolution
        return wx, wy

    # ==================================================================
    #  核心操作 2: 碰撞检测
    # ==================================================================

    def is_in_bounds(self, row: int, col: int) -> bool:
        """检查栅格坐标是否在地图范围内"""
        return 0 <= row < self.height and 0 <= col < self.width

    def is_occupied(self, row: int, col: int) -> bool:
        """该格子是否被障碍物占据"""
        if not self.is_in_bounds(row, col):
            return True  # 地图外的区域视为占据（边界保护）
        return self.data[row, col] == self.OCCUPIED

    def is_free(self, row: int, col: int) -> bool:
        """该格子是否可通行"""
        if not self.is_in_bounds(row, col):
            return False
        return self.data[row, col] == self.FREE

    # ==================================================================
    #  核心操作 3: 射线投射 (Ray Casting)
    #  模拟激光雷达: 从机器人沿某个方向发一条射线，找到第一个碰到的障碍物。
    #  用 Bresenham 直线算法 —— 逐格推进，效率 O(max(w,h))。
    # ==================================================================

    def raycast(self, start_row: int, start_col: int,
                end_row: int, end_col: int) -> tuple:
        """
        Bresenham 直线投射。
        返回: (hit_row, hit_col, is_obstacle)
          如果碰到障碍物 → 返回障碍物格子坐标
          如果到达终点    → 返回终点坐标, is_obstacle=False
        """
        dr = abs(end_row - start_row)
        dc = abs(end_col - start_col)
        sr = 1 if end_row > start_row else -1
        sc = 1 if end_col > start_col else -1
        err = dr - dc

        row, col = start_row, start_col

        while row != end_row or col != end_col:
            if self.is_occupied(row, col):
                return row, col, True  # 碰到障碍物
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                row += sr
            if e2 < dr:
                err += dr
                col += sc

        return end_row, end_col, self.is_occupied(end_row, end_col)

    # ==================================================================
    #  可视化
    # ==================================================================

    def plot(self, ax=None, title="Occupancy Grid"):
        """用 matplotlib 画出占据栅格地图"""
        if ax is None:
            _, ax = plt.subplots(figsize=(8, 8))

        # 构建可视化数组 (白色=空闲, 黑色=占据, 灰色=未知)
        vis = np.full((self.height, self.width, 3), 0.5)  # 灰色底
        vis[self.data == self.FREE] = [1, 1, 1]            # 白
        vis[self.data == self.OCCUPIED] = [0, 0, 0]        # 黑

        extent = [self.world_min_x, self.world_max_x,
                  self.world_min_y, self.world_max_y]
        ax.imshow(vis, origin='lower', extent=extent)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(title)

        # 画上一个"机器人"标记
        mid_x = (self.world_min_x + self.world_max_x) / 2
        mid_y = (self.world_min_y + self.world_max_y) / 2
        ax.plot(mid_x, mid_y, 'r*', markersize=15, label='Robot')
        ax.legend()
        return ax


# ======================================================================
#  Demo: 加载地图，测试核心操作
# ======================================================================
if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    project_dir = os.path.dirname(script_dir)  # vehicle_bringup/
    maps_dir = os.path.join(project_dir, 'maps')

    grid = OccupancyGrid(
        os.path.join(maps_dir, 'mymap.pgm'),
        os.path.join(maps_dir, 'mymap.yaml'),
    )

    # --- 测试 1: 坐标转换 ---
    print("\n=== 测试 1: 坐标转换 ===")
    # 地图中心附近的一个点
    cx = (grid.world_min_x + grid.world_max_x) / 2
    cy = (grid.world_min_y + grid.world_max_y) / 2
    row, col = grid.world_to_map(cx, cy)
    wx, wy = grid.map_to_world(row, col)
    print(f"  世界 ({cx:.2f}, {cy:.2f}) → 栅格 ({row}, {col}) → 世界 ({wx:.2f}, {wy:.2f})")

    # --- 测试 2: 碰撞检测 ---
    print("\n=== 测试 2: 碰撞检测 ===")
    # 地图上找个明显是墙的位置
    wall_test_col = 500  # 随便取一个，改你自己地图里的值
    wall_test_row = 400
    if grid.is_in_bounds(wall_test_row, wall_test_col):
        print(f"  栅格 ({wall_test_row}, {wall_test_col}) 占据={grid.is_occupied(wall_test_row, wall_test_col)}")

    # --- 测试 3: 射线投射 ---
    print("\n=== 测试 3: 射线投射 (Bresenham) ===")
    # 从机器人位置往一个方向投射
    robot_row, robot_col = grid.world_to_map(cx, cy)
    hit_row, hit_col, is_obs = grid.raycast(robot_row, robot_col, 200, 700)
    print(f"  {robot_row, robot_col} → {hit_row}, {hit_col}, 命中障碍={is_obs}")

    # --- 画图 ---
    grid.plot(title="Map loaded from PGM")
    plt.show()
