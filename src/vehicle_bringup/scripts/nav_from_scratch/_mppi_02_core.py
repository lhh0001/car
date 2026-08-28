#!/usr/bin/env python3
"""
=== MPPI 02：核心算法 ===

MPPI = Model Predictive Path Integral Control

一句话：在控制空间里撒 K 条噪声轨迹，用代价函数加权平均，选最优控制。

数学推导（简化）：

设我们要优化一个有限时域 T 的代价函数：
  J(U) = φ(x_T) + Σ_t q(x_t, u_t)

其中 U = [u_0, u_1, ..., u_{T-1}] 是控制序列。

MPPI 的关键洞察：如果我们在基准控制序列 Ũ 上加高斯噪声 ε，
然后用代价的指数衰减做重要性采样权重，最优控制更新是：
  u_i* = ũ_i + Σ_k w_k · ε_i^k / Σ_k w_k

其中：
  w_k = exp(-1/λ · S_k)     ← 重要性权重
  S_k = 累加代价             ← 轨迹的总代价
  ε_i^k = 第 k 条轨迹在第 i 步的噪声
  λ = 温度参数（控制探索 vs 利用的平衡）

算法流程：
  1. 初始化控制序列 U = [u_0, u_1, ..., u_{T-1}]
  2. 重复：
     a) 生成 K 条噪声轨迹: U_k = U + ε_k,  ε_k ~ N(0, Σ)
     b) 用动力学模型 rollout 每条轨迹，计算累积代价 S_k
     c) 计算重要性权重: w_k = exp(-(S_k - min_S) / λ)
     d) 加权平均: U ← U + Σ(w_k · ε_k) / Σw_k
     e) 执行 u_0，U 向左移一位（滚动时域）
"""

import numpy as np
from _mppi_01_dynamics import diff_drive_dynamics, rollout_trajectory


class MPPIController:
    """MPPI 轨迹优化控制器"""

    def __init__(self,
                 horizon: int = 40,          # 时域步数 (T)
                 num_samples: int = 1000,     # 轨迹采样数 (K)
                 dt: float = 0.05,           # 时间步长
                 temperature: float = 0.1,    # 温度参数 λ
                 noise_std_v: float = 0.3,   # 线速度噪声标准差
                 noise_std_omega: float = 1.5,# 角速度噪声标准差
                 ):
        self.T = horizon
        self.K = num_samples
        self.dt = dt
        self.lam = temperature

        # 控制噪声协方差（对角）
        self.noise_std = np.array([noise_std_v, noise_std_omega])

        # 控制序列（初始化为零）
        self.U = np.zeros((self.T, 2))

    def sample_noise(self) -> np.ndarray:
        """生成 K 个噪声序列: shape (K, T, 2)"""
        return np.random.randn(self.K, self.T, 2) * self.noise_std

    def compute_cost(self, states: np.ndarray, path: np.ndarray,
                     controls: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """
        计算每条轨迹的累积代价（标量）。

        states:  (K, T+1, 3)  各条轨迹的状态序列
        path:    (M, 2)        参考路径点序列
        controls:(K, T, 2)     控制序列
        goal:    (2,)          目标位置 [gx, gy]

        代价组成:
          - 路径跟踪代价: 轨迹点离路径最近点的距离
          - 控制代价: 防止过猛加速
          - 目标代价: 靠近目标加分
        """
        K, Tp1, _ = states.shape
        cost = np.zeros(K)

        for k in range(K):
            traj_cost = 0.0
            for t in range(min(Tp1, self.T + 1)):
                traj_pt = states[k, t, :2]  # (x, y)

                # 路径跟踪: 到参考路径最近点的距离
                dist_to_path = np.min(np.linalg.norm(path - traj_pt, axis=1))

                # 控制平滑: 角速度变化的平方
                if t < self.T:
                    omega = controls[k, t, 1]
                    traj_cost += 0.05 * omega**2

                traj_cost += dist_to_path * 2.0  # 路径跟踪权重

            # 目标代价: 终点离目标多远
            final_pos = states[k, -1, :2]
            dist_to_goal = np.linalg.norm(final_pos - goal)
            traj_cost += dist_to_goal * 5.0

            cost[k] = traj_cost

        return cost

    def step(self, current_state: np.ndarray,
             path: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """
        执行一次 MPPI 迭代，返回最优控制 (v, ω)。

        参数:
          current_state: [x, y, θ] 当前机器人状态
          path:          参考路径 (M×2)
          goal:          [gx, gy] 目标位置

        返回:
          [v, ω] 该步最优控制
        """
        if len(path) == 0:
            return np.zeros(2)

        # --- Step 1: 采样噪声 ---
        noise = self.sample_noise()  # (K, T, 2)

        # --- Step 2: 构建 K 条控制序列 U + ε ---
        U_batch = self.U[np.newaxis, :, :] + noise  # (K, T, 2)

        # --- Step 3: 并 roll 每条轨迹 ---
        states_batch = np.zeros((self.K, self.T + 1, 3))
        for k in range(self.K):
            states_batch[k] = rollout_trajectory(current_state,
                                                 U_batch[k], self.dt)

        # --- Step 4: 计算代价 ---
        costs = self.compute_cost(states_batch, path, U_batch, goal)

        # --- Step 5: 重要性权重 ---
        costs_min = costs.min()
        weights = np.exp(-(costs - costs_min) / self.lam)
        weights_sum = weights.sum()

        if weights_sum < 1e-12:
            return np.zeros(2)

        weights /= weights_sum

        # --- Step 6: 加权平均控制更新 ---
        # U ← U + Σ(w_k · ε_k) / Σw_k
        weighted_noise = np.sum(noise * weights[:, np.newaxis, np.newaxis], axis=0)
        self.U += weighted_noise

        # --- Step 7: 滚动时域 (shift) ---
        optimal_u = self.U[0].copy()  # 只执行第一步
        self.U[:-1] = self.U[1:]
        self.U[-1] = np.array([0.0, 0.0])  # 最后一位补零

        return optimal_u
