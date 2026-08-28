#!/usr/bin/env python3
"""
=== MPPI 01：差速驱动运动学模型 ===

MPPI 需要模型来预测"如果我发 (v, ω) 这条指令，车 0.1 秒后会在哪里"。
这个模型就是状态转移函数 f(x_t, u_t) → x_{t+1}

差速驱动运动学：
  状态 x = [x, y, θ]ᵀ          (2D 位置 + 航向角)
  控制 u = [v, ω]ᵀ              (线速度 + 角速度)

  连续时间: ẋ = v·cos(θ),  ẏ = v·sin(θ),  θ̇ = ω
  离散时间: x_{k+1} = x_k + v_k·cos(θ_k)·dt
            y_{k+1} = y_k + v_k·sin(θ_k)·dt
            θ_{k+1} = θ_k + ω_k·dt

MPPI 需要多次调用这个函数做轨迹 rollout，所以必须快且数值稳定。
"""

import numpy as np


def diff_drive_dynamics(state: np.ndarray, control: np.ndarray, dt: float):
    """
    差速驱动机器人运动学模型的一次积分。

    参数:
      state:  当前状态 [x, y, θ]
      control: 控制输入 [v, ω]
      dt:     时间步长 (秒)

    返回:
      下一时刻状态 [x', y', θ']
    """
    x, y, theta = state
    v, omega = control

    x_next = x + v * np.cos(theta) * dt
    y_next = y + v * np.sin(theta) * dt
    theta_next = theta + omega * dt

    return np.array([x_next, y_next, theta_next])


def rollout_trajectory(state0: np.ndarray, controls: np.ndarray, dt: float):
    """
    从初始状态出发，执行一段控制序列，生成完整轨迹。

    参数:
      state0:   初始状态 [x, y, θ]
      controls: 控制序列 [[v0, ω0], [v1, ω1], ...], shape=(N_steps, 2)
      dt:       时间步长

    返回:
      states:   (N_steps+1, 3) — 含初始状态
    """
    N = len(controls)
    states = np.zeros((N + 1, 3))
    states[0] = state0

    for k in range(N):
        states[k + 1] = diff_drive_dynamics(states[k], controls[k], dt)

    return states


# ======================================================================
#  测试: 采样一条控制序列，看轨迹形状
# ======================================================================
if __name__ == '__main__':
    dt = 0.05  # 50Hz

    # 直走
    state0 = np.array([0.0, 0.0, 0.0])
    controls_straight = np.tile([0.5, 0.0], (40, 1))
    traj = rollout_trajectory(state0, controls_straight, dt)
    print(f"Straight: start {state0} → end {traj[-1]}")

    # 转弯
    controls_turn = np.tile([0.2, 1.0], (40, 1))
    traj_turn = rollout_trajectory(state0, controls_turn, dt)
    print(f"Turn:    start {state0} → end {traj_turn[-1]}")
