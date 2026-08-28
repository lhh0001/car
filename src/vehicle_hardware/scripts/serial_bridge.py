#!/usr/bin/env python3
"""
Serial bridge: PC ↔ ESP32 communication over USB.

Subscribes to /cmd_vel, converts to wheel speeds, sends to ESP32.
Reads encoder counts from ESP32, publishes /odom and /tf.
Also publishes /imu if the ESP32 reports IMU data.

Protocol (text, newline-delimited, 115200 baud):
  PC → ESP32:  M <left_pwm> <right_pwm>
  ESP32 → PC:  E <left_enc> <right_enc>
  ESP32 → PC:  I <ax> <ay> <az> <gx> <gy> <gz>
"""
import math
import struct
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

try:
    import serial
    _SERIAL_OK = True
except ImportError:
    _SERIAL_OK = False


# ─────────────────────────────── 电机参数（和 ESP32 固件一致） ───────────────

# N20 12GA 150RPM 减速比约 100:1，编码器 7 脉冲/转（电机端）
# 每圈输出轴 = 电机 7 * 减速比 100 ≈ 700 脉冲 / 轮圈
ENCODER_CPR = 700.0        # 编码器每转圈数（轮端）
WHEEL_RADIUS   = 0.04       # 轮子半径 (m)，4cm 常见
WHEEL_BASE     = 0.20       # 左右轮距 (m)
PWM_MAX        = 255        # 和固件一致


class SerialBridge(Node):
    """USB serial bridge between ROS2 and ESP32."""

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('encoder_cpr', ENCODER_CPR)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('wheel_base', WHEEL_BASE)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        if not _SERIAL_OK:
            self.get_logger().fatal('pyserial not installed: pip install pyserial')
            raise RuntimeError('pyserial missing')

        # 打开串口
        self._ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)  # ESP32 复位
        self.get_logger().info(f'Serial connected: {port} @ {baud}')

        # ---- ROS2 interface ----
        self._cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_br     = TransformBroadcaster(self)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._prev_left_enc  = 0
        self._prev_right_enc = 0
        self._prev_time = self.get_clock().now()

        # 定时读取串口
        self._timer = self.create_timer(0.02, self._read_serial)  # 50 Hz

    # ── 发送 ────────────────────────────────────────────────────────

    def _on_cmd(self, msg: Twist):
        """Cmd_vel → 差速解算 → PWM → 串口发送."""
        v = msg.linear.x  # m/s
        w = msg.angular.z  # rad/s

        # 差速模型: v_l = (2v - w*L) / 2R,  v_r = (2v + w*L) / 2R
        wheel_sep = self.get_parameter('wheel_base').value
        r         = self.get_parameter('wheel_radius').value
        v_left  = (2 * v - w * wheel_sep) / (2 * r)   # rad/s, 轮端角速度
        v_right = (2 * v + w * wheel_sep) / (2 * r)

        # 线性映射到 PWM（简化，不做 PID）
        pwm_left  = int(self._rads_to_pwm(v_left))
        pwm_right = int(self._rads_to_pwm(v_right))

        line = f'M {pwm_left} {pwm_right}\n'
        self._ser.write(line.encode())
        self.get_logger().info(f'CMD: {line.strip()}')

    def _rads_to_pwm(self, rad_s: float) -> float:
        """轮端角速度 → PWM（粗略线性映射，真实场景用 PID）."""
        # N20 12GA 150RPM 空载: 150 rpm = 15.7 rad/s → PWM_MAX
        MAX_RADS = 15.7
        pwm = (rad_s / MAX_RADS) * PWM_MAX
        return max(-PWM_MAX, min(PWM_MAX, pwm))

    # ── 接收 ────────────────────────────────────────────────────────

    def _read_serial(self):
        """50Hz 定时读取串口数据，有就处理，没有就跳过."""
        while self._ser.in_waiting > 0:
            line = self._ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            if line.startswith('E '):
                self._on_encoder(line)
            elif line.startswith('I '):
                self._on_imu(line)

    def _on_encoder(self, line: str):
        """解析编码器数据，计算里程计."""
        parts = line.split()
        if len(parts) < 3:
            return

        now = self.get_clock().now()
        left  = int(parts[1])
        right = int(parts[2])

        cpr = self.get_parameter('encoder_cpr').value
        r   = self.get_parameter('wheel_radius').value
        L   = self.get_parameter('wheel_base').value

        d_left  = (left  - self._prev_left_enc)  / cpr * 2 * math.pi
        d_right = (right - self._prev_right_enc) / cpr * 2 * math.pi
        self._prev_left_enc  = left
        self._prev_right_enc = right

        d_dist  = (d_right + d_left) / 2.0 * r
        d_yaw   = (d_right - d_left) / L * r
        dt      = (now - self._prev_time).nanoseconds / 1e9
        self._prev_time = now

        # 更新全局位姿
        self._odom_yaw += d_yaw
        self._odom_x   += d_dist * math.cos(self._odom_yaw)
        self._odom_y   += d_dist * math.sin(self._odom_yaw)

        # 速度
        vx = d_dist / dt if dt > 0 else 0.0
        vw = d_yaw  / dt if dt > 0 else 0.0

        # 发布 TF
        t = TransformStamped()
        t.header.stamp    = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self._odom_x
        t.transform.translation.y = self._odom_y
        t.transform.translation.z = 0.0
        q = self._yaw_to_quat(self._odom_yaw)
        t.transform.rotation = q
        self._tf_br.sendTransform(t)

        # 发布 Odometry
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = vw
        self._odom_pub.publish(odom)

    def _on_imu(self, line: str):
        """IMU 数据（预留，暂不发布）."""
        pass  # TODO: 发布 sensor_msgs/Imu

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        if self._ser and self._ser.is_open:
            self._ser.write(b'M 0 0\n')  # 停车
            self._ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
