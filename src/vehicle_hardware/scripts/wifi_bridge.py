#!/usr/bin/env python3
"""
WiFi bridge: PC ↔ ESP32 communication over UDP.

Subscribes to /cmd_vel → PID 轮速闭环 → sends PWM to ESP32.
Reads encoder counts from ESP32, publishes /odom and /tf.

Protocol (UDP, text, newline-delimited):
  PC → ESP32:  M <left_pwm> <right_pwm>
  ESP32 → PC:  E <left_enc> <right_enc>
  ESP32 → PC:  I <ax> <ay> <az> <gx> <gy> <gz>
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import socket
import select


ENCODER_CPR = 700.0        # 编码器每转圈数（轮端）
WHEEL_RADIUS   = 0.04       # 轮子半径 (m)，4cm 常见
WHEEL_BASE     = 0.20       # 左右轮距 (m)
PWM_MAX        = 255        # 和固件一致


# ============================================================
#  PID 控制器
# ============================================================
class PID:
    """增量式 PID，带积分限幅防饱和."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_max: float, integ_max: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.integ_max = integ_max
        self.reset()


    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time_ns = None

    def step(self, setpoint: float, measurement: float,
             now_ns: int) -> float:
        """输入目标值、测量值、当前时间(ns)，输出控制量."""
        error = setpoint - measurement

        if self._prev_time_ns is not None:
            dt = (now_ns - self._prev_time_ns) / 1e9
            if dt <= 1e-6:
                dt = 1e-6
        else:
            dt = 0.001

        # 积分 + 限幅（anti-windup）
        self._integral += error * dt
        self._integral = max(-self.integ_max, min(self.integ_max,
                                                   self._integral))

        # 微分
        derivative = (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time_ns = now_ns

        output = (self.kp * error +
                  self.ki * self._integral +
                  self.kd * derivative)
        return max(-self.out_max, min(self.out_max, output))

class WifiBridge(Node):
    def __init__(self):
        #继承父类，起名字  
        super().__init__("wifi_bridge")
        self.declare_parameter("host", "192.168.4.1")
        self.declare_parameter("port", 8888)
        
        #声明可调的参数
        self.declare_parameter('encoder_cpr', ENCODER_CPR)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('wheel_base', WHEEL_BASE)
        
        #打开传输通道
        self._sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self._sock.setblocking(False)

        self._host=self.get_parameter("host").value
        self._port=self.get_parameter("port").value
        self._sock.bind(("0.0.0.0",self._port))
        # 启动心跳：告诉 ESP32 PC 的 IP，否则 ESP32 不发编码器，PID 永远不跑
        self._sock.sendto(b'M 0 0\n', (self._host, self._port))

        #订阅/发布话题
        self._cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_br     = TransformBroadcaster(self)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._prev_left_enc  = 0
        self._prev_right_enc = 0
        self._enc_ready = False      # 第一次读到编码器只初始化，不算增量
        self._target_left = 0.0
        self._target_right = 0.0
        self._last_cmd_time = self.get_clock().now()
        self.declare_parameter('pid_kp', 10.0)
        self.declare_parameter('pid_ki', 2.0)
        self.declare_parameter('pid_kd', 0.0)
        self.declare_parameter('pid_integ_max', 100.0)

        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        imax = self.get_parameter('pid_integ_max').value
        self._pid_left = PID(kp, ki, kd, PWM_MAX, imax)
        self._pid_right = PID(kp, ki, kd, PWM_MAX, imax)
        self._prev_time = self.get_clock().now()

        # 启动定时轮询 
        self._timer = self.create_timer(0.02, self._read_socket)  # 50 Hz
   
    def _on_cmd(self, msg: Twist):
        v = msg.linear.x  # m/s
        w = msg.angular.z  # rad/s

        # 差速模型: v_l = (2v - w*L) / 2R,  v_r = (2v + w*L) / 2R
        wheel_sep = self.get_parameter('wheel_base').value
        r         = self.get_parameter('wheel_radius').value
        v_left  = (2 * v - w * wheel_sep) / (2 * r)   # rad/s, 轮端角速度
        v_right = (2 * v + w * wheel_sep) / (2 * r)

        self._target_left=v_left
        self._target_right=v_right
        self._last_cmd_time = self.get_clock().now()
        # 线性映射到 PWM（简化，不做 PID）
        #pwm_left  = int(self._rads_to_pwm(v_left))
        #pwm_right = int(self._rads_to_pwm(v_right))

        #line = f'M {pwm_left} {pwm_right}\n'
        #self._sock.sendto(line.encode(), (self._host, self._port))
        #self.get_logger().info(f'CMD: {line.strip()}')
    
    def _rads_to_pwm(self, rad_s: float) -> float:
        """轮端角速度 → PWM（粗略线性映射，真实场景用 PID）."""
        # N20 12GA 150RPM 空载: 150 rpm = 15.7 rad/s → PWM_MAX
        MAX_RADS = 15.7
        pwm = (rad_s / MAX_RADS) * PWM_MAX
        return max(-PWM_MAX, min(PWM_MAX, pwm))
    
    def _read_socket(self):
        """50Hz 定时读取串口数据，有就处理，没有就跳过."""
        ready, _, _ = select.select([self._sock], [], [], 0)
        if not ready:
            return
        data = self._sock.recv(1024).decode(errors="ignore")
        for line in data.split("\n"):
            line =line.strip()
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

        if not self._enc_ready:
            # 第一次读数：初始化基准值，跳过增量计算
            self._prev_left_enc  = left
            self._prev_right_enc = right
            self._prev_time = now
            self._enc_ready = True
            return

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
        
        if dt >0:
            if (now - self._last_cmd_time).nanoseconds / 1e9 > 0.5:
                self._target_left = 0.0
                self._target_right = 0.0
                
            actual_left=d_left/dt
            actual_right=d_right/dt

            # 停车检测：目标为0且实际轮速接近0时，直接发0并复位PID
            # 这样能消除积分残留和编码器噪声导致的微动/蜂鸣
            STOP_THRESH = 0.5  # rad/s, 低于此值视为静止
            if self._target_left == 0.0 and self._target_right == 0.0 \
               and abs(actual_left) < STOP_THRESH and abs(actual_right) < STOP_THRESH:
                self._pid_left.reset()
                self._pid_right.reset()
                line = 'M 0 0\n'
                self._sock.sendto(line.encode(), (self._host, self._port))
                return

            pwm_left = int(self._pid_left.step(self._target_left,actual_left,now.nanoseconds))
            pwm_right = int(self._pid_right.step(self._target_right, actual_right, now.nanoseconds))

            line = f'M {pwm_left} {pwm_right}\n'
            self._sock.sendto(line.encode(), (self._host, self._port))


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
        try:
            self._sock.sendto(b'M 0 0\n', (self._host, self._port))
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = WifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


    





        


        


