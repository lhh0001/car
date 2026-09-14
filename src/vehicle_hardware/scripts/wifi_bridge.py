#!/usr/bin/env python3
"""
WiFi bridge: PC ↔ ESP32 communication over UDP.

Subscribes to /cmd_vel → PID 轮速闭环 → sends PWM to ESP32.
Reads encoder counts from ESP32, publishes /odom and /tf.

Protocol (UDP, text, newline-delimited):
  PC → ESP32:  M <left_pwm> <right_pwm>
  ESP32 → PC:  E <boot_id> <sequence> <sample_ms> <left_enc> <right_enc>
               （仍兼容旧版 E <left_enc> <right_enc>）
  ESP32 → PC:  I <ax> <ay> <az> <gx> <gy> <gz>
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
import socket
import select


ENCODER_CPR = 360.0        # A 相 RISING 计数，实测约 358-359 次/轮圈
WHEEL_RADIUS   = 0.02       # 轮子半径 (m)，实车直径 4 cm
WHEEL_BASE     = 0.13       # 左右轮中心距 (m)
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

    def step(self, setpoint: float, measurement: float, dt: float) -> float:
        """输入目标值、测量值和设备采样周期，输出控制量。"""
        error = setpoint - measurement
        dt = max(dt, 1e-6)

        # 积分 + 限幅（anti-windup）
        self._integral += error * dt
        self._integral = max(-self.integ_max, min(self.integ_max,
                                                   self._integral))

        # 微分
        derivative = (error - self._prev_error) / dt

        self._prev_error = error
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
        self.declare_parameter("bind_host", "192.168.4.2")
        
        #声明可调的参数
        self.declare_parameter('encoder_cpr', ENCODER_CPR)
        self.declare_parameter('left_encoder_direction', 1.0)
        self.declare_parameter('right_encoder_direction', -1.0)
        self.declare_parameter('odom_linear_direction', 1.0)
        self.declare_parameter('left_motor_direction', -1.0)
        self.declare_parameter('right_motor_direction', -1.0)
        self.declare_parameter('angular_command_direction', 1.0)
        self.declare_parameter('wheel_radius', WHEEL_RADIUS)
        self.declare_parameter('wheel_base', WHEEL_BASE)
        self.declare_parameter('imu_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_odom_tf', True)
        
        #打开传输通道
        self._sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self._sock.setblocking(False)

        self._host=self.get_parameter("host").value
        self._port=self.get_parameter("port").value
        self._bind_host = self.get_parameter("bind_host").value
        self._sock.bind((self._bind_host,self._port))
        self._sock.connect((self._host,self._port))
        # 启动心跳：告诉 ESP32 PC 的 IP，否则 ESP32 不发编码器，PID 永远不跑
        self._sock.send(b'M 0 0\n')

        #订阅/发布话题
        self._cmd_sub = self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._on_cmd, 10)
        self._odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self._imu_pub = self.create_publisher(Imu, '/imu', 20)
        self._joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        self._tf_br     = TransformBroadcaster(self)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._prev_left_enc  = 0
        self._prev_right_enc = 0
        self._enc_ready = False      # 第一次读到编码器只初始化，不算增量
        self._encoder_boot_id = None
        self._prev_encoder_sequence = None
        self._prev_sample_ms = None
        self._prev_receive_time = self.get_clock().now()
        self._last_encoder_time = self.get_clock().now()
        self._last_pwm = (0, 0)
        self._target_left = 0.0
        self._target_right = 0.0
        self._turning = False
        self._last_cmd_time = self.get_clock().now()
        self.declare_parameter('pid_kp', 10.0)
        self.declare_parameter('pid_ki', 2.0)
        self.declare_parameter('pid_kd', 0.0)
        self.declare_parameter('pid_integ_max', 100.0)
        self.declare_parameter('motor_min_pwm', 180)
        self.declare_parameter('motor_max_pwm', 230)
        self.declare_parameter('motor_turn_min_pwm', 210)
        self.declare_parameter('motor_turn_max_pwm', 250)
        self.declare_parameter('wheel_target_deadband', 0.5)

        kp = self.get_parameter('pid_kp').value
        ki = self.get_parameter('pid_ki').value
        kd = self.get_parameter('pid_kd').value
        imax = self.get_parameter('pid_integ_max').value
        self._pid_left = PID(kp, ki, kd, PWM_MAX, imax)
        self._pid_right = PID(kp, ki, kd, PWM_MAX, imax)

        # 启动定时轮询 
        self._timer = self.create_timer(0.02, self._read_socket)  # 50 Hz
        # 固件重启后也要重新注册回传地址；遥测中断时只发停车命令。
        self._heartbeat_timer = self.create_timer(0.1, self._send_heartbeat)
   
    def _on_cmd(self, msg: Twist):
        v = msg.linear.x  # m/s
        # 遵循 ROS REP-103：+angular.z 对应从上方看逆时针（左转）。
        w = (msg.angular.z *
             self.get_parameter('angular_command_direction').value)  # rad/s
        self._turning = not math.isclose(msg.angular.z, 0.0, abs_tol=1e-6)

        # 差速模型: v_l = (2v - w*L) / 2R,  v_r = (2v + w*L) / 2R
        wheel_sep = self.get_parameter('wheel_base').value
        r         = self.get_parameter('wheel_radius').value
        v_left  = (2 * v - w * wheel_sep) / (2 * r)   # rad/s, 轮端角速度
        v_right = (2 * v + w * wheel_sep) / (2 * r)

        # Nav2 接近目标时会给出很小的非零速度。对有较大机械死区的
        # 电机，强行把这种命令提升到最小 PWM 会直接冲过目标。
        deadband = max(0.0, float(
            self.get_parameter('wheel_target_deadband').value))
        if abs(v_left) < deadband:
            v_left = 0.0
        if abs(v_right) < deadband:
            v_right = 0.0

        # 键盘命令会在前进、原地转向和后退之间离散切换。目标变化时
        # 必须清掉旧积分，否则前进积累的积分会在按 J/L 后继续驱动车轮，
        # 造成延迟响应甚至突然加速。
        # 速度平滑器会持续改变目标幅值，不能每次都清积分；只在停车或
        # 换向时复位，避免控制器退化成始终输出最小 PWM。
        left_reversing = v_left * self._target_left < 0.0
        right_reversing = v_right * self._target_right < 0.0
        stopping = math.isclose(v_left, 0.0, abs_tol=1e-6) and \
            math.isclose(v_right, 0.0, abs_tol=1e-6)
        if left_reversing or right_reversing or stopping:
            self._pid_left.reset()
            self._pid_right.reset()

        self._target_left = v_left
        self._target_right = v_right
        self._last_cmd_time = self.get_clock().now()
        if stopping:
            # 不等待下一帧编码器，零速命令立即送到 ESP32。
            self._send_motor(0, 0)
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
        """读取本轮所有已到达的 UDP 数据报，防止 IMU 挤掉编码器数据。"""
        while True:
            ready, _, _ = select.select([self._sock], [], [], 0)
            if not ready:
                return
            try:
                data = self._sock.recv(1024).decode(errors="ignore")
            except BlockingIOError:
                return
            for line in data.split("\n"):
                line = line.strip()
                if not line:
                    continue
                if line.startswith('E '):
                    self._on_encoder(line)
                elif line.startswith('I '):
                    self._on_imu(line)

    def _send_motor(self, left: int, right: int):
        left_direction = self.get_parameter('left_motor_direction').value
        right_direction = self.get_parameter('right_motor_direction').value
        physical_left = int(left * left_direction)
        physical_right = int(right * right_direction)
        self._last_pwm = (physical_left, physical_right)
        line = f'M {physical_left} {physical_right}\n'
        self._sock.send(line.encode())

    def _apply_motor_limits(self, pwm: float, target: float,
                            measurement: float,
                            turning: bool = False) -> int:
        """Apply measured startup dead-zone and a conservative test limit."""
        if math.isclose(target, 0.0, abs_tol=1e-6):
            return 0

        if turning:
            min_pwm = int(self.get_parameter('motor_turn_min_pwm').value)
            max_pwm = int(self.get_parameter('motor_turn_max_pwm').value)
        else:
            min_pwm = int(self.get_parameter('motor_min_pwm').value)
            max_pwm = int(self.get_parameter('motor_max_pwm').value)
        min_pwm = max(0, min(PWM_MAX, min_pwm))
        max_pwm = max(min_pwm, min(PWM_MAX, max_pwm))

        # 已达到或超过目标轮速时必须撤掉驱动力。旧逻辑会在 PID 要求
        # 减速时仍强制输出同方向最小 PWM，导致车辆越过目标还在加速。
        direction = 1 if target > 0.0 else -1
        moving_in_target_direction = measurement * direction > 0.0
        if (moving_in_target_direction and
                abs(measurement) >= abs(target)):
            return 0
        if pwm * direction <= 0.0:
            return 0

        # 只有实际轮速低于目标、确实需要驱动时，才补偿机械死区。
        if abs(pwm) < min_pwm:
            pwm = direction * min_pwm
        return int(max(-max_pwm, min(max_pwm, pwm)))

    def _send_heartbeat(self):
        now = self.get_clock().now()
        if (now - self._last_encoder_time).nanoseconds / 1e9 > 0.2:
            self._target_left = 0.0
            self._target_right = 0.0
            self._turning = False
            self._pid_left.reset()
            self._pid_right.reset()
            self._last_pwm = (0, 0)
        left, right = self._last_pwm
        self._sock.send(f'M {left} {right}\n'.encode())

    @staticmethod
    def _int32_delta(current: int, previous: int) -> int:
        """计算有符号 32 位累计计数器跨越溢出点后的增量。"""
        return (current - previous + (1 << 31)) % (1 << 32) - (1 << 31)
    
    
    def _on_encoder(self, line: str):
        """解析编码器数据，计算里程计."""
        now = self.get_clock().now()
        parts = line.split()
        boot_id = sequence = sample_ms = None
        try:
            if len(parts) == 6:
                boot_id = int(parts[1], 16)
                sequence = int(parts[2]) & 0xffffffff
                sample_ms = int(parts[3]) & 0xffffffff
                left, right = int(parts[4]), int(parts[5])
            elif len(parts) == 3:
                left, right = int(parts[1]), int(parts[2])
            else:
                return
        except ValueError:
            self.get_logger().warn(f'Invalid encoder packet: {line}')
            return

        if boot_id is not None:
            if self._encoder_boot_id != boot_id:
                self._encoder_boot_id = boot_id
                self._enc_ready = False
                self._pid_left.reset()
                self._pid_right.reset()
                self._last_pwm = (0, 0)
            elif self._prev_encoder_sequence is not None:
                sequence_delta = (
                    sequence - self._prev_encoder_sequence) & 0xffffffff
                if sequence_delta == 0 or sequence_delta >= 0x80000000:
                    return

        self._last_encoder_time = now

        cpr = self.get_parameter('encoder_cpr').value
        left_direction = self.get_parameter('left_encoder_direction').value
        right_direction = self.get_parameter('right_encoder_direction').value
        r   = self.get_parameter('wheel_radius').value
        L   = self.get_parameter('wheel_base').value

        # Publish measured wheel angles so robot_state_publisher can maintain
        # base_link -> left_wheel/right_wheel TF for the RViz RobotModel.
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [
            math.fmod(left_direction * left / cpr * 2.0 * math.pi,
                      2.0 * math.pi),
            math.fmod(right_direction * right / cpr * 2.0 * math.pi,
                      2.0 * math.pi),
        ]
        self._joint_state_pub.publish(joint_state)

        if not self._enc_ready:
            # 第一次读数：初始化基准值，跳过增量计算
            self._prev_left_enc  = left
            self._prev_right_enc = right
            self._prev_receive_time = now
            self._prev_sample_ms = sample_ms
            self._prev_encoder_sequence = sequence
            self._enc_ready = True
            return

        d_left = (left_direction *
                  self._int32_delta(left, self._prev_left_enc) /
                  cpr * 2 * math.pi)
        d_right = (right_direction *
                   self._int32_delta(right, self._prev_right_enc) /
                   cpr * 2 * math.pi)
        self._prev_left_enc  = left
        self._prev_right_enc = right

        if sample_ms is not None and self._prev_sample_ms is not None:
            dt = ((sample_ms - self._prev_sample_ms) & 0xffffffff) / 1000.0
        else:
            dt = (now - self._prev_receive_time).nanoseconds / 1e9
        self._prev_sample_ms = sample_ms
        self._prev_encoder_sequence = sequence
        self._prev_receive_time = now

        if dt <= 0.0 or dt > 2.0:
            self._pid_left.reset()
            self._pid_right.reset()
            self._send_motor(0, 0)
            return

        d_dist  = ((d_right + d_left) / 2.0 * r *
                   self.get_parameter('odom_linear_direction').value)
        d_yaw   = (d_right - d_left) / L * r

        # 更新全局位姿
        mid_yaw = self._odom_yaw + d_yaw / 2.0
        self._odom_x += d_dist * math.cos(mid_yaw)
        self._odom_y += d_dist * math.sin(mid_yaw)
        self._odom_yaw += d_yaw

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
        if self.get_parameter('publish_odom_tf').value:
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
                self._turning = False
                
            actual_left=d_left/dt
            actual_right=d_right/dt

            # 停车检测：目标为0且实际轮速接近0时，直接发0并复位PID
            # 这样能消除积分残留和编码器噪声导致的微动/蜂鸣
            STOP_THRESH = 0.5  # rad/s, 低于此值视为静止
            if self._target_left == 0.0 and self._target_right == 0.0 \
               and abs(actual_left) < STOP_THRESH and abs(actual_right) < STOP_THRESH:
                self._pid_left.reset()
                self._pid_right.reset()
                self._send_motor(0, 0)
                return

            pwm_left = self._apply_motor_limits(
                self._pid_left.step(self._target_left, actual_left, dt),
                self._target_left,
                actual_left,
                self._turning)
            pwm_right = self._apply_motor_limits(
                self._pid_right.step(self._target_right, actual_right, dt),
                self._target_right,
                actual_right,
                self._turning)

            self._send_motor(pwm_left, pwm_right)


    def _on_imu(self, line: str):
        """发布 ESP32 的原始 IMU 数据。

        协议单位必须是 m/s² 和 rad/s：
        I <ax> <ay> <az> <gx> <gy> <gz>
        """
        parts = line.split()
        if len(parts) != 7:
            self.get_logger().warn(f'Invalid IMU packet: {line}')
            return
        try:
            ax, ay, az, gx, gy, gz = (float(value) for value in parts[1:])
        except ValueError:
            self.get_logger().warn(f'Invalid IMU values: {line}')
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.get_parameter('imu_frame').value
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.0004
        imu.linear_acceleration_covariance[0] = 0.25
        imu.linear_acceleration_covariance[4] = 0.25
        imu.linear_acceleration_covariance[8] = 0.25
        # ESP32 当前只上传加速度和角速度；未估计姿态。
        imu.orientation_covariance[0] = -1.0
        self._imu_pub.publish(imu)

    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        try:
            self._sock.send(b'M 0 0\n')
            self._sock.close()
        except Exception:
            pass
        try:
            super().destroy_node()
        except KeyboardInterrupt:
            # ros2 launch 已在退出时，第二个 SIGINT 不应把正常
            # 的停车/销毁流程报成节点异常。
            pass

def main():
    rclpy.init()
    node = WifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 在收到 SIGINT 时可能已经关闭默认 context；重复 shutdown
        # 会抛出 RCLError，掩盖正常的停车与退出流程。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()


    





        


        
