#!/usr/bin/env python3
"""Fuse wheel linear velocity with IMU yaw rate for planar odometry."""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class OdomImuFusion(Node):
    def __init__(self):
        super().__init__('odom_imu_fusion')
        self.declare_parameter('wheel_odom_topic', '/wheel/odom')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('imu_weight', 0.9)
        self.declare_parameter('imu_timeout', 0.15)
        self.declare_parameter('max_imu_yaw_rate', 3.5)
        self.declare_parameter('bias_samples', 100)
        self.declare_parameter('stationary_linear_threshold', 0.01)
        self.declare_parameter('stationary_angular_threshold', 0.05)

        self._pub = self.create_publisher(
            Odometry, self.get_parameter('output_odom_topic').value, 20)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter('wheel_odom_topic').value,
            self._on_wheel_odom, 50)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._on_imu, 100)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_wheel_stamp = None
        self._last_imu_time = None
        self._imu_gz = 0.0
        self._gyro_bias = 0.0
        self._bias_values = []
        self._bias_ready = False
        self._wheel_v = 0.0
        self._wheel_w = 0.0

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _quat(yaw):
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    def _on_imu(self, msg):
        gz = float(msg.angular_velocity.z)
        max_rate = float(self.get_parameter('max_imu_yaw_rate').value)
        if not math.isfinite(gz) or abs(gz) > max_rate:
            self.get_logger().warning(
                f'Rejected IMU yaw-rate spike: {gz:.3f} rad/s',
                throttle_duration_sec=2.0)
            return

        self._imu_gz = gz
        self._last_imu_time = self.get_clock().now()

        if not self._bias_ready:
            self._bias_values.append(gz)
            required = int(self.get_parameter('bias_samples').value)
            if len(self._bias_values) >= required:
                self._gyro_bias = sum(self._bias_values) / len(self._bias_values)
                self._bias_ready = True
                self.get_logger().info(
                    f'IMU yaw-rate bias calibrated: {self._gyro_bias:.6f} rad/s')
            return

        linear_limit = float(
            self.get_parameter('stationary_linear_threshold').value)
        angular_limit = float(
            self.get_parameter('stationary_angular_threshold').value)
        if (abs(self._wheel_v) < linear_limit and
                abs(self._wheel_w) < angular_limit and
                abs(gz - self._gyro_bias) < angular_limit):
            self._gyro_bias = 0.999 * self._gyro_bias + 0.001 * gz

    def _on_wheel_odom(self, msg):
        stamp = self._stamp_seconds(msg.header.stamp)
        if self._last_wheel_stamp is None:
            self._last_wheel_stamp = stamp
            return
        dt = stamp - self._last_wheel_stamp
        self._last_wheel_stamp = stamp
        if dt <= 0.0 or dt > 0.5:
            return

        self._wheel_v = float(msg.twist.twist.linear.x)
        self._wheel_w = float(msg.twist.twist.angular.z)
        fused_w = self._wheel_w

        imu_fresh = False
        if self._last_imu_time is not None and self._bias_ready:
            age = (self.get_clock().now() - self._last_imu_time).nanoseconds * 1e-9
            imu_fresh = age <= float(self.get_parameter('imu_timeout').value)
        if imu_fresh:
            imu_w = self._imu_gz - self._gyro_bias
            weight = min(1.0, max(0.0,
                float(self.get_parameter('imu_weight').value)))
            fused_w = weight * imu_w + (1.0 - weight) * self._wheel_w

        d_yaw = fused_w * dt
        mid_yaw = self._yaw + 0.5 * d_yaw
        self._x += self._wheel_v * dt * math.cos(mid_yaw)
        self._y += self._wheel_v * dt * math.sin(mid_yaw)
        self._yaw += d_yaw
        q = self._quat(self._yaw)

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.pose.pose.position.x = self._x
        out.pose.pose.position.y = self._y
        out.pose.pose.orientation = q
        out.twist.twist.linear.x = self._wheel_v
        out.twist.twist.angular.z = fused_w
        out.pose.covariance[0] = 0.02 ** 2
        out.pose.covariance[7] = 0.02 ** 2
        out.pose.covariance[35] = 0.03 ** 2
        out.twist.covariance[0] = 0.03 ** 2
        out.twist.covariance[35] = 0.05 ** 2
        self._pub.publish(out)

        tf = TransformStamped()
        tf.header = out.header
        tf.child_frame_id = out.child_frame_id
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation = q
        self._tf.sendTransform(tf)


def main():
    rclpy.init()
    node = OdomImuFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
