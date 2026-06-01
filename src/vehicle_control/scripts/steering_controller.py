#!/usr/bin/env python3
"""
Ackermann steering controller: converts /cmd_vel (Twist) to front-wheel steering angles.

Architecture:
  /cmd_vel (Twist) --> [bicycle model + rate limiter] --> /steering_controller/commands

All tunable parameters are ROS2 parameters (modifiable at runtime):
  ros2 run vehicle_control steering_controller.py --ros-args -p wheelbase:=2.8
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class SteeringController(Node):
    """Proportional Ackermann steering controller with rate limiting and watchdog."""

    def __init__(self):
        super().__init__('steering_controller_node')

        # ---- tunable parameters (change via --ros-args or YAML) ----
        self.declare_parameter('wheelbase', 2.64)           # m
        self.declare_parameter('max_steer_angle', 0.6)      # rad (mechanical limit)
        self.declare_parameter('deadband', 0.02)             # m/s & rad/s
        self.declare_parameter('max_steer_rate', 1.0)        # rad/s (smoothing)
        self.declare_parameter('cmd_timeout', 0.5)           # s (watchdog)
        self.declare_parameter('publish_rate', 50.0)         # Hz

        # ---- state ----
        self._current_steer = 0.0
        self._last_time = self.get_clock().now()
        self._last_cmd_time = self.get_clock().now()
        self._stopped = True

        # ---- pubs & subs ----
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', 10)

        # periodic publish — ensures zero command on startup / timeout
        period = 1.0 / self.get_parameter('publish_rate').value
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            'Steering controller ready | '
            f'wheelbase={self._p("wheelbase"):.2f}m '
            f'max_steer={self._p("max_steer_angle"):.2f}rad '
            f'deadband={self._p("deadband"):.3f}'
        )

    def _p(self, name: str) -> float:
        """Shortcut: get current parameter value."""
        return self.get_parameter(name).value

    # ------------------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist):
        """Process incoming Twist command."""
        if not self._is_finite(msg):
            self.get_logger().warn(
                'NaN/Inf cmd_vel — ignoring', throttle_duration_sec=1.0)
            return

        v = msg.linear.x
        w = msg.angular.z
        self._last_cmd_time = self.get_clock().now()

        target = self._compute_steer(v, w)
        target = self._rate_limit(target)
        self._current_steer = target
        self._stopped = (abs(v) < self._p('deadband') and
                         abs(w) < self._p('deadband'))

    def _compute_steer(self, v: float, w: float) -> float:
        """Bicycle model: tan(delta) = L * ω / v."""
        deadband = self._p('deadband')
        max_steer = self._p('max_steer_angle')

        if abs(v) < deadband:
            return (max_steer if w > deadband
                    else -max_steer if w < -deadband
                    else 0.0)

        curvature = self._p('wheelbase') * w / v
        target = math.atan(curvature)
        return max(-max_steer, min(max_steer, target))

    def _rate_limit(self, target: float) -> float:
        """Smooth steering via rate-of-change limit."""
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds / 1e9
        self._last_time = now

        if not (0.0 < dt < 0.5):
            return target

        max_step = self._p('max_steer_rate') * dt
        diff = target - self._current_steer
        return self._current_steer + max(-max_step, min(max_step, diff))

    def _publish(self):
        """Periodic publish with watchdog — zeroes steering on command timeout."""
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if dt > self._p('cmd_timeout') and not self._stopped:
            self._current_steer = 0.0
            self._stopped = True
            self.get_logger().debug('Cmd timeout — steering zeroed')

        cmd = Float64MultiArray()
        cmd.data = [self._current_steer, self._current_steer]
        self._pub.publish(cmd)

    @staticmethod
    def _is_finite(msg: Twist) -> bool:
        return all(math.isfinite(v) for v in (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        ))


def main():
    rclpy.init()
    node = SteeringController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt — shutting down')
    except Exception:
        node.get_logger().exception('Unexpected error')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
