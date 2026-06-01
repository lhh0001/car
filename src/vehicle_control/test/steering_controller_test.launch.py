#!/usr/bin/env python3
"""
Integration test for the steering controller node.

Spawns the steering_controller, publishes test Twist messages,
and verifies the output Float64MultiArray commands are correct.
"""
import math
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing_ros


@pytest.mark.launch_test
def generate_test_description():
    """Launch the steering controller node under test."""
    return LaunchDescription([
        Node(
            package='vehicle_control',
            executable='steering_controller.py',
            name='test_steering_controller',
            parameters=[{
                'wheelbase': 2.64,
                'max_steer_angle': 0.6,
                'deadband': 0.02,
                'max_steer_rate': 100.0,  # fast rate limit — effectively disabled for testing
                'cmd_timeout': 10.0,       # long timeout
                'publish_rate': 100.0,
            }],
            output='screen',
        ),
        # Tell launch_testing to run the test class below
        launch_testing.actions.ReadyToTest(),
    ])


class TestSteeringController(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_client')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.received = []

        def _cb(msg: Float64MultiArray):
            self.received.append(msg.data)

        self.sub = self.node.create_subscription(
            Float64MultiArray, '/steering_controller/commands', _cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _spin_until(self, timeout: float = 1.0):
        """Spin until we have at least 1 received message."""
        import time
        start = time.time()
        while not self.received and (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_straight_line_zero_steer(self):
        """Moving straight forward should produce near-zero steer angle."""
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self._spin_until()

        self.assertTrue(len(self.received) > 0, 'No steering command received')
        steer_l, steer_r = self.received[-1]
        self.assertAlmostEqual(steer_l, 0.0, delta=0.01, msg='Straight line: left steer should be ~0')
        self.assertAlmostEqual(steer_r, 0.0, delta=0.01, msg='Straight line: right steer should be ~0')

    def test_pure_rotation_gives_max_steer(self):
        """Pure rotation (v=0, w>0) should give max positive steer."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.pub.publish(msg)
        self._spin_until()

        self.assertTrue(len(self.received) > 0, 'No steering command received')
        steer_l, steer_r = self.received[-1]
        expected = 0.6  # max_steer_angle
        self.assertAlmostEqual(steer_l, expected, delta=0.01)
        self.assertAlmostEqual(steer_r, expected, delta=0.01)

    def test_deadband_zero_output(self):
        """Below-threshold velocity should produce zero steer."""
        msg = Twist()
        msg.linear.x = 0.001  # below 0.02 deadband
        msg.angular.z = 0.001
        self.pub.publish(msg)
        self._spin_until()

        self.assertTrue(len(self.received) > 0)
        steer_l, steer_r = self.received[-1]
        self.assertAlmostEqual(steer_l, 0.0, delta=0.01)
        self.assertAlmostEqual(steer_r, 0.0, delta=0.01)

    def test_turning_steer_direction(self):
        """Positive angular vel should produce positive steer angle."""
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.3  # gentle turn
        self.pub.publish(msg)
        self._spin_until()

        self.assertTrue(len(self.received) > 0)
        steer_l, steer_r = self.received[-1]
        # Bicycle model: delta = atan(L * w / v) = atan(2.64 * 0.3 / 1.0) ≈ 0.67 rad
        # But limited to max_steer=0.6, so should be ~0.6
        self.assertGreater(steer_l, 0.0, 'Turning right should give positive steer')
        self.assertGreater(steer_r, 0.0, 'Both wheels steer same direction')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check that the node launched without errors."""
        launch_testing.asserts.assertExitCodes(proc_info)
