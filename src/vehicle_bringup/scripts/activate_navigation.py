#!/usr/bin/env python3
"""
Sequentially activate navigation nodes for saved-map navigation.

Also acts as a /map relay: map_server publishes once with transient_local,
but late subscribers (costmap static_layer) often miss it. This script
re-publishes to /map_continuous at 1 Hz so the costmap never misses the map.

Activation order:
  1. configure + activate map_server
  2. wait for /map
  3. configure + activate AMCL
  4. configure + activate Nav2 nodes (dependency order)
  5. keep running as map relay
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav_msgs.msg import OccupancyGrid


class NavActivator(Node):
    """Sequence-activates lifecycle nodes, then runs as map relay."""

    def __init__(self):
        super().__init__('nav_activator')
        self._last_map = None

        _map_qos = QoSProfile(depth=1)

        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, _map_qos)

        self._map_pub = self.create_publisher(
            OccupancyGrid, '/map', _map_qos)
        self._map_timer = self.create_timer(1.0, self._publish_map)

    def _on_map(self, msg: OccupancyGrid):
        self._last_map = msg
        self.get_logger().info('Received /map from map_server')

    def _publish_map(self):
        if self._last_map is not None:
            self._map_pub.publish(self._last_map)

    def _activate(self, name: str, timeout: float = 10.0) -> bool:
        srv_name = f'/{name}/change_state'
        cli = self.create_client(ChangeState, srv_name)
        self.get_logger().info(f'Waiting for service {srv_name}...')
        if not cli.wait_for_service(timeout):
            self.get_logger().error(f'{srv_name} not ready after {timeout}s')
            return False
        self.get_logger().info(f'Service {srv_name} ready')

        # configure (transition 1): unconfigured → inactive
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_CONFIGURE
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result() or not fut.result().success:
            self.get_logger().error(f'{name} configure FAILED')
            return False
        self.get_logger().info(f'{name} configured OK')

        # activate (transition 3): inactive → active
        req.transition.id = Transition.TRANSITION_ACTIVATE
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if not fut.result() or not fut.result().success:
            self.get_logger().error(f'{name} activate FAILED')
            return False
        self.get_logger().info(f'{name} activated OK')
        return True

    def run(self):
        # Step 1: activate map_server
        if not self._activate('map_server'):
            return
        self.get_logger().info('map_server activated, waiting for /map...')

        # Step 2: wait for /map (max 15s)
        waited = 0
        while self._last_map is None and waited < 15:
            rclpy.spin_once(self, timeout_sec=1.0)
            waited += 1
            if waited % 3 == 0:
                self.get_logger().info(f'  waiting for /map... {waited}s')

        if self._last_map is None:
            self.get_logger().error('/map not received after 15s')
            return

        self.get_logger().info('/map received — relay to /map started (1 Hz)')

        # Step 3: activate AMCL
        if not self._activate('amcl'):
            return

        # Step 4: activate Nav2 nodes in dependency order
        # bt_navigator needs behavior/planner/controller active FIRST
        nav2_nodes = [
            'behavior_server',
            'planner_server',
            'controller_server',
            'bt_navigator',
            'velocity_smoother',
        ]
        for node_name in nav2_nodes:
            self.get_logger().info(f'Activating {node_name}...')
            if not self._activate(node_name):
                self.get_logger().error(f'STOPPED at {node_name} — check logs above')
                return

        self.get_logger().info('All navigation nodes activated — relay running...')

        # Keep running as map relay
        rclpy.spin(self)


def main():
    rclpy.init()
    node = NavActivator()
    try:
        node.run()
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
