#!/usr/bin/env python3
"""
Tkinter-based teleoperation GUI for ROS2 robots.

Usage:
  ros2 run vehicle_control teleop_gui.py
  ros2 run vehicle_control teleop_gui.py --ros-args -p max_linear:=5.0 -p max_angular:=3.0

Supports WASD and arrow keys. Release all keys to stop.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Tkinter is conditionally imported to allow headless testing
try:
    import tkinter as tk
    _TK_AVAILABLE = True
except ImportError:
    _TK_AVAILABLE = False


_MODIFIER_KEYS = frozenset({
    'Shift_L', 'Shift_R', 'Control_L', 'Control_R',
    'Alt_L', 'Alt_R', 'Caps_Lock', 'Num_Lock',
    'Super_L', 'Super_R', 'Meta_L', 'Meta_R',
})

_DRIVE_KEYS = frozenset({
    'w', 'up',      # forward
    's', 'down',    # backward
    'a', 'left',    # turn left
    'd', 'right',   # turn right
})


class TeleopGUI(Node):
    """Simple keyboard teleop node with Tkinter GUI."""

    def __init__(self):
        if not _TK_AVAILABLE:
            raise RuntimeError(
                'tkinter not available — install python3-tk or run headless')

        super().__init__('teleop_gui')

        # ---- parameters ----
        self.declare_parameter('max_linear', 2.0)    # m/s
        self.declare_parameter('max_angular', 2.0)   # rad/s
        self.declare_parameter('publish_rate', 50.0) # Hz
        self.declare_parameter('startup_delay', 0.5) # s

        self._pressed: set[str] = set()
        self._ready = False

        # Publisher
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Periodic publish timer
        period = 1.0 / self.get_parameter('publish_rate').value
        self._timer = self.create_timer(period, self._publish_cmd)

        # Delay startup to avoid spurious window-init events
        self._startup_timer = self.create_timer(
            self.get_parameter('startup_delay').value, self._enable)

        # ---- GUI ----
        self._build_gui()

        self.get_logger().info(
            f'Teleop GUI ready | '
            f'max_linear={self._p("max_linear"):.1f}m/s '
            f'max_angular={self._p("max_angular"):.1f}rad/s'
        )

    def _p(self, name: str) -> float:
        return self.get_parameter(name).value

    # ---- GUI construction ----
    def _build_gui(self):
        self._root = tk.Tk()
        self._root.title('Vehicle Teleop')

        tk.Label(
            self._root,
            text='WASD / Arrow Keys to drive\nRelease to stop',
            font=('', 14), pady=10,
        ).pack()

        self._status = tk.Label(
            self._root, text='STOPPED', font=('', 18, 'bold'), fg='red', pady=10)
        self._status.pack()

        self._speed_label = tk.Label(
            self._root, text='linear: 0.0  angular: 0.0', font=('', 12))
        self._speed_label.pack(pady=10)

        self._root.bind('<KeyPress>', self._on_press)
        self._root.bind('<KeyRelease>', self._on_release)
        self._root.protocol('WM_DELETE_WINDOW', self._on_close)

    def _enable(self):
        self._startup_timer.cancel()
        self._ready = True

    # ---- keyboard events ----
    def _on_press(self, event):
        key = event.keysym.lower()
        if key in _MODIFIER_KEYS or key not in _DRIVE_KEYS:
            return
        self._pressed.add(key)
        self._update_status()

    def _on_release(self, event):
        key = event.keysym.lower()
        if key in _MODIFIER_KEYS or key not in _DRIVE_KEYS:
            return
        self._pressed.discard(key)
        self._update_status()

    def _on_close(self):
        """Graceful shutdown on window close."""
        self.get_logger().info('Window closed — shutting down')
        self._root.destroy()

    # ---- command computation ----
    def _get_cmd(self) -> tuple[float, float]:
        linear, angular = 0.0, 0.0
        max_v = self._p('max_linear')
        max_w = self._p('max_angular')

        for key in self._pressed:
            if key in ('w', 'up'):
                linear += max_v
            if key in ('s', 'down'):
                linear -= max_v
            if key in ('a', 'left'):
                angular += max_w
            if key in ('d', 'right'):
                angular -= max_w

        return linear, angular

    def _update_status(self):
        linear, angular = self._get_cmd()
        moving = linear != 0.0 or angular != 0.0
        self._status.config(
            text='MOVING' if moving else 'STOPPED',
            fg='green' if moving else 'red',
        )
        self._speed_label.config(
            text=f'linear: {linear:.1f}  angular: {angular:.1f}'
        )

    def _publish_cmd(self):
        if not self._ready:
            return
        linear, angular = self._get_cmd()
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._pub.publish(msg)

    # ---- main loop ----
    def run(self):
        """Blocking Tk+ROS event loop."""
        self._root.after(10, self._ros_spin_once)
        self._root.mainloop()

    def _ros_spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        self._root.after(10, self._ros_spin_once)


def main():
    rclpy.init()
    gui = TeleopGUI()
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
