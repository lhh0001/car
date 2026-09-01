#!/usr/bin/env python3
"""
Real-time ROS2 graph monitor — shows active nodes and topics in a Qt GUI.

Usage:
  ros2 run vehicle_control graph_monitor.py
  ros2 run vehicle_control graph_monitor.py --ros-args -p refresh_rate:=2.0
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QTreeWidget, QTreeWidgetItem, QLabel, QPushButton, QCheckBox,
        QSplitter, QHeaderView,
    )
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtGui import QFont, QColor, QBrush
    _QT_OK = True
except ImportError:
    _QT_OK = False


# ─────────────────────────────── ROS2 backend ───────────────────────────────

class GraphMonitor(Node):
    """Polls the ROS2 graph and emits signal-like callbacks with fresh data."""

    def __init__(self):
        super().__init__('graph_monitor')
        self.declare_parameter('refresh_rate', 1.0)  # Hz

        # Cache to detect changes
        self._prev_nodes: set[str] = set()
        self._prev_topics: dict[str, list[str]] = {}

    @property
    def interval_ms(self) -> int:
        return int(1000.0 / self.get_parameter('refresh_rate').value)

    def poll(self) -> tuple[list[tuple[str, str, str]], list[tuple[str, list[str]]]]:
        """
        Returns (nodes, topics) where:
          nodes  = [(name, namespace, status), ...]
          topics = [(name, [type1, type2, ...]), ...]
        """
        node_info = self.get_node_names_and_namespaces()
        nodes = []
        for full_name, ns in node_info:
            # full_name includes namespace prefix, e.g. /namespace/node_name
            nodes.append((full_name, ns, 'running'))

        topic_info = self.get_topic_names_and_types()
        topics = [(name, sorted(types)) for name, types in topic_info]

        # Detect new/removed nodes
        current_names = {n[0] for n in nodes}
        new_nodes = current_names - self._prev_nodes
        gone_nodes = self._prev_nodes - current_names
        self._prev_nodes = current_names

        # Detect new/removed topics
        current_topics = {t[0]: t[1] for t in topics}
        new_topics = set(current_topics) - set(self._prev_topics)
        gone_topics = set(self._prev_topics) - set(current_topics)
        self._prev_topics = current_topics

        changes = {
            'new_nodes': new_nodes,
            'gone_nodes': gone_nodes,
            'new_topics': new_topics,
            'gone_topics': gone_topics,
        }
        return nodes, topics, changes


# ─────────────────────────────── Qt GUI ─────────────────────────────────────

class _MonitorWindow(QMainWindow):
    """Main window: split pane with nodes on top, topics on bottom."""

    def __init__(self, ros_node: GraphMonitor):
        super().__init__()
        self._ros = ros_node
        self._nodes: list[tuple[str, str, str]] = []
        self._topics: list[tuple[str, list[str]]] = []

        self.setWindowTitle('ROS2 Graph Monitor')
        self.resize(800, 500)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)

        # ── Toolbar ──
        toolbar = QHBoxLayout()
        title = QLabel('<b>Nodes &amp; Topics</b>')
        title.setFont(QFont('', 12))
        toolbar.addWidget(title)
        toolbar.addStretch()

        self._auto_cb = QCheckBox('Auto-refresh')
        self._auto_cb.setChecked(True)
        self._auto_cb.toggled.connect(self._on_auto_toggle)
        toolbar.addWidget(self._auto_cb)

        refresh_btn = QPushButton('Refresh Now')
        refresh_btn.clicked.connect(self._poll)
        toolbar.addWidget(refresh_btn)

        count_label = QLabel('')
        self._count_label = count_label
        toolbar.addWidget(count_label)
        root_layout.addLayout(toolbar)

        # ── Splitter (nodes / topics) ──
        splitter = QSplitter(Qt.Vertical)

        self._node_tree = self._make_tree(['Node', 'Namespace', 'Status'])
        splitter.addWidget(self._node_tree)

        self._topic_tree = self._make_tree(['Topic', 'Type(s)'])
        splitter.addWidget(self._topic_tree)

        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        root_layout.addWidget(splitter)

        # ── Timer ──
        self._timer = QTimer()
        self._timer.timeout.connect(self._poll)
        self._start_timer()

        self._poll()  # initial load

    # ── helpers ──────────────────────────────────────────────────────────

    def _make_tree(self, headers: list[str]) -> QTreeWidget:
        tree = QTreeWidget()
        tree.setHeaderLabels(headers)
        tree.setAlternatingRowColors(True)
        tree.setRootIsDecorated(False)
        header = tree.header()
        header.setStretchLastSection(True)
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        return tree

    def _start_timer(self):
        if self._auto_cb.isChecked():
            self._timer.start(self._ros.interval_ms)

    def _on_auto_toggle(self, checked: bool):
        if checked:
            self._timer.start(self._ros.interval_ms)
        else:
            self._timer.stop()

    # ── data fetch ───────────────────────────────────────────────────────

    def _poll(self):
        try:
            nodes, topics, changes = self._ros.poll()
        except Exception:
            return  # ROS not ready yet

        self._nodes = nodes
        self._topics = topics

        self._rebuild_nodes(changes)
        self._rebuild_topics(changes)
        self._count_label.setText(
            f'{len(nodes)} nodes · {len(topics)} topics'
        )

    def _rebuild_nodes(self, changes: dict):
        tree = self._node_tree
        if tree.topLevelItemCount() != len(self._nodes):
            tree.clear()
            for name, ns, status in sorted(self._nodes):
                item = QTreeWidgetItem([name, ns, status])
                if name in changes['new_nodes']:
                    for c in range(3):
                        item.setBackground(c, QBrush(QColor('#d4edda')))  # green tint
                tree.addTopLevelItem(item)

    def _rebuild_topics(self, changes: dict):
        tree = self._topic_tree
        if tree.topLevelItemCount() != len(self._topics):
            tree.clear()
            for name, types in sorted(self._topics):
                item = QTreeWidgetItem([name, ', '.join(types)])
                if name in changes['new_topics']:
                    for c in range(2):
                        item.setBackground(c, QBrush(QColor('#d4edda')))
                tree.addTopLevelItem(item)


# ─────────────────────────────── main ───────────────────────────────────────

def main():
    if not _QT_OK:
        print('[ERROR] PyQt5 not installed.  Install with:')
        print('        sudo apt install python3-pyqt5')
        return

    rclpy.init()
    ros_node = GraphMonitor()

    app = QApplication([])
    window = _MonitorWindow(ros_node)
    window.show()

    # Qt + ROS2 cooperative event loop
    def _ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.0)

    timer = QTimer()
    timer.timeout.connect(_ros_spin)
    timer.start(50)  # spin ROS at 20 Hz

    try:
        app.exec()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
