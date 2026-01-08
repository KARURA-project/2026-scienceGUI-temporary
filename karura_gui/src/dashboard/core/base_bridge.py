# src/karura_dash/core/base_bridge.py
from typing import Type

import rclpy
from PySide6.QtCore import QObject, Signal

from .ros2_worker import ROS2Worker
from dashboard.backend.base_node import BaseDashboardNode


class BaseROS2Bridge(QObject):
    ros_error = Signal(str)

    def __init__(self, node_cls: Type[BaseDashboardNode], node_name: str):
        super().__init__()

        rclpy.init(args=None)
        self.node = node_cls(node_name)
        self.worker = ROS2Worker(self.node)

        self.worker.error.connect(self.ros_error)

    def start(self):
        self.worker.start()

    def shutdown(self):
        self.worker.stop()
        self.worker.wait()
        self.node.destroy_node()
        rclpy.shutdown()
