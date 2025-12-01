import rclpy
from PySide6.QtCore import QObject, Signal, QThread

class ROS2Worker(QThread): 
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self._running = True
        
    def run(self):
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception as e:
            print(f"ROS2Worker encountered an error: {e}")
    
    def stop(self):
        self._running = False
        self.wait()