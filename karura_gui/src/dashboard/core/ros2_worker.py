import traceback
import rclpy
from PySide6.QtCore import QThread, Signal

class ROS2Worker(QThread):
    error = Signal(str)        
    finished_ok = Signal()     

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self._running = True

    def run(self):
        try:
            while self._running and rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except Exception:
            msg = traceback.format_exc()
            print(f"ROS2Worker encountered an error:\n{msg}")
            self.error.emit(msg)
        finally:
            self.finished_ok.emit()

    def stop(self):
        self._running = False
        self.wait()
