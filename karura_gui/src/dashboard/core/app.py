import sys
from PySide6.QtWidgets import QApplication

def run_dashboard():
    app = QApplication(sys.argv)
    # Initialize and show the main dashboard window here
    ros2_bridge = ROS2Bridge(node_name="karura_dashboard_bridge")
    window = window_cls(ros2_bridge)
    window.show()
    
    # Start the ROS2 bridge
    sys.exit(app.exec())