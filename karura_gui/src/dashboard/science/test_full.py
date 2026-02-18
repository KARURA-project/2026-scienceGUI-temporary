"""
Test Science Dashboard UI (No ROS 2 Required)

This script tests the Science Dashboard UI components without requiring ROS 2.
It creates mock data to demonstrate the interface functionality.
"""

import sys
import os
import types
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

# Add the project root to Python path
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, project_root)

# Mock ROS 2 modules if they are missing (for testing without ROS environment)
try:
    import numpy as np
    import sensor_msgs.msg
    import std_msgs.msg
    
    # If imports succeed, alias them for the simulation loop
    Image = sensor_msgs.msg.Image
    NavSatFix = sensor_msgs.msg.NavSatFix
    Float64 = std_msgs.msg.Float64
    Float64MultiArray = std_msgs.msg.Float64MultiArray
    Int = std_msgs.msg.Int32
    
except ImportError:
    print("ROS 2 modules not found. Mocking them for UI testing...")
    import numpy as np
    
    # Create mock modules structure in sys.modules
    rclpy = types.ModuleType("rclpy")
    sys.modules["rclpy"] = rclpy
    
    rclpy_node = types.ModuleType("rclpy.node")
    sys.modules["rclpy.node"] = rclpy_node
    rclpy.node = rclpy_node
    
    class MockNode:
        def __init__(self, *args, **kwargs): pass
        def destroy_node(self): pass
        def create_subscription(self, *args, **kwargs): pass
        def create_publisher(self, *args, **kwargs): pass
    
    rclpy.Node = MockNode
    rclpy.node.Node = MockNode
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.ok = lambda: True
    rclpy.spin_once = lambda *args, **kwargs: None

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg
    
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg
    
    # Define Mock Classes
    class MockImage:
        def __init__(self):
            self.encoding = "rgb8"
            self.height = 240
            self.width = 320
            self.step = 320 * 3
            self.data = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8).tobytes()
    
    class MockFloat64:
        def __init__(self, data=0.0): self.data = data
    class MockInt32:
        def __init__(self, data=0): self.data = data
    class MockBool:
        def __init__(self, data=False): self.data = data
    class MockString:
        def __init__(self, data=""): self.data = data
    class MockFloat64MultiArray:
        def __init__(self, data=None): self.data = data if data else []
    class MockNavSatFix:
        def __init__(self): self.latitude = 0.0; self.longitude = 0.0; self.altitude = 0.0

    # Assign to mock modules
    sensor_msgs.msg.Image = MockImage
    sensor_msgs.msg.NavSatFix = MockNavSatFix
    std_msgs.msg.Float64 = MockFloat64
    std_msgs.msg.Float64MultiArray = MockFloat64MultiArray
    std_msgs.msg.Int32 = MockInt32
    std_msgs.msg.Bool = MockBool
    std_msgs.msg.String = MockString
    
    # Aliases for simulation loop
    Image = MockImage
    NavSatFix = MockNavSatFix
    Float64 = MockFloat64
    Float64MultiArray = MockFloat64MultiArray
    Int = MockInt32

try:
    from dashboard.science import ScienceMainWindow
except ImportError as e:
    print(f"Import error: {e}")
    print(f"Project root: {project_root}")
    print(f"Python path: {sys.path[:3]}")
    sys.exit(1)


class MockScienceBridge:
    """Mock bridge that simulates ROS 2 data without actual ROS 2."""

    def __init__(self):
        # Mock signals (we'll simulate them with direct calls)
        pass

    def start(self):
        """Mock start - does nothing."""
        pass

    def shutdown(self):
        """Mock shutdown - does nothing."""
        pass

    def simulate_data(self, window):
        """Simulate incoming ROS 2 data by directly calling window update methods."""

        # Mock Camera data (Random Noise)
        img_msg = Image()
        window.on_downward_cam_update(img_msg)
        window.on_panoramic_cam_update(img_msg)
        window.on_box_cam_update(img_msg)
        window.on_fluorescence_cam_update(img_msg)

        # Mock temperature data
        temp_msg = Float64()
        temp_msg.data = 25.0 + np.random.normal(0, 2)  # 25°C ± 2°C
        window.on_temperature_update(temp_msg)

        # Mock humidity data
        humidity_msg = Float64()
        humidity_msg.data = 60.0 + np.random.normal(0, 5)  # 60% ± 5%
        window.on_humidity_update(humidity_msg)

        # Mock pressure data
        pressure_msg = Float64()
        pressure_msg.data = 101.3 + np.random.normal(0, 0.5)  # 101.3 kPa ± 0.5
        window.on_pressure_update(pressure_msg)

        # Mock battery data
        battery_msg = Int()
        battery_msg.data = max(0, min(100, 85 + np.random.normal(0, 3)))  # ~85% ± 3%
        window.on_battery_update(battery_msg)

        # Mock IMU data (roll, pitch, yaw)
        imu_msg = Float64MultiArray()
        imu_msg.data = [
            np.random.normal(0, 5),  # roll
            np.random.normal(0, 3),  # pitch
            np.random.normal(0, 10)  # yaw
        ]
        window.on_imu_update(imu_msg)

        # Mock GPS data
        gps_msg = NavSatFix()
        gps_msg.latitude = 29.2108 + np.random.normal(0, 0.001)  # TAMU campus approx
        gps_msg.longitude = -98.1234 + np.random.normal(0, 0.001)
        gps_msg.altitude = 100.0 + np.random.normal(0, 2)
        window.on_gps_update(gps_msg)


def main():
    """Test the Science Dashboard with mock data."""
    print("Testing Science Dashboard UI...")

    # Create Qt application
    app = QApplication(sys.argv)

    try:
        # Create mock bridge and window
        mock_bridge = MockScienceBridge()
        window = ScienceMainWindow()

        # Connect mock bridge (though we won't use signals)
        # window.connect_signals(mock_bridge)

        # Start mock data simulation
        mock_bridge.start()

        # Set up timer to update data every 2 seconds
        timer = QTimer()
        timer.timeout.connect(lambda: mock_bridge.simulate_data(window))
        timer.start(2000)  # Update every 2 seconds

        # Show window
        window.move(100, 100)
        window.show()
        window.raise_()
        window.activateWindow()
        
        print(f"[DEBUG] Window visible: {window.isVisible()}")
        print(f"[DEBUG] Window geometry: {window.geometry()}")
        print("Science Dashboard opened successfully!")
        print("Mock data will update every 2 seconds.")
        print("Close the window to exit.")

        # Run application
        sys.exit(app.exec())

    except Exception as e:
        print(f"Error testing dashboard: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()