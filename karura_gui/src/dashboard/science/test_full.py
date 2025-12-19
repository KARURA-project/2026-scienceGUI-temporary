"""
Test Science Dashboard UI (No ROS 2 Required)

This script tests the Science Dashboard UI components without requiring ROS 2.
It creates mock data to demonstrate the interface functionality.
"""

import sys
import os
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer

# Add the project root to Python path
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, project_root)

try:
    from dashboard.science import ScienceMainWindow
    # For testing without ROS 2, we'll create mock message classes
    class MockFloat64:
        def __init__(self, data=0.0):
            self.data = data

    class MockInt:
        def __init__(self, data=0):
            self.data = data

    class MockNavSatFix:
        def __init__(self):
            self.latitude = 0.0
            self.longitude = 0.0
            self.altitude = 0.0

    Float64 = MockFloat64
    Int = MockInt
    NavSatFix = MockNavSatFix

    import numpy as np
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
        imu_msg = type('MockFloat64MultiArray', (), {
            'data': [
                np.random.normal(0, 5),  # roll
                np.random.normal(0, 3),  # pitch
                np.random.normal(0, 10)  # yaw
            ]
        })()
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
        window.show()
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