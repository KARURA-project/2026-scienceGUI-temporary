"""
Science Dashboard Custom Widgets

Reusable UI components for the Science Dashboard, including:
- Image/video display
- Sensor gauges
- Telemetry panels
- Data indicators
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QGridLayout,
    QPushButton, QSlider, QSpinBox, QDoubleSpinBox, QSizePolicy
)
from PySide6.QtCore import Qt, QSize, Signal
from PySide6.QtGui import QFont, QColor, QImage, QPixmap
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Float64, Int32, Bool, String
import numpy as np


class ImageDisplayWidget(QWidget):
    """
    Displays ROS Image messages as a pixmap.
    Handles conversion from ROS Image to QPixmap for display.
    """

    def __init__(self, title: str = "Camera Feed", parent=None):
        super().__init__(parent)
        self.title = title
        self._last_pixmap: QPixmap | None = None
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)  # <-- keeps title tight to image

        # Title label (aligned directly over image)
        self.title_label = QLabel(self.title)
        self.title_label.setFont(QFont("Arial", 12, QFont.Bold))
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignLeft)
        layout.addWidget(self.title_label)

        # Image display
        self.image_label = QLabel("NO SIGNAL")
        self.image_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.image_label.setStyleSheet("border: 1px solid #3A2426; background-color: #0F0B0C; color: #555555;")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # IMPORTANT: allow it to grow and fill the tab area
        self.image_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.image_label.setMinimumSize(1, 1)

        layout.addWidget(self.image_label, 1)  # <-- stretch

        self.setLayout(layout)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Re-scale last frame to new widget size
        if self._last_pixmap is not None:
            self._render_pixmap(self._last_pixmap)

    def _render_pixmap(self, pixmap: QPixmap):
        scaled = pixmap.scaled(
            self.image_label.size(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)

    def update_image(self, ros_image: ROSImage):
        """
        Convert ROS Image message to QPixmap and display.
        """
        try:
            if ros_image.encoding == "rgb8":
                data = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
                q_image = QImage(data.data, ros_image.width, ros_image.height, 3 * ros_image.width,
                                 QImage.Format.Format_RGB888)

            elif ros_image.encoding == "bgr8":
                data = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
                data = data[..., ::-1]  # BGR -> RGB
                q_image = QImage(data.data, ros_image.width, ros_image.height, 3 * ros_image.width,
                                 QImage.Format.Format_RGB888)

            elif ros_image.encoding == "mono8":
                data = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width))
                q_image = QImage(data.data, ros_image.width, ros_image.height, ros_image.width,
                                 QImage.Format.Format_Grayscale8)

            else:
                self.image_label.setText(f"Unsupported encoding: {ros_image.encoding}")
                return

            pixmap = QPixmap.fromImage(q_image)
            self._last_pixmap = pixmap
            self._render_pixmap(pixmap)

        except Exception as e:
            self.image_label.setText(f"Error displaying image: {e}")

class SensorGaugeWidget(QWidget):
    """
    Displays a single sensor value with unit and visual indicator.
    """

    def __init__(self, title: str, unit: str = "", min_val: float = 0,
                 max_val: float = 100, parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.min_val = min_val
        self.max_val = max_val
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout()

        title_label = QLabel(self.title)
        title_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(title_label)

        self.value_label = QLabel("-- --")
        self.value_label.setFont(QFont("Courier", 14, QFont.Bold))
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.value_label)

        self.status_frame = QFrame()
        self.status_frame.setMinimumHeight(10)
        self.status_frame.setStyleSheet("background-color: #cccccc; border-radius: 3px;")
        layout.addWidget(self.status_frame)

        self.setLayout(layout)

    def update_value(self, msg: Float64):
        """Update the displayed sensor value."""
        value = msg.data
        self.value_label.setText(f"{value:.2f} {self.unit}")

        if value < self.min_val:
            color = "orange"
        elif value > self.max_val:
            color = "red"
        else:
            color = "green"

        self.status_frame.setStyleSheet(f"background-color: {color}; border-radius: 3px;")


class TelemetryPanelWidget(QWidget):
    """
    Displays multiple telemetry values in a grid layout.
    """
    
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.data_labels = {}
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title_label = QLabel(self.title)
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title_label)
        
        # Grid for telemetry items
        self.grid_layout = QGridLayout()
        layout.addLayout(self.grid_layout)
        
        self.setLayout(layout)
    
    def add_telemetry_item(self, key: str, label: str, default_value: str = "-- --"):
        """Add a telemetry item to the panel."""
        label_widget = QLabel(label)
        label_widget.setFont(QFont("Arial", 10, QFont.Bold))
        
        value_widget = QLabel(default_value)
        value_widget.setFont(QFont("Courier", 10))
        
        row = len(self.data_labels)
        self.grid_layout.addWidget(label_widget, row, 0)
        self.grid_layout.addWidget(value_widget, row, 1)
        
        self.data_labels[key] = value_widget
    
    def update_telemetry(self, key: str, value: str):
        """Update a telemetry value."""
        if key in self.data_labels:
            self.data_labels[key].setText(value)


class BatteryIndicatorWidget(QWidget):
    """
    Displays battery percentage with a visual indicator.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title_label = QLabel("Battery")
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title_label)
        
        # Battery percentage
        self.percentage_label = QLabel("-- %")
        self.percentage_label.setFont(QFont("Arial", 16, QFont.Bold))
        self.percentage_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.percentage_label)
        
        # Battery bar
        self.battery_frame = QFrame()
        self.battery_frame.setMinimumHeight(20)
        self.battery_frame.setStyleSheet("background-color: #cccccc; border: 1px solid #666666; border-radius: 3px;")
        layout.addWidget(self.battery_frame)
        
        self.setLayout(layout)
    
    def update_battery(self, msg: Int32):
        """Update battery percentage."""
        percentage = max(0, min(100, msg.data))
        self.percentage_label.setText(f"{percentage}%")
        
        # Update bar color
        if percentage > 50:
            color = "green"
        elif percentage > 25:
            color = "orange"
        else:
            color = "red"
        
        width_percent = percentage
        self.battery_frame.setStyleSheet(
            f"background: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
            f"stop:0 {color}, stop:{width_percent/100} {color}, "
            f"stop:{width_percent/100} #cccccc, stop:1 #cccccc); "
            f"border: 1px solid #666666; border-radius: 3px;"
        )


class IMUDisplayWidget(QWidget):
    """
    Displays IMU data (Roll, Pitch, Yaw) from Float64MultiArray.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title_label = QLabel("IMU (Roll/Pitch/Yaw)")
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title_label)
        
        # Grid for IMU values
        grid = QGridLayout()
        
        self.imu_labels = {}
        for i, name in enumerate(["Roll", "Pitch", "Yaw"]):
            label = QLabel(name)
            label.setFont(QFont("Arial", 10, QFont.Bold))
            value = QLabel("-- °")
            value.setFont(QFont("Courier", 10))
            grid.addWidget(label, i, 0)
            grid.addWidget(value, i, 1)
            self.imu_labels[name.lower()] = value
        
        layout.addLayout(grid)
        self.setLayout(layout)
    
    def update_imu(self, msg):
        """Update IMU values from Float64MultiArray."""
        try:
            if hasattr(msg, 'data') and len(msg.data) >= 3:
                roll = msg.data[0]
                pitch = msg.data[1]
                yaw = msg.data[2]
                
                self.imu_labels['roll'].setText(f"{roll:.2f}°")
                self.imu_labels['pitch'].setText(f"{pitch:.2f}°")
                self.imu_labels['yaw'].setText(f"{yaw:.2f}°")
        except Exception as e:
            print(f"Error updating IMU: {e}")


class GPSDisplayWidget(QWidget):
    """
    Displays GPS data (Latitude, Longitude, Altitude).
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title
        title_label = QLabel("GPS Data")
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title_label)
        
        # Grid for GPS values
        grid = QGridLayout()
        
        self.gps_labels = {}
        for i, (name, unit) in enumerate([("Latitude", "°"), ("Longitude", "°"), ("Altitude", "m")]):
            label = QLabel(name)
            label.setFont(QFont("Arial", 10, QFont.Bold))
            value = QLabel(f"-- {unit}")
            value.setFont(QFont("Courier", 10))
            grid.addWidget(label, i, 0)
            grid.addWidget(value, i, 1)
            self.gps_labels[name.lower()] = value
        
        layout.addLayout(grid)
        self.setLayout(layout)
    
    def update_gps(self, msg):
        """Update GPS values from NavSatFix."""
        try:
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude
            
            self.gps_labels['latitude'].setText(f"{latitude:.6f}°")
            self.gps_labels['longitude'].setText(f"{longitude:.6f}°")
            self.gps_labels['altitude'].setText(f"{altitude:.2f}m")
        except Exception as e:
            print(f"Error updating GPS: {e}")


class StepperMotorControlWidget(QWidget):
    """
    Control widget for NEMA17 stepper motor (27:1 gearbox).
    
    Provides:
    - Position control (target position setting)
    - Velocity control (speed limiting)
    - Enable/Disable controls
    - Emergency stop
    - Home button
    - Real-time telemetry display (position, velocity, current)
    """
    
    # Signals for command emission
    set_position_signal = Signal(float)  # Target position in degrees
    set_velocity_signal = Signal(float)  # Max velocity in deg/s
    enable_signal = Signal(bool)         # Enable/disable motor
    stop_signal = Signal()               # Emergency stop
    home_signal = Signal()               # Home position
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        """Initialize the stepper motor control UI."""
        main_layout = QVBoxLayout()
        
        # Title
        title_label = QLabel("NEMA17 Stepper Motor (27:1 Gearbox)")
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        main_layout.addWidget(title_label)
        
        # Status section
        status_layout = QHBoxLayout()
        
        self.status_label = QLabel("Status: DISCONNECTED")
        self.status_label.setFont(QFont("Courier", 10))
        status_layout.addWidget(self.status_label)
        
        self.enabled_indicator = QFrame()
        self.enabled_indicator.setMaximumSize(20, 20)
        self.enabled_indicator.setStyleSheet("background-color: red; border-radius: 10px;")
        status_layout.addWidget(self.enabled_indicator)
        
        status_layout.addStretch()
        main_layout.addLayout(status_layout)
        
        # Control buttons
        button_layout = QHBoxLayout()
        
        self.enable_button = QPushButton("Enable")
        self.enable_button.setCheckable(True)
        self.enable_button.toggled.connect(self._on_enable_toggled)
        button_layout.addWidget(self.enable_button)
        
        self.stop_button = QPushButton("Emergency Stop")
        self.stop_button.setStyleSheet("background-color: #ff6666;")
        self.stop_button.clicked.connect(self.stop_signal.emit)
        button_layout.addWidget(self.stop_button)
        
        self.home_button = QPushButton("Home")
        self.home_button.clicked.connect(self.home_signal.emit)
        button_layout.addWidget(self.home_button)
        
        main_layout.addLayout(button_layout)
        
        # Position control
        pos_control_layout = QGridLayout()
        
        pos_label = QLabel("Target Position:")
        pos_label.setFont(QFont("Arial", 10, QFont.Bold))
        pos_control_layout.addWidget(pos_label, 0, 0)
        
        self.position_spinbox = QDoubleSpinBox()
        self.position_spinbox.setRange(-360000, 360000)
        self.position_spinbox.setSingleStep(10)
        self.position_spinbox.setSuffix(" °")
        self.position_spinbox.setDecimals(2)
        pos_control_layout.addWidget(self.position_spinbox, 0, 1)
        
        pos_button = QPushButton("Set Position")
        pos_button.clicked.connect(lambda: self.set_position_signal.emit(self.position_spinbox.value()))
        pos_control_layout.addWidget(pos_button, 0, 2)
        
        main_layout.addLayout(pos_control_layout)
        
        # Velocity control
        vel_control_layout = QGridLayout()
        
        vel_label = QLabel("Velocity Limit:")
        vel_label.setFont(QFont("Arial", 10, QFont.Bold))
        vel_control_layout.addWidget(vel_label, 0, 0)
        
        self.velocity_spinbox = QDoubleSpinBox()
        self.velocity_spinbox.setRange(0, 10000)
        self.velocity_spinbox.setSingleStep(10)
        self.velocity_spinbox.setSuffix(" °/s")
        self.velocity_spinbox.setValue(100)
        self.velocity_spinbox.setDecimals(2)
        vel_control_layout.addWidget(self.velocity_spinbox, 0, 1)
        
        vel_button = QPushButton("Set Velocity")
        vel_button.clicked.connect(lambda: self.set_velocity_signal.emit(self.velocity_spinbox.value()))
        vel_control_layout.addWidget(vel_button, 0, 2)
        
        main_layout.addLayout(vel_control_layout)
        
        # Telemetry display
        telemetry_layout = QGridLayout()
        
        telemetry_label = QLabel("Telemetry")
        telemetry_label.setFont(QFont("Arial", 11, QFont.Bold))
        telemetry_layout.addWidget(telemetry_label, 0, 0, 1, 2)
        
        # Current Position
        current_pos_label = QLabel("Current Position:")
        current_pos_label.setFont(QFont("Arial", 9))
        telemetry_layout.addWidget(current_pos_label, 1, 0)
        
        self.current_position_label = QLabel("-- °")
        self.current_position_label.setFont(QFont("Courier", 9))
        telemetry_layout.addWidget(self.current_position_label, 1, 1)
        
        # Target Position Display
        target_pos_label = QLabel("Target Position:")
        target_pos_label.setFont(QFont("Arial", 9))
        telemetry_layout.addWidget(target_pos_label, 2, 0)
        
        self.target_position_label = QLabel("-- °")
        self.target_position_label.setFont(QFont("Courier", 9))
        telemetry_layout.addWidget(self.target_position_label, 2, 1)
        
        # Current Velocity
        current_vel_label = QLabel("Current Velocity:")
        current_vel_label.setFont(QFont("Arial", 9))
        telemetry_layout.addWidget(current_vel_label, 3, 0)
        
        self.current_velocity_label = QLabel("-- °/s")
        self.current_velocity_label.setFont(QFont("Courier", 9))
        telemetry_layout.addWidget(self.current_velocity_label, 3, 1)
        
        # Motor Current
        motor_current_label = QLabel("Motor Current:")
        motor_current_label.setFont(QFont("Arial", 9))
        telemetry_layout.addWidget(motor_current_label, 4, 0)
        
        self.motor_current_label = QLabel("-- A")
        self.motor_current_label.setFont(QFont("Courier", 9))
        telemetry_layout.addWidget(self.motor_current_label, 4, 1)
        
        # Error status
        error_label = QLabel("Error Status:")
        error_label.setFont(QFont("Arial", 9))
        telemetry_layout.addWidget(error_label, 5, 0)
        
        self.error_status_label = QLabel("OK")
        self.error_status_label.setFont(QFont("Courier", 9))
        self.error_status_label.setStyleSheet("color: green;")
        telemetry_layout.addWidget(self.error_status_label, 5, 1)
        
        main_layout.addLayout(telemetry_layout)
        
        main_layout.addStretch()
        self.setLayout(main_layout)
    
    def _on_enable_toggled(self, checked: bool):
        """Handle enable button toggle."""
        self.enable_signal.emit(checked)
    
    def update_position(self, msg: Float64):
        """Update current position display."""
        self.current_position_label.setText(f"{msg.data:.2f}°")
    
    def update_target_position(self, msg: Float64):
        """Update target position display."""
        self.target_position_label.setText(f"{msg.data:.2f}°")
    
    def update_velocity(self, msg: Float64):
        """Update current velocity display."""
        self.current_velocity_label.setText(f"{msg.data:.2f}°/s")
    
    def update_current(self, msg: Float64):
        """Update motor current display."""
        # Clamp at max current (1.68A) for color indication
        current_amps = msg.data
        self.motor_current_label.setText(f"{current_amps:.3f}A")
        
        # Color code based on current
        if current_amps > 1.5:
            self.motor_current_label.setStyleSheet("color: red;")
        elif current_amps > 1.0:
            self.motor_current_label.setStyleSheet("color: orange;")
        else:
            self.motor_current_label.setStyleSheet("color: green;")
    
    def update_enabled(self, msg: Bool):
        """Update enabled status indicator."""
        is_enabled = msg.data
        self.enable_button.setChecked(is_enabled)
        
        if is_enabled:
            self.enabled_indicator.setStyleSheet("background-color: green; border-radius: 10px;")
            self.status_label.setText("Status: ENABLED")
        else:
            self.enabled_indicator.setStyleSheet("background-color: red; border-radius: 10px;")
            self.status_label.setText("Status: DISABLED")
    
    def update_error(self, msg: String):
        """Update error status display."""
        error_text = msg.data
        if error_text and error_text.strip():
            self.error_status_label.setText(f"ERROR: {error_text}")
            self.error_status_label.setStyleSheet("color: red;")
        else:
            self.error_status_label.setText("OK")
            self.error_status_label.setStyleSheet("color: green;")
