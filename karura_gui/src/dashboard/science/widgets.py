"""
Science Dashboard Custom Widgets

Reusable UI components for the Science Dashboard, including:
- Image/video display
- Sensor gauges
- Telemetry panels
- Data indicators
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QGridLayout
)
from PySide6.QtCore import Qt, QPixmap, QSize
from PySide6.QtGui import QFont, QColor, QImage
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import Float64, Int
import numpy as np


class ImageDisplayWidget(QWidget):
    """
    Displays ROS Image messages as a pixmap.
    Handles conversion from ROS Image to QPixmap for display.
    """
    
    def __init__(self, title: str = "Camera Feed", parent=None):
        super().__init__(parent)
        self.title = title
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        
        # Title label
        title_label = QLabel(self.title)
        title_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(title_label)
        
        # Image display
        self.image_label = QLabel()
        self.image_label.setMinimumSize(320, 240)
        self.image_label.setStyleSheet("border: 1px solid #cccccc; background-color: #000000;")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.image_label)
        
        self.setLayout(layout)
    
    def update_image(self, ros_image: ROSImage):
        """
        Convert ROS Image message to QPixmap and display.
        
        Args:
            ros_image: sensor_msgs.msg.Image
        """
        try:
            # Convert ROS image data to numpy array
            if ros_image.encoding == "rgb8":
                # RGB image
                data = np.frombuffer(ros_image.data, dtype=np.uint8)
                data = data.reshape((ros_image.height, ros_image.width, 3))
                q_image = QImage(
                    data.data, ros_image.width, ros_image.height,
                    3 * ros_image.width, QImage.Format.Format_RGB888
                )
            elif ros_image.encoding == "bgr8":
                # BGR image (convert to RGB)
                data = np.frombuffer(ros_image.data, dtype=np.uint8)
                data = data.reshape((ros_image.height, ros_image.width, 3))
                data = data[..., ::-1]  # BGR to RGB
                q_image = QImage(
                    data.data, ros_image.width, ros_image.height,
                    3 * ros_image.width, QImage.Format.Format_RGB888
                )
            elif ros_image.encoding == "mono8":
                # Grayscale image
                data = np.frombuffer(ros_image.data, dtype=np.uint8)
                data = data.reshape((ros_image.height, ros_image.width))
                q_image = QImage(
                    data.data, ros_image.width, ros_image.height,
                    ros_image.width, QImage.Format.Format_Grayscale8
                )
            else:
                self.image_label.setText(f"Unsupported encoding: {ros_image.encoding}")
                return
            
            # Scale image to fit widget
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaledToWidth(
                self.image_label.width(), Qt.TransformationMode.SmoothTransformation
            )
            self.image_label.setPixmap(scaled_pixmap)
        except Exception as e:
            self.image_label.setText(f"Error displaying image: {str(e)}")


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
        
        # Title
        title_label = QLabel(self.title)
        title_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(title_label)
        
        # Value display
        self.value_label = QLabel("-- --")
        self.value_label.setFont(QFont("Courier", 14, QFont.Bold))
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.value_label)
        
        # Status frame
        self.status_frame = QFrame()
        self.status_frame.setMinimumHeight(10)
        self.status_frame.setStyleSheet("background-color: #cccccc; border-radius: 3px;")
        layout.addWidget(self.status_frame)
        
        self.setLayout(layout)
    
    def update_value(self, msg: Float64):
        """Update the displayed sensor value."""
        value = msg.data
        self.value_label.setText(f"{value:.2f} {self.unit}")
        
        # Update status color based on range
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
    
    def update_battery(self, msg: Int):
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
