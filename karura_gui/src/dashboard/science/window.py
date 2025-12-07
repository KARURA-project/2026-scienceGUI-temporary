"""
Science Dashboard Main Window

Provides the primary UI layout for the Science Dashboard,
combining camera feeds, sensor telemetry, and control panels.
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QScrollArea
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont

from .widgets import (
    ImageDisplayWidget, SensorGaugeWidget, TelemetryPanelWidget,
    BatteryIndicatorWidget, IMUDisplayWidget, GPSDisplayWidget
)


class ScienceMainWindow(QMainWindow):
    """
    Main window for the Science Dashboard.
    
    Displays:
    - Multiple camera feeds (downward, box, fluorescence)
    - Sensor telemetry (temperature, humidity, pressure)
    - IMU and GPS data
    - Battery status
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Karura Science Dashboard")
        self.setGeometry(100, 100, 1400, 900)
        
        self.init_ui()
    
    def init_ui(self):
        """Initialize the main UI layout."""
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QHBoxLayout()
        
        # ========================================
        # LEFT PANEL: Camera Feeds (Tabbed)
        # ========================================
        camera_tab = QTabWidget()
        
        # Downward camera
        self.downward_cam_widget = ImageDisplayWidget("Downward Camera")
        camera_tab.addTab(self.downward_cam_widget, "Downward")
        
        # Box camera
        self.box_cam_widget = ImageDisplayWidget("Science Box Camera")
        camera_tab.addTab(self.box_cam_widget, "Box")
        
        # Fluorescence camera
        self.fluorescence_cam_widget = ImageDisplayWidget("Fluorescence Camera")
        camera_tab.addTab(self.fluorescence_cam_widget, "Fluorescence")
        
        main_layout.addWidget(camera_tab, 3)  # 60% width
        
        # ========================================
        # RIGHT PANEL: Telemetry & Status
        # ========================================
        right_layout = QVBoxLayout()
        
        # --- Sensor Telemetry Section ---
        sensor_panel = TelemetryPanelWidget("Sensor Telemetry")
        self.temperature_label = SensorGaugeWidget(
            "Temperature", "°C", min_val=-10, max_val=50
        )
        self.humidity_label = SensorGaugeWidget(
            "Humidity", "%", min_val=0, max_val=100
        )
        self.pressure_label = SensorGaugeWidget(
            "Pressure", "kPa", min_val=85, max_val=105
        )
        
        sensor_group_layout = QVBoxLayout()
        sensor_group_layout.addWidget(self.temperature_label)
        sensor_group_layout.addWidget(self.humidity_label)
        sensor_group_layout.addWidget(self.pressure_label)
        
        sensor_container = QWidget()
        sensor_container.setLayout(sensor_group_layout)
        right_layout.addWidget(sensor_container)
        
        # --- Battery Status Section ---
        self.battery_widget = BatteryIndicatorWidget()
        right_layout.addWidget(self.battery_widget)
        
        # --- IMU Data Section ---
        self.imu_widget = IMUDisplayWidget()
        right_layout.addWidget(self.imu_widget)
        
        # --- GPS Data Section ---
        self.gps_widget = GPSDisplayWidget()
        right_layout.addWidget(self.gps_widget)
        
        right_layout.addStretch()
        
        # Wrap right layout in a widget and add to main
        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        main_layout.addWidget(right_widget, 2)  # 40% width
        
        central_widget.setLayout(main_layout)
        
        # Apply stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QLabel {
                color: #333333;
            }
            QTabWidget::pane {
                border: 1px solid #cccccc;
            }
            QTabBar::tab {
                background-color: #e0e0e0;
                padding: 5px 15px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: #ffffff;
            }
        """)
    
    def connect_signals(self, bridge):
        """
        Connect Science Bridge signals to UI update slots.
        
        Args:
            bridge: ScienceBridge instance
        """
        # Camera signals
        bridge.downward_cam_signal.connect(self.on_downward_cam_update)
        bridge.box_cam_signal.connect(self.on_box_cam_update)
        bridge.fluorescence_cam_signal.connect(self.on_fluorescence_cam_update)
        
        # Sensor signals
        bridge.temperature_signal.connect(self.on_temperature_update)
        bridge.humidity_signal.connect(self.on_humidity_update)
        bridge.pressure_signal.connect(self.on_pressure_update)
        
        # Other data signals
        bridge.roll_pitch_yaw_signal.connect(self.on_imu_update)
        bridge.battery_data_signal.connect(self.on_battery_update)
        bridge.gps_data_signal.connect(self.on_gps_update)
    
    # ========================================
    # SIGNAL HANDLERS
    # ========================================
    
    def on_downward_cam_update(self, msg):
        """Handle downward camera image update."""
        self.downward_cam_widget.update_image(msg)
    
    def on_box_cam_update(self, msg):
        """Handle science box camera image update."""
        self.box_cam_widget.update_image(msg)
    
    def on_fluorescence_cam_update(self, msg):
        """Handle fluorescence camera image update."""
        self.fluorescence_cam_widget.update_image(msg)
    
    def on_temperature_update(self, msg):
        """Handle temperature sensor update."""
        self.temperature_label.update_value(msg)
    
    def on_humidity_update(self, msg):
        """Handle humidity sensor update."""
        self.humidity_label.update_value(msg)
    
    def on_pressure_update(self, msg):
        """Handle pressure sensor update."""
        self.pressure_label.update_value(msg)
    
    def on_imu_update(self, msg):
        """Handle IMU (Roll/Pitch/Yaw) update."""
        self.imu_widget.update_imu(msg)
    
    def on_battery_update(self, msg):
        """Handle battery status update."""
        self.battery_widget.update_battery(msg)
    
    def on_gps_update(self, msg):
        """Handle GPS data update."""
        self.gps_widget.update_gps(msg)
