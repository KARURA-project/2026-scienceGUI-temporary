"""
Science Dashboard Main Window

Provides the primary UI layout for the Science Dashboard,
combining camera feeds, sensor telemetry, and control panels.
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QScrollArea, QSplitter, QGridLayout
)
from PySide6.QtCore import Qt, QSize
from PySide6.QtGui import QFont

from .widgets import (
    ImageDisplayWidget, SensorGaugeWidget, TelemetryPanelWidget,
    BatteryIndicatorWidget, IMUDisplayWidget, GPSDisplayWidget,
    StepperMotorControlWidget
)


class ScienceMainWindow(QMainWindow):
    """
    Main window for the Science Dashboard.
    
    Displays:
    - Multiple camera feeds (downward, box, fluorescence)
    - Sensor telemetry (temperature, humidity, pressure)
    - IMU and GPS data
    - Battery status
    - NEMA17 stepper motor control (27:1 gearbox)
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Karura Science Dashboard")
        self.setGeometry(100, 100, 1280, 720)
        
        self.init_ui()
    
    def init_ui(self):
        """Initialize the main UI layout."""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout()

        # Camera feeds (tabbed)
        camera_tab = QTabWidget()

        # Create widgets for Tabs
        self.panoramic_cam_widget = ImageDisplayWidget("Panoramic Camera")
        self.downward_front_cam_widget = ImageDisplayWidget("Downward Front Camera")
        self.downward_back_cam_widget = ImageDisplayWidget("Downward Back Camera")
        self.arm_cam_widget = ImageDisplayWidget("Arm Camera")
        #self.box_cam_widget = ImageDisplayWidget("Science Box Camera")
        self.box_cam_widget = ImageDisplayWidget("Front Camera")

        # Create SEPARATE widgets for Grid (Quad View)
        # Qt widgets cannot be in two places at once, so we need duplicates.
        self.grid_panoramic = ImageDisplayWidget("Panoramic")
        self.grid_downward_front = ImageDisplayWidget("Downward Front")
        self.grid_downward_back = ImageDisplayWidget("Downward Back")
        self.grid_arm = ImageDisplayWidget("Arm")
        self.grid_box = ImageDisplayWidget("Box")
        self.grid_box = ImageDisplayWidget("Front")

        # Quad View Layout: Panoramic on top, 2x2 Grid on bottom
        quad_tab_widget = QWidget()
        quad_layout = QVBoxLayout()
        quad_tab_widget.setLayout(quad_layout)

        # Top: Panoramic
        quad_layout.addWidget(self.grid_panoramic, stretch=1)

        # Bottom: 2x2 Grid
        grid_container = QWidget()
        video_grid = QGridLayout()
        grid_container.setLayout(video_grid)
        video_grid.setContentsMargins(0, 5, 0, 0)
        video_grid.setSpacing(5)

        # Add to grid (Row, Column)
        video_grid.addWidget(self.grid_downward_front, 0, 0)
        video_grid.addWidget(self.grid_downward_back, 0, 1)
        video_grid.addWidget(self.grid_arm, 1, 0)
        video_grid.addWidget(self.grid_box, 1, 1)
        
        quad_layout.addWidget(grid_container, stretch=2)

        # Add the grid tab
        camera_tab.addTab(quad_tab_widget, "Quad View")

        # Add individual tabs
        camera_tab.addTab(self.downward_front_cam_widget, "Front")
        camera_tab.addTab(self.downward_back_cam_widget, "Back")
        camera_tab.addTab(self.arm_cam_widget, "Arm")
        camera_tab.addTab(self.box_cam_widget, "Box")
        camera_tab.addTab(self.box_cam_widget, "Front")
        camera_tab.addTab(self.panoramic_cam_widget, "Panoramic")

        main_layout.addWidget(camera_tab, 3)

        # Telemetry panel
        right_layout = QVBoxLayout()

        # Sensor telemetry
        self.temperature_label = SensorGaugeWidget(
            "Temperature", "°C", min_val=15, max_val=35
        )
        self.humidity_label = SensorGaugeWidget(
            "Humidity", "%", min_val=30, max_val=90
        )
        self.pressure_label = SensorGaugeWidget(
            "Pressure", "kPa", min_val=90, max_val=110
        )
        
        # New Sensors
        self.uv_label = SensorGaugeWidget("UV Index", "", 0, 11)
        self.co2_label = SensorGaugeWidget("CO2", "ppm", 400, 1500)
        self.voc_label = SensorGaugeWidget("VOC", "ppb", 0, 500)
        self.hcho_label = SensorGaugeWidget("HCHO", "mg/m³", 0, 0.1)
        self.nh3_label = SensorGaugeWidget("NH3", "ppm", 0, 5)

        # 8 sensors in 4 rows x 2 columns
        sensor_group_layout = QGridLayout()
        sensor_group_layout.addWidget(self.temperature_label, 0, 0)
        sensor_group_layout.addWidget(self.humidity_label, 0, 1)
        sensor_group_layout.addWidget(self.pressure_label, 1, 0)
        sensor_group_layout.addWidget(self.uv_label, 1, 1)
        sensor_group_layout.addWidget(self.co2_label, 2, 0)
        sensor_group_layout.addWidget(self.voc_label, 2, 1)
        sensor_group_layout.addWidget(self.hcho_label, 3, 0)
        sensor_group_layout.addWidget(self.nh3_label, 3, 1)

        sensor_container = QWidget()
        sensor_container.setLayout(sensor_group_layout)
        right_layout.addWidget(sensor_container)

        # Battery, IMU, GPS
        self.battery_widget = BatteryIndicatorWidget()
        right_layout.addWidget(self.battery_widget)

        self.imu_widget = IMUDisplayWidget()
        right_layout.addWidget(self.imu_widget)

        self.gps_widget = GPSDisplayWidget()
        right_layout.addWidget(self.gps_widget)
        
        # Stepper motor control
        self.stepper_widget = StepperMotorControlWidget()
        right_layout.addWidget(self.stepper_widget)

        right_layout.addStretch()

        right_widget = QWidget()
        right_widget.setLayout(right_layout)
        main_layout.addWidget(right_widget, 2)

        central_widget.setLayout(main_layout)

        # self.setStyleSheet("""
        #     QMainWindow {
        #         background-color: #f0f0f0;
        #     }
        #     QLabel {
        #         color: #333333;
        #     }
        #     QTabWidget::pane {
        #         border: 1px solid #cccccc;
        #     }
        #     QTabBar::tab {
        #         background-color: #e0e0e0;
        #         padding: 5px 15px;
        #         margin-right: 2px;
        #     }
        #     QTabBar::tab:selected {
        #         background-color: #ffffff;
        #     }
        # """)
    
    def connect_signals(self, bridge):
        """
        Connect Science Bridge signals to UI update slots.
        
        Args:
            bridge: ScienceBridge instance
        """
        # Camera signals
        bridge.downward_front_cam_signal.connect(self.on_downward_front_cam_update)
        bridge.downward_back_cam_signal.connect(self.on_downward_back_cam_update)
        bridge.arm_cam_signal.connect(self.on_arm_cam_update)
        bridge.panoramic_cam_signal.connect(self.on_panoramic_cam_update)
        bridge.box_cam_signal.connect(self.on_box_cam_update)
        
        # Sensor signals
        bridge.temperature_signal.connect(self.on_temperature_update)
        bridge.humidity_signal.connect(self.on_humidity_update)
        bridge.pressure_signal.connect(self.on_pressure_update)
        
        # New Sensor signals
        bridge.uv_signal.connect(self.on_uv_update)
        bridge.co2_signal.connect(self.on_co2_update)
        bridge.voc_signal.connect(self.on_voc_update)
        bridge.hcho_signal.connect(self.on_hcho_update)
        bridge.nh3_signal.connect(self.on_nh3_update)
        
        # Other data signals
        bridge.roll_pitch_yaw_signal.connect(self.on_imu_update)
        bridge.battery_data_signal.connect(self.on_battery_update)
        bridge.gps_data_signal.connect(self.on_gps_update)
        
        # Stepper motor signals
        bridge.stepper_position_signal.connect(self.on_stepper_position_update)
        bridge.stepper_velocity_signal.connect(self.on_stepper_velocity_update)
        bridge.stepper_current_signal.connect(self.on_stepper_current_update)
        bridge.stepper_enabled_signal.connect(self.on_stepper_enabled_update)
        bridge.stepper_error_signal.connect(self.on_stepper_error_update)
        bridge.stepper_target_position_signal.connect(self.on_stepper_target_position_update)
    
    # ========================================
    # SIGNAL HANDLERS
    # ========================================
    
    def on_downward_front_cam_update(self, msg):
        """Handle downward front camera image update."""
        self.downward_front_cam_widget.update_image(msg)
        self.grid_downward_front.update_image(msg)

    def on_downward_back_cam_update(self, msg):
        """Handle downward back camera image update."""
        self.downward_back_cam_widget.update_image(msg)
        self.grid_downward_back.update_image(msg)

    def on_arm_cam_update(self, msg):
        """Handle arm camera image update."""
        self.arm_cam_widget.update_image(msg)
        self.grid_arm.update_image(msg)
    
    def on_panoramic_cam_update(self, msg):
        """Handle panoramic camera image update."""
        self.panoramic_cam_widget.update_image(msg)
        self.grid_panoramic.update_image(msg)

    def on_box_cam_update(self, msg):
        """Handle science box camera image update."""
        self.box_cam_widget.update_image(msg)
        self.grid_box.update_image(msg)
    
    def on_temperature_update(self, msg):
        """Handle temperature sensor update."""
        self.temperature_label.update_value(msg)
    
    def on_humidity_update(self, msg):
        """Handle humidity sensor update."""
        self.humidity_label.update_value(msg)
    
    def on_pressure_update(self, msg):
        """Handle pressure sensor update."""
        self.pressure_label.update_value(msg)
    
    def on_uv_update(self, msg):
        """Handle UV sensor update."""
        self.uv_label.update_value(msg)

    def on_co2_update(self, msg):
        """Handle CO2 sensor update."""
        self.co2_label.update_value(msg)

    def on_voc_update(self, msg):
        """Handle VOC sensor update."""
        self.voc_label.update_value(msg)

    def on_hcho_update(self, msg):
        """Handle HCHO sensor update."""
        self.hcho_label.update_value(msg)

    def on_nh3_update(self, msg):
        """Handle NH3 sensor update."""
        self.nh3_label.update_value(msg)
    
    def on_imu_update(self, msg):
        """Handle IMU (Roll/Pitch/Yaw) update."""
        self.imu_widget.update_imu(msg)
    
    def on_battery_update(self, msg):
        """Handle battery status update."""
        self.battery_widget.update_battery(msg)
    
    def on_gps_update(self, msg):
        """Handle GPS data update."""
        self.gps_widget.update_gps(msg)
    
    def on_stepper_position_update(self, msg):
        """Handle stepper motor position update."""
        self.stepper_widget.update_position(msg)
    
    def on_stepper_velocity_update(self, msg):
        """Handle stepper motor velocity update."""
        self.stepper_widget.update_velocity(msg)
    
    def on_stepper_current_update(self, msg):
        """Handle stepper motor current update."""
        self.stepper_widget.update_current(msg)
    
    def on_stepper_enabled_update(self, msg):
        """Handle stepper motor enabled status update."""
        self.stepper_widget.update_enabled(msg)
    
    def on_stepper_error_update(self, msg):
        """Handle stepper motor error status update."""
        self.stepper_widget.update_error(msg)
    
    def on_stepper_target_position_update(self, msg):
        """Handle stepper motor target position update."""
        self.stepper_widget.update_target_position(msg)
