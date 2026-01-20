"""
ScienceNode

ROS 2 node responsible for science and visualization-related topics
for the Karura dashboard GUI, including video streams, sensor data,
and stepper motor control.

TODO: Implement the Science Bridge to register PySide6 callbacks 
for the topics dispatched here.
"""
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int32, Bool, String
from sensor_msgs.msg import Image, NavSatFix
from .base_node import BaseDashboardNode 

class ScienceNode(BaseDashboardNode):
    """
    Science dashboard ROS 2 node.
    Default node name: "karura_science_gui"
    
    Subscribes to:
    - Camera feeds (downward, box, fluorescence)
    - Sensor data (temperature, humidity, pressure)
    - IMU, GPS, and battery data
    - Stepper motor telemetry (position, velocity, current, status)
    """

    def __init__(self, node_name: str = "karura_science_gui"):
        super().__init__(node_name)

        # ----------------------------------------------------
        # Subscribers for Image/Video Streams (sensor_msgs/Image)
        # ----------------------------------------------------

        # Downward Camera: video image showing the front of the rover
        self.create_subscription(
            Image,
            "/downward_cam",
            self._on_downward_cam,
            10,
        )

        # Box Camera: video image monitoring inside science box
        self.create_subscription(
            Image,
            "/box_cam",
            self._on_box_cam,
            10,
        )

        # Fluorescence Camera: video image from fluorescence reaction
        self.create_subscription(
            Image,
            "/fluorescence_cam",
            self._on_fluorescence_cam,
            10,
        )
        
        # ----------------------------------------------------
        # Subscribers for Sensor Data (std_msgs/Float64)
        # ----------------------------------------------------
        
        # Temperature Sensor
        self.create_subscription(
            Float64,
            "/temperature",
            self._on_temperature,
            10,
        )

        # Humidity Sensor
        self.create_subscription(
            Float64,
            "/humidity",
            self._on_humidity,
            10,
        )
        
        # Pressure Sensor
        self.create_subscription(
            Float64,
            "/pressure",
            self._on_pressure,
            10,
        )
        
        # ----------------------------------------------------
        # Subscribers for General Data (IMU, Battery, GPS)
        # ----------------------------------------------------
        
        # IMU Data (Roll/Pitch/Yaw)
        self.create_subscription(
            Float64MultiArray,
            "/roll_pitch_yaw", # Assuming this topic name based on contents
            self._on_roll_pitch_yaw,
            10,
        )
        
        # Battery Data (Int)
        self.create_subscription(
            Int32,
            "/battery_data",
            self._on_battery_data,
            10,
        )
        
        # GPS Data (NavSatFix)
        self.create_subscription(
            NavSatFix,
            "/gps_data",
            self._on_gps_data,
            10,
        )
        
        # ----------------------------------------------------
        # Subscribers for Stepper Motor Telemetry
        # (NEMA17 - 1.8 Degree - 1.68A Stepper - 27:1 Gearbox)
        # ----------------------------------------------------
        
        # Stepper motor current position (degrees)
        self.create_subscription(
            Float64,
            "/stepper/position",
            self._on_stepper_position,
            10,
        )
        
        # Stepper motor current velocity (degrees per second)
        self.create_subscription(
            Float64,
            "/stepper/velocity",
            self._on_stepper_velocity,
            10,
        )
        
        # Stepper motor current draw (Amperes)
        self.create_subscription(
            Float64,
            "/stepper/current",
            self._on_stepper_current,
            10,
        )
        
        # Stepper motor enabled status
        self.create_subscription(
            Bool,
            "/stepper/enabled",
            self._on_stepper_enabled,
            10,
        )
        
        # Stepper motor error status
        self.create_subscription(
            String,
            "/stepper/error",
            self._on_stepper_error,
            10,
        )
        
        # Stepper motor target position (degrees)
        self.create_subscription(
            Float64,
            "/stepper/target_position",
            self._on_stepper_target_position,
            10,
        )


    # ----------------------------------------------------
    # Callback methods using the BaseDashboardNode dispatch pattern
    # ----------------------------------------------------
    
    # Note: Using the name-mangled format for dispatch: _BaseDashboardNode__dispatch
    # This is required because __dispatch is defined as a private method in base_node.py.

    # --- Image Callbacks ---
    def _on_downward_cam(self, msg: Image):
        # Dispatch the received Image message to the bridge using the topic name as key
        self._BaseDashboardNode__dispatch("downward_cam", msg)

    def _on_box_cam(self, msg: Image):
        self._BaseDashboardNode__dispatch("box_cam", msg)
        
    def _on_fluorescence_cam(self, msg: Image):
        self._BaseDashboardNode__dispatch("fluorescence_cam", msg)

    # --- Sensor Callbacks ---
    def _on_temperature(self, msg: Float64):
        self._BaseDashboardNode__dispatch("temperature", msg)

    def _on_humidity(self, msg: Float64):
        self._BaseDashboardNode__dispatch("humidity", msg)

    def _on_pressure(self, msg: Float64):
        self._BaseDashboardNode__dispatch("pressure", msg)

    # --- General Data Callbacks ---
    def _on_roll_pitch_yaw(self, msg: Float64MultiArray):
        self._BaseDashboardNode__dispatch("roll_pitch_yaw", msg)
        
    def _on_battery_data(self, msg: Int32):
        self._BaseDashboardNode__dispatch("battery_data", msg)

    def _on_gps_data(self, msg: NavSatFix):
        self._BaseDashboardNode__dispatch("gps_data", msg)
    
    # --- Stepper Motor Callbacks ---
    def _on_stepper_position(self, msg: Float64):
        """Handle stepper motor position update."""
        self._BaseDashboardNode__dispatch("stepper_position", msg)
    
    def _on_stepper_velocity(self, msg: Float64):
        """Handle stepper motor velocity update."""
        self._BaseDashboardNode__dispatch("stepper_velocity", msg)
    
    def _on_stepper_current(self, msg: Float64):
        """Handle stepper motor current update."""
        self._BaseDashboardNode__dispatch("stepper_current", msg)
    
    def _on_stepper_enabled(self, msg: Bool):
        """Handle stepper motor enabled status update."""
        self._BaseDashboardNode__dispatch("stepper_enabled", msg)
    
    def _on_stepper_error(self, msg: String):
        """Handle stepper motor error status update."""
        self._BaseDashboardNode__dispatch("stepper_error", msg)
    
    def _on_stepper_target_position(self, msg: Float64):
        """Handle stepper motor target position update."""
        self._BaseDashboardNode__dispatch("stepper_target_position", msg)