"""
ScienceNode

ROS 2 node responsible for science and visualization-related topics
for the Karura dashboard GUI, including video streams and sensor data.

TODO: Implement the Science Bridge to register PySide6 callbacks 
for the topics dispatched here.
"""
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int32
from sensor_msgs.msg import Image, NavSatFix
from .base_node import BaseDashboardNode 

class ScienceNode(BaseDashboardNode):
    """
    Science dashboard ROS 2 node.
    Default node name: "karura_science_gui"
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