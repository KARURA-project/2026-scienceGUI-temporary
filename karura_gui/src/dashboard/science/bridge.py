"""
Science Dashboard Bridge

Connects the ScienceNode (ROS 2 backend) to the ScienceMainWindow (PySide6 UI)
using Qt signals/slots for thread-safe data transfer.
"""

from typing import Type

from PySide6.QtCore import QObject, Signal

from dashboard.backend.science_node import ScienceNode
from dashboard.core.base_bridge import BaseROS2Bridge

# Import ROS message types for signal type hints
from std_msgs.msg import Float64, Float32, Float64MultiArray, Int32, Bool, String
from sensor_msgs.msg import Image, NavSatFix


class ScienceBridge(BaseROS2Bridge):
    """
    ROS 2 Bridge for the Science Dashboard.

    Defines Qt Signals that transfer data from the ScienceNode (running in
    the ROS2Worker thread) to the ScienceMainWindow (running in the main thread).

    Signal Flow:
    1. ROS 2 topic message arrives at ScienceNode callback
    2. Callback dispatches the message via __dispatch(key, msg)
    3. Bridge's registered callback emits the corresponding Qt Signal
    4. Signal is received by ScienceMainWindow slots on the main thread
    5. Window updates UI widgets
    """

    # Camera signals
    downward_front_cam_signal = Signal(Image)
    downward_back_cam_signal = Signal(Image)
    arm_cam_signal = Signal(Image)
    panoramic_cam_signal = Signal(Image)
    box_cam_signal = Signal(Image)

    # Sensor signals
    temperature_signal = Signal(Float64)
    humidity_signal = Signal(Float64)
    pressure_signal = Signal(Float64)
    
    # New Sensor signals (UV, CO2, VOC, HCHO, NH3)
    uv_signal = Signal(Float64)
    co2_signal = Signal(Float64)
    voc_signal = Signal(Float64)
    hcho_signal = Signal(Float64)
    nh3_signal = Signal(Float64)

    # General data signals
    roll_pitch_yaw_signal = Signal(Float64MultiArray)
    battery_data_signal = Signal(Int32)
    gps_data_signal = Signal(NavSatFix)
    
    # Stepper motor signals
    stepper_position_signal = Signal(Float64)
    stepper_velocity_signal = Signal(Float64)
    stepper_current_signal = Signal(Float64)
    stepper_enabled_signal = Signal(Bool)
    stepper_error_signal = Signal(String)
    stepper_target_position_signal = Signal(Float64)

    error_signal = Signal(str)
    
    def _on_ros_error(self, err) -> None:
        """
        Handler for ROS-side errors emitted by BaseROS2Bridge.
        Keeps UI informed and prevents crashes.
        """
        try:
            msg = str(err)
            # Emit to UI / log panel
            self.error_signal.emit(msg)
        except Exception as e:
            # Last-resort: avoid raising inside error handler
            print(f"[ScienceBridge] Failed in _on_ros_error: {e} (original: {err})")


    def __init__(self, node_name: str = "science_bridge_node"):
        """
        Initialize the Science Bridge.

        Args:
            node_name: Name for the ROS 2 node
        """
        super().__init__(ScienceNode, node_name)
        self.science_node: ScienceNode = self.node

        # Register callbacks
        self.science_node.register_callback("downward_front_cam", self._emit_downward_front_cam)
        self.science_node.register_callback("downward_back_cam", self._emit_downward_back_cam)
        self.science_node.register_callback("arm_cam", self._emit_arm_cam)
        self.science_node.register_callback("panoramic_cam", self._emit_panoramic_cam)
        self.science_node.register_callback("box_cam", self._emit_box_cam)

        self.science_node.register_callback("temperature", self._emit_temperature)
        self.science_node.register_callback("humidity", self._emit_humidity)
        self.science_node.register_callback("pressure", self._emit_pressure)
        
        self.science_node.register_callback("uv", self._emit_uv)
        self.science_node.register_callback("co2", self._emit_co2)
        self.science_node.register_callback("voc", self._emit_voc)
        self.science_node.register_callback("hcho", self._emit_hcho)
        self.science_node.register_callback("nh3", self._emit_nh3)

        self.science_node.register_callback("roll_pitch_yaw", self._emit_roll_pitch_yaw)
        self.science_node.register_callback("battery_data", self._emit_battery_data)
        self.science_node.register_callback("gps_data", self._emit_gps_data)
        
        # Register stepper motor callbacks
        self.science_node.register_callback("stepper_position", self._emit_stepper_position)
        self.science_node.register_callback("stepper_velocity", self._emit_stepper_velocity)
        self.science_node.register_callback("stepper_current", self._emit_stepper_current)
        self.science_node.register_callback("stepper_enabled", self._emit_stepper_enabled)
        self.science_node.register_callback("stepper_error", self._emit_stepper_error)
        self.science_node.register_callback("stepper_target_position", self._emit_stepper_target_position)

        self.ros_error.connect(self._on_ros_error)
    
    # ----------------------------------------
    # EMITTER METHODS - CAMERA FEEDS
    # ----------------------------------------
    
    def _emit_downward_front_cam(self, msg: Image):
        """Emit downward front camera image signal."""
        try:
            self.downward_front_cam_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting downward_front_cam: {e}")

    def _emit_downward_back_cam(self, msg: Image):
        """Emit downward back camera image signal."""
        try:
            self.downward_back_cam_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting downward_back_cam: {e}")

    def _emit_arm_cam(self, msg: Image):
        """Emit arm camera image signal."""
        try:
            self.arm_cam_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting arm_cam: {e}")
    
    def _emit_panoramic_cam(self, msg: Image):
        """Emit panoramic camera image signal."""
        try:
            self.panoramic_cam_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting panoramic_cam: {e}")

    def _emit_box_cam(self, msg: Image):
        """Emit science box camera image signal."""
        try:
            self.box_cam_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting box_cam: {e}")
    
    # ----------------------------------------
    # EMITTER METHODS - SENSORS
    # ----------------------------------------
    
    def _emit_temperature(self, msg: Float64):
        """Emit temperature sensor signal."""
        try:
            self.temperature_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting temperature: {e}")
    
    def _emit_humidity(self, msg: Float64):
        """Emit humidity sensor signal."""
        try:
            self.humidity_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting humidity: {e}")
    
    def _emit_pressure(self, msg: Float64):
        """Emit pressure sensor signal."""
        try:
            self.pressure_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting pressure: {e}")
            
    def _emit_uv(self, msg: Float64):
        """Emit UV sensor signal."""
        try:
            self.uv_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting uv: {e}")

    def _emit_co2(self, msg: Float64):
        """Emit CO2 sensor signal."""
        try:
            self.co2_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting co2: {e}")

    def _emit_voc(self, msg: Float64):
        """Emit VOC sensor signal."""
        try:
            self.voc_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting voc: {e}")

    def _emit_hcho(self, msg: Float64):
        """Emit HCHO sensor signal."""
        try:
            self.hcho_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting hcho: {e}")

    def _emit_nh3(self, msg: Float64):
        """Emit NH3 sensor signal."""
        try:
            self.nh3_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting nh3: {e}")
    
    # ----------------------------------------
    # EMITTER METHODS - GENERAL DATA
    # ----------------------------------------
    
    def _emit_roll_pitch_yaw(self, msg: Float64MultiArray):
        """Emit IMU (Roll/Pitch/Yaw) signal."""
        try:
            self.roll_pitch_yaw_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting roll_pitch_yaw: {e}")
    
    def _emit_battery_data(self, msg: Int32):
        """Emit battery status signal."""
        try:
            self.battery_data_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting battery_data: {e}")
    
    def _emit_gps_data(self, msg: NavSatFix):
        """Emit GPS data signal."""
        try:
            self.gps_data_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting gps_data: {e}")
    
    # ----------------------------------------
    # EMITTER METHODS - STEPPER MOTOR
    # ----------------------------------------
    
    def _emit_stepper_position(self, msg: Float64):
        """Emit stepper position signal."""
        try:
            self.stepper_position_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting stepper_position: {e}")
    
    def _emit_stepper_velocity(self, msg: Float64):
        """Emit stepper velocity signal."""
        try:
            self.stepper_velocity_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting stepper_velocity: {e}")
    
    def _emit_stepper_current(self, msg: Float64):
        """Emit stepper current signal."""
        try:
            self.stepper_current_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting stepper_current: {e}")
    
    def _emit_stepper_enabled(self, msg: Bool):
        """Emit stepper enabled status signal."""
        try:
            self.stepper_enabled_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting stepper_enabled: {e}")
    
    def _emit_stepper_error(self, msg: String):
        """Emit stepper error status signal."""
        try:
            self.stepper_error_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting stepper_error: {e}")
    
    def _emit_stepper_target_position(self, msg: Float64):
        """Emit stepper target position signal."""
        try:
            self.stepper_target_position_signal.emit(msg)
        except Exception as e:
            self.error_signal.emit(f"Error emitting stepper_target_position: {e}")