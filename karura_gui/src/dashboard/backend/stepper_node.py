"""
StepperMotorNode

ROS 2 node responsible for NEMA17 stepper motor (27:1 gearbox) control
for the Karura dashboard GUI using Phidget stepper motors.

Provides control and telemetry for stepper motor operations including:
- Position tracking
- Velocity control
- Current monitoring
- Error handling
"""

from rclpy.node import Node
from std_msgs.msg import Float64, Int32, Bool, String
from .base_node import BaseDashboardNode


class StepperMotorNode(BaseDashboardNode):
    """
    Stepper Motor ROS 2 node for science dashboard.
    Default node name: "karura_stepper_gui"
    
    Manages NEMA17 stepper motors with 27:1 gearbox via Phidget interface.
    """

    def __init__(self, node_name: str = "karura_stepper_gui"):
        super().__init__(node_name)

        # -------------------------------------------------------
        # Subscribers for Stepper Motor Status/Telemetry
        # -------------------------------------------------------
        
        # Current motor position (degrees)
        self.create_subscription(
            Float64,
            "/stepper/position",
            self._on_stepper_position,
            10,
        )
        
        # Current motor velocity (degrees per second)
        self.create_subscription(
            Float64,
            "/stepper/velocity",
            self._on_stepper_velocity,
            10,
        )
        
        # Current motor current (Amperes)
        self.create_subscription(
            Float64,
            "/stepper/current",
            self._on_stepper_current,
            10,
        )
        
        # Motor enabled status (True/False)
        self.create_subscription(
            Bool,
            "/stepper/enabled",
            self._on_stepper_enabled,
            10,
        )
        
        # Motor error status (error message or empty string)
        self.create_subscription(
            String,
            "/stepper/error",
            self._on_stepper_error,
            10,
        )
        
        # Target position (degrees) - for tracking commanded vs actual
        self.create_subscription(
            Float64,
            "/stepper/target_position",
            self._on_stepper_target_position,
            10,
        )
        
        # -------------------------------------------------------
        # Publishers for Stepper Motor Commands
        # (These will be created as needed by the ROS node)
        # -------------------------------------------------------
        # /stepper/set_position - Float64 (target position in degrees)
        # /stepper/set_velocity - Float64 (max velocity in deg/s)
        # /stepper/enable - Bool (enable/disable motor)
        # /stepper/stop - Bool (emergency stop)
        # /stepper/home - Bool (home position command)
    
    # -------------------------------------------------------
    # Callback methods - Topic handlers
    # -------------------------------------------------------
    
    def _on_stepper_position(self, msg: Float64):
        """Handle stepper position update."""
        self._dispatch("stepper_position", msg)
    
    def _on_stepper_velocity(self, msg: Float64):
        """Handle stepper velocity update."""
        self._dispatch("stepper_velocity", msg)
    
    def _on_stepper_current(self, msg: Float64):
        """Handle stepper current update."""
        self._dispatch("stepper_current", msg)
    
    def _on_stepper_enabled(self, msg: Bool):
        """Handle stepper enabled status update."""
        self._dispatch("stepper_enabled", msg)
    
    def _on_stepper_error(self, msg: String):
        """Handle stepper error status update."""
        self._dispatch("stepper_error", msg)
    
    def _on_stepper_target_position(self, msg: Float64):
        """Handle stepper target position update."""
        self._dispatch("stepper_target_position", msg)
