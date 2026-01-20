"""
Example Stepper Motor ROS 2 Publisher Node

This is an example implementation showing how to create a ROS 2 node that:
1. Controls the NEMA17 stepper motor via Phidget controller
2. Publishes telemetry topics that the dashboard subscribes to
3. Subscribes to command topics from the dashboard

To use this in your project:
1. Create a package in your ROS 2 workspace
2. Copy this file as your main node
3. Update the package.xml and setup.py accordingly
4. Launch alongside the dashboard GUI
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, String
import logging

from dashboard.core.phidget_stepper import NEMA17StepperMotor

logger = logging.getLogger(__name__)


class StepperMotorPublisherNode(Node):
    """
    ROS 2 node for controlling NEMA17 stepper motor via Phidget.
    
    Publishes:
    - /stepper/position (Float64)
    - /stepper/velocity (Float64)
    - /stepper/current (Float64)
    - /stepper/enabled (Bool)
    - /stepper/error (String)
    - /stepper/target_position (Float64)
    
    Subscribes to:
    - /stepper/set_position (Float64)
    - /stepper/set_velocity (Float64)
    - /stepper/enable (Bool)
    - /stepper/stop (Bool)
    - /stepper/home (Bool)
    """
    
    def __init__(self):
        super().__init__('stepper_motor_node')
        
        # Declare parameters
        self.declare_parameter('port_index', 0)
        self.declare_parameter('hub_port', -1)  # -1 means no hub
        self.declare_parameter('channel', 0)
        self.declare_parameter('velocity_limit', 10000.0)
        self.declare_parameter('publish_rate_hz', 10.0)
        
        # Get parameters
        port_index = self.get_parameter('port_index').value
        hub_port = self.get_parameter('hub_port').value
        channel = self.get_parameter('channel').value
        velocity_limit = self.get_parameter('velocity_limit').value
        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        
        # Convert hub_port (-1 means None)
        hub_port = hub_port if hub_port >= 0 else None
        
        # Initialize stepper motor
        self.motor = NEMA17StepperMotor(
            port_index=port_index,
            hub_port=hub_port,
            channel=channel,
            velocity_limit=velocity_limit
        )
        
        # Publishers
        self.position_pub = self.create_publisher(Float64, '/stepper/position', 10)
        self.velocity_pub = self.create_publisher(Float64, '/stepper/velocity', 10)
        self.current_pub = self.create_publisher(Float64, '/stepper/current', 10)
        self.enabled_pub = self.create_publisher(Bool, '/stepper/enabled', 10)
        self.error_pub = self.create_publisher(String, '/stepper/error', 10)
        self.target_position_pub = self.create_publisher(Float64, '/stepper/target_position', 10)
        
        # Subscribers
        self.create_subscription(Float64, '/stepper/set_position', self._on_set_position, 10)
        self.create_subscription(Float64, '/stepper/set_velocity', self._on_set_velocity, 10)
        self.create_subscription(Bool, '/stepper/enable', self._on_enable, 10)
        self.create_subscription(Bool, '/stepper/stop', self._on_stop, 10)
        self.create_subscription(Bool, '/stepper/home', self._on_home, 10)
        
        # Register motor callbacks
        self.motor.register_position_callback(self._publish_position)
        self.motor.register_velocity_callback(self._publish_velocity)
        self.motor.register_current_callback(self._publish_current)
        self.motor.register_error_callback(self._publish_error)
        
        # State tracking
        self.last_error = ""
        
        # Timer for publishing telemetry
        timer_period = 1.0 / publish_rate_hz
        self.create_timer(timer_period, self._publish_telemetry)
        
        # Connect to motor
        self.get_logger().info("Connecting to stepper motor...")
        if self.motor.connect():
            self.get_logger().info("Successfully connected to stepper motor")
            self._publish_error("")  # Clear any errors
        else:
            self.get_logger().error("Failed to connect to stepper motor")
            self._publish_error("Failed to connect to device")
    
    def _publish_telemetry(self):
        """Publish current motor telemetry."""
        if not self.motor.is_connected:
            return
        
        # Publish position
        pos = self.motor.get_position()
        if pos is not None:
            msg = Float64()
            msg.data = pos
            self.position_pub.publish(msg)
        
        # Publish velocity
        vel = self.motor.get_velocity()
        if vel is not None:
            msg = Float64()
            msg.data = vel
            self.velocity_pub.publish(msg)
        
        # Publish current
        current = self.motor.get_current()
        if current is not None:
            msg = Float64()
            msg.data = current
            self.current_pub.publish(msg)
        
        # Publish enabled status
        msg = Bool()
        msg.data = self.motor.is_enabled
        self.enabled_pub.publish(msg)
    
    def _publish_position(self, position_degrees: float):
        """Callback: publish position from motor event."""
        msg = Float64()
        msg.data = position_degrees
        self.position_pub.publish(msg)
    
    def _publish_velocity(self, velocity_deg_per_sec: float):
        """Callback: publish velocity from motor event."""
        msg = Float64()
        msg.data = velocity_deg_per_sec
        self.velocity_pub.publish(msg)
    
    def _publish_current(self, current_amps: float):
        """Callback: publish current from motor event."""
        msg = Float64()
        msg.data = current_amps
        self.current_pub.publish(msg)
    
    def _publish_error(self, error_msg: str):
        """Callback: publish error from motor event."""
        if error_msg != self.last_error:
            self.last_error = error_msg
            msg = String()
            msg.data = error_msg
            self.error_pub.publish(msg)
            
            if error_msg:
                self.get_logger().error(f"Motor error: {error_msg}")
    
    def _on_set_position(self, msg: Float64):
        """Handle set position command."""
        try:
            if self.motor.is_connected and self.motor.is_enabled:
                self.motor.set_target_position(msg.data)
                self.get_logger().info(f"Set target position to {msg.data}°")
                # Publish target position
                target_msg = Float64()
                target_msg.data = msg.data
                self.target_position_pub.publish(target_msg)
            else:
                self.get_logger().warn("Cannot set position: motor not connected or enabled")
        except Exception as e:
            self.get_logger().error(f"Error setting position: {e}")
    
    def _on_set_velocity(self, msg: Float64):
        """Handle set velocity command."""
        try:
            if self.motor.is_connected:
                self.motor.set_velocity_limit(msg.data)
                self.get_logger().info(f"Set velocity limit to {msg.data}°/s")
            else:
                self.get_logger().warn("Cannot set velocity: motor not connected")
        except Exception as e:
            self.get_logger().error(f"Error setting velocity: {e}")
    
    def _on_enable(self, msg: Bool):
        """Handle enable/disable command."""
        try:
            if msg.data:
                if self.motor.is_connected and not self.motor.is_enabled:
                    self.motor.enable()
                    self.get_logger().info("Motor enabled")
            else:
                if self.motor.is_connected and self.motor.is_enabled:
                    self.motor.disable()
                    self.get_logger().info("Motor disabled")
        except Exception as e:
            self.get_logger().error(f"Error enabling/disabling motor: {e}")
    
    def _on_stop(self, msg: Bool):
        """Handle emergency stop command."""
        try:
            if msg.data and self.motor.is_connected:
                self.motor.stop()
                self.get_logger().warn("Emergency stop activated")
        except Exception as e:
            self.get_logger().error(f"Error during emergency stop: {e}")
    
    def _on_home(self, msg: Bool):
        """Handle home command."""
        try:
            if msg.data and self.motor.is_connected:
                self.motor.home(0.0)
                self.get_logger().info("Motor homed")
        except Exception as e:
            self.get_logger().error(f"Error homing motor: {e}")
    
    def destroy_node(self):
        """Clean up on shutdown."""
        self.get_logger().info("Shutting down stepper motor node...")
        if self.motor.is_connected:
            self.motor.disable()
            self.motor.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = StepperMotorPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
