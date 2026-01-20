"""
Phidget Stepper Motor Module

Interface for controlling NEMA17 stepper motors (1.8°, 1.68A, 27:1 gearbox)
using Phidget stepper controllers.

This module provides:
- Connection management to Phidget stepper devices
- Position and velocity control
- Current monitoring
- Error handling
"""

from typing import Optional, Callable
import logging

try:
    from Phidget22.Devices.Stepper import Stepper
    from Phidget22.Devices.Manager import Manager
    from Phidget22.Net import Server
except ImportError:
    raise ImportError(
        "Phidget22 library not found. Install with: pip install phidget22"
    )


logger = logging.getLogger(__name__)


class NEMA17StepperMotor:
    """
    Interface for NEMA17 stepper motor (1.8°, 1.68A) with 27:1 gearbox.
    
    Motor Specifications:
    - Step angle: 1.8 degrees
    - Max current: 1.68 Amperes
    - Gearbox ratio: 27:1
    - Full rotation: 200 steps (without gearbox)
    - With gearbox: 200 * 27 = 5400 steps per revolution
    - Degrees per step: 360 / 5400 = 0.0667 degrees
    """
    
    # Motor specifications
    STEP_ANGLE = 1.8  # degrees
    GEARBOX_RATIO = 27  # 27:1 gearbox
    STEPS_PER_REV = 200  # native steps per revolution
    TOTAL_STEPS_PER_REV = STEPS_PER_REV * GEARBOX_RATIO  # 5400
    DEGREES_PER_STEP = 360.0 / TOTAL_STEPS_PER_REV  # 0.0667 degrees
    MAX_CURRENT = 1.68  # Amperes
    
    def __init__(self, 
                 port_index: int = 0,
                 hub_port: Optional[int] = None,
                 channel: int = 0,
                 velocity_limit: float = 10000.0):
        """
        Initialize NEMA17 stepper motor controller.
        
        Args:
            port_index: USB port index for Phidget hub (usually 0 for first device)
            hub_port: Hub port number (if connected via hub). None for direct connection.
            channel: Channel index on the stepper controller (usually 0)
            velocity_limit: Maximum velocity in steps/second (default 10000)
        """
        self.port_index = port_index
        self.hub_port = hub_port
        self.channel = channel
        self.velocity_limit = velocity_limit
        
        self.stepper: Optional[Stepper] = None
        self.is_connected = False
        self.is_enabled = False
        
        self._position_callback: Optional[Callable[[float], None]] = None
        self._velocity_callback: Optional[Callable[[float], None]] = None
        self._current_callback: Optional[Callable[[float], None]] = None
        self._error_callback: Optional[Callable[[str], None]] = None
    
    def connect(self) -> bool:
        """
        Connect to the Phidget stepper motor device.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.stepper = Stepper()
            
            # Set device selection if using hub
            if self.hub_port is not None:
                self.stepper.setHubPort(self.hub_port)
            
            self.stepper.setChannel(self.channel)
            
            # Set up event handlers
            self.stepper.setOnPositionChangeHandler(self._on_position_change)
            self.stepper.setOnVelocityChangeHandler(self._on_velocity_change)
            self.stepper.setOnCurrentChangeHandler(self._on_current_change)
            
            # Open and wait for attachment
            self.stepper.openWaitForAttachment(5000)
            
            self.is_connected = True
            logger.info(f"Connected to stepper motor on port {self.port_index}, channel {self.channel}")
            
            # Set initial velocity limit
            self.set_velocity_limit(self.velocity_limit)
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to stepper motor: {e}")
            self._trigger_error_callback(f"Connection failed: {str(e)}")
            return False
    
    def disconnect(self) -> bool:
        """
        Disconnect from the Phidget stepper motor device.
        
        Returns:
            True if disconnect successful, False otherwise
        """
        try:
            if self.stepper is not None:
                self.disable()
                self.stepper.close()
                self.is_connected = False
                logger.info("Disconnected from stepper motor")
                return True
        except Exception as e:
            logger.error(f"Error disconnecting stepper motor: {e}")
        
        return False
    
    def enable(self) -> bool:
        """
        Enable the stepper motor.
        
        Returns:
            True if enabled successfully, False otherwise
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            self.stepper.setEngaged(True)
            self.is_enabled = True
            logger.info("Stepper motor enabled")
            return True
            
        except Exception as e:
            logger.error(f"Failed to enable stepper motor: {e}")
            self._trigger_error_callback(f"Enable failed: {str(e)}")
            return False
    
    def disable(self) -> bool:
        """
        Disable the stepper motor.
        
        Returns:
            True if disabled successfully, False otherwise
        """
        try:
            if not self.is_connected or self.stepper is None:
                return False
            
            self.stepper.setEngaged(False)
            self.is_enabled = False
            logger.info("Stepper motor disabled")
            return True
            
        except Exception as e:
            logger.error(f"Failed to disable stepper motor: {e}")
            self._trigger_error_callback(f"Disable failed: {str(e)}")
            return False
    
    def set_target_position(self, position_degrees: float) -> bool:
        """
        Set the target position for the stepper motor.
        
        Args:
            position_degrees: Target position in degrees
            
        Returns:
            True if set successfully, False otherwise
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            # Convert degrees to steps
            target_steps = position_degrees / self.DEGREES_PER_STEP
            
            self.stepper.setTargetPosition(target_steps)
            logger.debug(f"Set target position to {position_degrees}° ({target_steps} steps)")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set target position: {e}")
            self._trigger_error_callback(f"Set position failed: {str(e)}")
            return False
    
    def set_velocity_limit(self, velocity_deg_per_sec: float) -> bool:
        """
        Set the velocity limit for the stepper motor.
        
        Args:
            velocity_deg_per_sec: Velocity limit in degrees per second
            
        Returns:
            True if set successfully, False otherwise
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            # Convert deg/s to steps/s
            velocity_steps_per_sec = velocity_deg_per_sec / self.DEGREES_PER_STEP
            
            # Clamp to valid range
            velocity_steps_per_sec = max(0, min(velocity_steps_per_sec, self.velocity_limit))
            
            self.stepper.setVelocityLimit(velocity_steps_per_sec)
            logger.debug(f"Set velocity limit to {velocity_deg_per_sec}°/s")
            return True
            
        except Exception as e:
            logger.error(f"Failed to set velocity limit: {e}")
            self._trigger_error_callback(f"Set velocity failed: {str(e)}")
            return False
    
    def stop(self) -> bool:
        """
        Stop the stepper motor immediately.
        
        Returns:
            True if stopped successfully, False otherwise
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            self.stepper.setVelocityLimit(0)
            logger.info("Stepper motor stopped")
            return True
            
        except Exception as e:
            logger.error(f"Failed to stop stepper motor: {e}")
            self._trigger_error_callback(f"Stop failed: {str(e)}")
            return False
    
    def home(self, home_position_degrees: float = 0.0) -> bool:
        """
        Home the stepper motor to a reference position.
        
        Args:
            home_position_degrees: Home position in degrees (default 0.0)
            
        Returns:
            True if homed successfully, False otherwise
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            # Set current position as home
            home_steps = home_position_degrees / self.DEGREES_PER_STEP
            self.stepper.setPosition(home_steps)
            logger.info(f"Homed stepper motor to {home_position_degrees}°")
            return True
            
        except Exception as e:
            logger.error(f"Failed to home stepper motor: {e}")
            self._trigger_error_callback(f"Homing failed: {str(e)}")
            return False
    
    def get_position(self) -> Optional[float]:
        """
        Get the current position of the stepper motor.
        
        Returns:
            Current position in degrees, or None if error
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            current_steps = self.stepper.getPosition()
            current_degrees = current_steps * self.DEGREES_PER_STEP
            return current_degrees
            
        except Exception as e:
            logger.error(f"Failed to get position: {e}")
            self._trigger_error_callback(f"Get position failed: {str(e)}")
            return None
    
    def get_velocity(self) -> Optional[float]:
        """
        Get the current velocity of the stepper motor.
        
        Returns:
            Current velocity in degrees per second, or None if error
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            current_steps_per_sec = self.stepper.getVelocity()
            current_deg_per_sec = current_steps_per_sec * self.DEGREES_PER_STEP
            return current_deg_per_sec
            
        except Exception as e:
            logger.error(f"Failed to get velocity: {e}")
            self._trigger_error_callback(f"Get velocity failed: {str(e)}")
            return None
    
    def get_current(self) -> Optional[float]:
        """
        Get the current motor current draw.
        
        Returns:
            Current draw in Amperes, or None if error
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            current_amps = self.stepper.getCurrent()
            return current_amps
            
        except Exception as e:
            logger.error(f"Failed to get current: {e}")
            self._trigger_error_callback(f"Get current failed: {str(e)}")
            return None
    
    def is_moving(self) -> Optional[bool]:
        """
        Check if the stepper motor is currently moving.
        
        Returns:
            True if moving, False if stationary, None if error
        """
        try:
            if not self.is_connected or self.stepper is None:
                raise RuntimeError("Motor not connected")
            
            # Check if velocity is non-zero or target differs from current position
            velocity = self.stepper.getVelocity()
            return velocity != 0.0
            
        except Exception as e:
            logger.error(f"Failed to check if moving: {e}")
            return None
    
    def register_position_callback(self, callback: Callable[[float], None]):
        """Register callback for position changes."""
        self._position_callback = callback
    
    def register_velocity_callback(self, callback: Callable[[float], None]):
        """Register callback for velocity changes."""
        self._velocity_callback = callback
    
    def register_current_callback(self, callback: Callable[[float], None]):
        """Register callback for current changes."""
        self._current_callback = callback
    
    def register_error_callback(self, callback: Callable[[str], None]):
        """Register callback for error events."""
        self._error_callback = callback
    
    # -------------------------------------------------------
    # Internal event handlers
    # -------------------------------------------------------
    
    def _on_position_change(self, steps: float):
        """Internal handler for position changes."""
        degrees = steps * self.DEGREES_PER_STEP
        logger.debug(f"Position changed to {degrees}°")
        self._trigger_position_callback(degrees)
    
    def _on_velocity_change(self, steps_per_sec: float):
        """Internal handler for velocity changes."""
        deg_per_sec = steps_per_sec * self.DEGREES_PER_STEP
        logger.debug(f"Velocity changed to {deg_per_sec}°/s")
        self._trigger_velocity_callback(deg_per_sec)
    
    def _on_current_change(self, current_amps: float):
        """Internal handler for current changes."""
        logger.debug(f"Current changed to {current_amps}A")
        self._trigger_current_callback(current_amps)
    
    # -------------------------------------------------------
    # Callback triggers
    # -------------------------------------------------------
    
    def _trigger_position_callback(self, degrees: float):
        """Trigger registered position callback."""
        if self._position_callback:
            try:
                self._position_callback(degrees)
            except Exception as e:
                logger.error(f"Error in position callback: {e}")
    
    def _trigger_velocity_callback(self, deg_per_sec: float):
        """Trigger registered velocity callback."""
        if self._velocity_callback:
            try:
                self._velocity_callback(deg_per_sec)
            except Exception as e:
                logger.error(f"Error in velocity callback: {e}")
    
    def _trigger_current_callback(self, current_amps: float):
        """Trigger registered current callback."""
        if self._current_callback:
            try:
                self._current_callback(current_amps)
            except Exception as e:
                logger.error(f"Error in current callback: {e}")
    
    def _trigger_error_callback(self, error_msg: str):
        """Trigger registered error callback."""
        if self._error_callback:
            try:
                self._error_callback(error_msg)
            except Exception as e:
                logger.error(f"Error in error callback: {e}")
