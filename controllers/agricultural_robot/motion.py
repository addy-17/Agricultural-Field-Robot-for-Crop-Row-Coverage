"""
motion.py - Robot Motion Interface for Pioneer 3-AT (and Skid-Steer Robots)
CSAI4014 AISR Project | Member 2 Deliverable

Role:
    Configures wheel/motor parameters and exposes a clean, high-level motion
    interface so other team members (planners, controllers, turn generators)
    can request velocity and turn commands without dealing with low-level
    motor names, register configurations, or wheel speed equations.

Robot Platform:
    Pioneer 3-AT (4-wheel skid-steer mobile robot in Webots)
    - 4 independently driven wheels: front-left, front-right, back-left, back-right
    - Effective track width (L): 0.40 m
    - Wheel radius (r): 0.111 m
    - Max motor angular velocity: 6.4 rad/s (approx 0.71 m/s linear velocity)
"""

import math
from typing import Dict, Tuple, Optional

# Try importing Webots controller; fall back to Mock mode for standalone testing
try:
    from controller import Robot, Motor
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False
    Robot = None
    Motor = None


class Pioneer3ATMotion:
    """
    High-level motion interface for the Pioneer 3-AT robot in Webots.
    
    Provides convenient motion primitives:
      - set_velocity(linear_velocity, angular_velocity)
      - move_forward(speed)
      - move_backward(speed)
      - turn_left(angular_speed)
      - turn_right(angular_speed)
      - stop()
    """

    # Physical vehicle constants (Pioneer 3-AT specifications)
    WHEEL_RADIUS: float = 0.111        # meters (111 mm)
    TRACK_WIDTH: float = 0.400         # meters (effective distance between left & right wheels)
    MAX_MOTOR_SPEED: float = 6.40      # rad/s (Pioneer 3-AT standard motor max velocity)
    DEFAULT_LINEAR_SPEED: float = 0.50 # m/s nominal agricultural transit speed
    DEFAULT_ANGULAR_SPEED: float = 0.8 # rad/s nominal turning rate

    def __init__(self, robot: Optional[object] = None):
        """
        Initialize motors and parameters on the robot.
        
        Args:
            robot: Webots Robot instance. If None, creates a new Robot() or enters Mock mode.
        """
        self.is_mock: bool = False
        self.robot = robot

        # Motor device references
        self.motors: Dict[str, Optional[object]] = {
            "front_left": None,
            "front_right": None,
            "back_left": None,
            "back_right": None,
        }

        # Current commanded setpoints
        self.cmd_linear_vel: float = 0.0
        self.cmd_angular_vel: float = 0.0
        self.left_motor_speed: float = 0.0
        self.right_motor_speed: float = 0.0

        self._init_motors()

    def _init_motors(self) -> None:
        """Locates and initializes the 4 wheel motors in velocity control mode."""
        if not WEBOTS_AVAILABLE:
            self.is_mock = True
            print("[motion.py] Webots controller not detected. Running in STANDALONE/MOCK mode.")
            return

        if self.robot is None:
            try:
                self.robot = Robot()
            except Exception as e:
                self.is_mock = True
                print(f"[motion.py] Could not instantiate Webots Robot ({e}). Running in MOCK mode.")
                return

        # Pioneer 3-AT standard Webots motor names
        motor_names = {
            "front_left": "front left wheel",
            "front_right": "front right wheel",
            "back_left": "back left wheel",
            "back_right": "back right wheel",
        }

        for key, name in motor_names.items():
            try:
                motor = self.robot.getDevice(name)
                if motor is not None:
                    # In Webots, setting position to infinity enables velocity control mode
                    motor.setPosition(float('+inf'))
                    motor.setVelocity(0.0)
                    self.motors[key] = motor
                else:
                    print(f"[motion.py] Warning: Motor '{name}' not found on robot.")
            except Exception as ex:
                print(f"[motion.py] Error initializing motor '{name}': {ex}")

        # If any motor failed to bind, fall back to mock
        if any(m is None for m in self.motors.values()):
            print("[motion.py] One or more motors unavailable. Entering hybrid/mock mode.")
            self.is_mock = True
        else:
            print("[motion.py] Successfully initialized all 4 Pioneer 3-AT wheel motors.")

    def calculate_wheel_speeds(self, linear_vel: float, angular_vel: float) -> Tuple[float, float]:
        """
        Inverse kinematics for skid-steer / differential drive:
        Maps desired body linear velocity (m/s) and angular velocity (rad/s)
        to left and right wheel angular velocities (rad/s).
        
        Equations:
            v_left  = v - omega * (L / 2)
            v_right = v + omega * (L / 2)
            omega_left  = v_left  / r
            omega_right = v_right / r
        """
        half_width = self.TRACK_WIDTH / 2.0
        v_left = linear_vel - (angular_vel * half_width)
        v_right = linear_vel + (angular_vel * half_width)

        w_left = v_left / self.WHEEL_RADIUS
        w_right = v_right / self.WHEEL_RADIUS

        # Symmetrical scaling if speed limit exceeded (preserves steering curvature)
        max_requested = max(abs(w_left), abs(w_right))
        if max_requested > self.MAX_MOTOR_SPEED:
            scale = self.MAX_MOTOR_SPEED / max_requested
            w_left *= scale
            w_right *= scale

        return w_left, w_right

    def set_velocity(self, linear_velocity: float, angular_velocity: float) -> Tuple[float, float]:
        """
        Primary interface: Sets robot body velocities.
        
        Args:
            linear_velocity: Desired forward speed in m/s (negative for reverse).
            angular_velocity: Desired counter-clockwise turning rate in rad/s (positive = turn left).
            
        Returns:
            Tuple of (left_motor_speed, right_motor_speed) in rad/s.
        """
        self.cmd_linear_vel = float(linear_velocity)
        self.cmd_angular_vel = float(angular_velocity)

        w_left, w_right = self.calculate_wheel_speeds(self.cmd_linear_vel, self.cmd_angular_vel)
        self.left_motor_speed = w_left
        self.right_motor_speed = w_right

        if not self.is_mock:
            # Set left motors
            if self.motors["front_left"]:
                self.motors["front_left"].setVelocity(w_left)
            if self.motors["back_left"]:
                self.motors["back_left"].setVelocity(w_left)

            # Set right motors
            if self.motors["front_right"]:
                self.motors["front_right"].setVelocity(w_right)
            if self.motors["back_right"]:
                self.motors["back_right"].setVelocity(w_right)

        return w_left, w_right

    def move_forward(self, speed: Optional[float] = None) -> Tuple[float, float]:
        """Drives the robot straight forward at the given or default linear speed."""
        spd = self.DEFAULT_LINEAR_SPEED if speed is None else speed
        return self.set_velocity(linear_velocity=abs(spd), angular_velocity=0.0)

    def move_backward(self, speed: Optional[float] = None) -> Tuple[float, float]:
        """Drives the robot straight backward at the given or default speed."""
        spd = self.DEFAULT_LINEAR_SPEED if speed is None else speed
        return self.set_velocity(linear_velocity=-abs(spd), angular_velocity=0.0)

    def turn_left(self, angular_speed: Optional[float] = None) -> Tuple[float, float]:
        """In-place counter-clockwise rotation (turns left)."""
        w = self.DEFAULT_ANGULAR_SPEED if angular_speed is None else angular_speed
        return self.set_velocity(linear_velocity=0.0, angular_velocity=abs(w))

    def turn_right(self, angular_speed: Optional[float] = None) -> Tuple[float, float]:
        """In-place clockwise rotation (turns right)."""
        w = self.DEFAULT_ANGULAR_SPEED if angular_speed is None else angular_speed
        return self.set_velocity(linear_velocity=0.0, angular_velocity=-abs(w))

    def stop(self) -> Tuple[float, float]:
        """Immediately commands zero velocity to all motors."""
        return self.set_velocity(linear_velocity=0.0, angular_velocity=0.0)

    def get_status(self) -> Dict[str, float]:
        """Returns the current motion commands and wheel speed telemetry."""
        return {
            "cmd_linear_vel_mps": self.cmd_linear_vel,
            "cmd_angular_vel_radps": self.cmd_angular_vel,
            "left_motor_radps": self.left_motor_speed,
            "right_motor_radps": self.right_motor_speed,
            "is_mock": float(self.is_mock)
        }


# Quick test when executed directly
if __name__ == "__main__":
    print("Testing Pioneer3ATMotion...")
    motion = Pioneer3ATMotion()
    
    # Test forward command
    wl, wr = motion.move_forward(0.5)
    print(f"Forward (0.5 m/s): Left={wl:.3f} rad/s, Right={wr:.3f} rad/s")
    assert math.isclose(wl, wr, rel_tol=1e-3), "Left and right speeds must match for straight line"

    # Test turn left command
    wl, wr = motion.turn_left(1.0)
    print(f"Turn Left (1.0 rad/s): Left={wl:.3f} rad/s, Right={wr:.3f} rad/s")
    assert wl < 0 < wr, "Left wheel must be negative and right wheel positive for left turn"

    # Test speed saturation
    wl, wr = motion.set_velocity(10.0, 0.0) # Requested speed far exceeds motor limit
    print(f"Saturated Forward: Left={wl:.3f} rad/s, Right={wr:.3f} rad/s (Max={motion.MAX_MOTOR_SPEED})")
    assert math.isclose(wl, motion.MAX_MOTOR_SPEED, rel_tol=1e-3), "Speed must be clamped to MAX_MOTOR_SPEED"

    # Test stop
    wl, wr = motion.stop()
    print(f"Stop: Left={wl:.3f} rad/s, Right={wr:.3f} rad/s")
    assert wl == 0.0 and wr == 0.0, "Motors must be 0 on stop"

    print("motion.py unit tests passed successfully!")

