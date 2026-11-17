"""
kinematics.py - Pioneer 3-AT Kinematics & Odometry Engine
CSAI4014 AISR Project | Member 1 Deliverable

Role:
    Provides mathematical modeling of the 4-wheel skid-steer mobile robot:
      - Forward & Inverse Kinematics
      - Instantaneous Center of Rotation (ICR)
      - Wheel velocity transformations with saturation limits
      - Dead-reckoning odometry estimation from wheel encoder feedback
"""

import math
from typing import Tuple, Dict


class Pioneer3ATKinematics:
    """
    Kinematic model for the 4-wheel skid-steer Pioneer 3-AT mobile robot.
    
    Standard Skid-Steer / Differential Drive Relations:
      v_left  = v - omega * (L / 2)
      v_right = v + omega * (L / 2)
      omega_left  = v_left  / r
      omega_right = v_right / r
    """

    # Physical vehicle constants
    WHEEL_RADIUS: float = 0.111       # meters
    TRACK_WIDTH: float = 0.400        # effective lateral distance between left & right wheels (m)
    WHEEL_BASE: float = 0.270         # longitudinal distance between front and rear axles (m)
    MAX_MOTOR_RADPS: float = 6.40     # max motor angular velocity limit (rad/s)
    SLIP_FACTOR: float = 0.95         # outdoor soil traction coefficient (accounts for tire slip)

    def __init__(self, track_width: float = TRACK_WIDTH, wheel_radius: float = WHEEL_RADIUS):
        self.L = track_width
        self.r = wheel_radius
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # radians

    def inverse_kinematics(self, v: float, omega: float) -> Tuple[float, float]:
        """
        Calculates left and right motor angular velocities (rad/s) from
        commanded linear velocity v (m/s) and angular velocity omega (rad/s).
        
        Returns:
            Tuple of (omega_left, omega_right) in rad/s.
        """
        effective_width = self.L / self.SLIP_FACTOR
        v_left = v - (omega * effective_width / 2.0)
        v_right = v + (omega * effective_width / 2.0)

        w_left = v_left / self.r
        w_right = v_right / self.r

        # Curvature-preserving scaling if maximum motor speed is exceeded
        max_req = max(abs(w_left), abs(w_right))
        if max_req > self.MAX_MOTOR_RADPS:
            scale = self.MAX_MOTOR_RADPS / max_req
            w_left *= scale
            w_right *= scale

        return w_left, w_right

    def forward_kinematics(self, w_left: float, w_right: float) -> Tuple[float, float]:
        """
        Calculates body linear velocity v (m/s) and angular velocity omega (rad/s)
        from measured wheel speeds w_left, w_right (rad/s).
        
        Returns:
            Tuple of (linear_velocity_v, angular_velocity_omega).
        """
        v_left = w_left * self.r
        v_right = w_right * self.r

        effective_width = self.L / self.SLIP_FACTOR
        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / effective_width

        return v, omega

    def update_odometry(self, v: float, omega: float, dt: float) -> Tuple[float, float, float]:
        """
        Dead-reckoning numerical integration using second-order Runge-Kutta / exact arc:
        Updates internal pose (x, y, theta) given velocities over time step dt.
        
        Returns:
            Tuple of (x, y, theta_rad).
        """
        if abs(omega) < 1e-6:
            # Straight-line approximation
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            # Circular arc integration
            theta_next = self.theta + omega * dt
            self.x += (v / omega) * (math.sin(theta_next) - math.sin(self.theta))
            self.y -= (v / omega) * (math.cos(theta_next) - math.cos(self.theta))
            self.theta = theta_next

        # Wrap theta into [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        return self.x, self.y, self.theta

    def reset_odometry(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        """Sets odometry pose to specified coordinates."""
        self.x = x
        self.y = y
        self.theta = theta

    def get_pose(self) -> Dict[str, float]:
        """Returns the current dead-reckoning pose."""
        return {
            "x": self.x,
            "y": self.y,
            "theta_rad": self.theta,
            "theta_deg": math.degrees(self.theta)
        }
