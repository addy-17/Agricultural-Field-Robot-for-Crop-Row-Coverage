"""
row_follower.py - In-Row Guidance & Steering Controllers
CSAI4014 AISR Project | Member 3 Deliverable

Role:
    Maintains the Pioneer 3-AT centered between parallel crop rows.
    Provides two selectable guidance methods for experimental comparison:
      - Method A (Baseline): Dual-Loop Lateral PID Guidance
      - Method B (Advanced / Novel): Non-Linear Stanley Guidance with Adaptive Velocity
"""

import math
from typing import Tuple
from pid_controller import PIDController


class MethodAPIDRowFollower:
    """
    Method A (Baseline): Dual-Loop In-Row PID Guidance.
    
    Equations:
      omega_lateral = Kp * (-e_y) + Ki * int(-e_y)dt + Kd * d(-e_y)/dt
      omega_heading = K_yaw * (-e_psi)
      omega_cmd = clamp(omega_lateral + omega_heading, -w_max, +w_max)
      v_cmd = V_NOMINAL (constant forward speed)
    """

    def __init__(
        self,
        nominal_speed: float = 0.55,
        kp_lat: float = 1.35,
        ki_lat: float = 0.05,
        kd_lat: float = 0.22,
        kp_yaw: float = 0.65,
        max_angular_vel: float = 0.50   # keeps v+omega*halftrack < 5.80 rad/s
    ):
        self.v_nominal = float(nominal_speed)
        self.kp_yaw = float(kp_yaw)
        self.max_omega = float(max_angular_vel)

        self.pid_lateral = PIDController(
            kp=kp_lat,
            ki=ki_lat,
            kd=kd_lat,
            output_limit=self.max_omega,
            integral_limit=0.50
        )

    def reset(self) -> None:
        self.pid_lateral.reset()

    def compute(self, lateral_error: float, heading_error: float, dt: float) -> Tuple[float, float]:
        """
        Computes commanded body velocities (v, omega) using baseline PID.
        
        Args:
            lateral_error: e_y (m), positive when shifted toward left wall.
            heading_error: e_psi (rad), positive when angled toward left wall.
            dt: Time step duration (s).
            
        Returns:
            Tuple of (v_cmd, omega_cmd).
        """
        # Steering effort to eliminate lateral error:
        # e_y > 0 (shifted left) requires omega < 0 (turn right)
        omega_lat = -self.pid_lateral.compute(error=lateral_error, dt=dt)

        # Proportional alignment to eliminate heading error:
        # e_psi > 0 (angled left) requires omega < 0 (turn right)
        omega_yaw = -self.kp_yaw * heading_error

        omega_cmd = omega_lat + omega_yaw
        omega_cmd = max(-self.max_omega, min(omega_cmd, self.max_omega))

        v_cmd = self.v_nominal
        return v_cmd, omega_cmd


class MethodBStanleyRowFollower:
    """
    Method B (Advanced): Non-Linear Stanley Guidance with Adaptive Speed.
    
    Equations:
      Steering Angle: delta = e_psi + arctan( (k_e * (-e_y)) / (v + epsilon) )
      omega_cmd = (v / L) * tan(delta)
      
      Adaptive Velocity:
        v_cmd = v_nominal / (1.0 + k_v1 * |e_y| + k_v2 * |e_psi|)
        (Slows down smoothly during large disturbances to prevent crop collision)
    """

    def __init__(
        self,
        nominal_speed: float = 0.55,
        min_speed: float = 0.28,
        stanley_gain: float = 1.70,
        heading_gain: float = 0.90,
        wheelbase: float = 0.39,
        max_angular_vel: float = 0.50   # keeps v+omega*halftrack < 5.80 rad/s
    ):
        self.v_nominal = float(nominal_speed)
        self.v_min = float(min_speed)
        self.k_e = float(stanley_gain)
        self.k_h = float(heading_gain)
        self.L = float(wheelbase)
        self.max_omega = float(max_angular_vel)
        self.epsilon = 0.10

    def reset(self) -> None:
        pass

    def compute(self, lateral_error: float, heading_error: float, dt: float) -> Tuple[float, float]:
        """
        Computes commanded body velocities (v, omega) using Stanley guidance.
        
        Conventions:
          - lateral_error > 0 when shifted left -> requires delta < 0 (steer right)
          - heading_error > 0 when angled left  -> requires delta < 0 (steer right)
        """
        # 1. Adaptive speed modulation based on tracking disturbance
        error_magnitude = abs(lateral_error) + (0.5 * abs(heading_error))
        v_cmd = self.v_nominal / (1.0 + (1.6 * error_magnitude))
        v_cmd = max(self.v_min, min(v_cmd, self.v_nominal))

        # 2. Stanley cross-track correction angle
        cross_track_term = math.atan2(self.k_e * lateral_error, v_cmd + self.epsilon)

        # 3. Heading alignment term
        heading_term = self.k_h * heading_error

        # 4. Equivalent steering angle delta (negative turns right to eliminate positive errors)
        delta = -(heading_term + cross_track_term)

        # 5. Convert steering angle to body yaw rate omega
        omega_cmd = (v_cmd / self.L) * math.tan(delta)
        omega_cmd = max(-self.max_omega, min(omega_cmd, self.max_omega))

        return v_cmd, omega_cmd
