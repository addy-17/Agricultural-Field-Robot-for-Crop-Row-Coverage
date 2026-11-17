"""
pid_controller.py - Industrial Grade PID Controller with Anti-Windup
CSAI4014 AISR Project | Member 3 Deliverable

Role:
    General-purpose PID controller featuring:
      - Proportional, Integral, and Derivative control
      - Integral anti-windup clamping
      - First-order derivative low-pass filtering to suppress sensor noise
      - Symmetric/asymmetric output saturation limits
"""

import math
from typing import Optional


class PIDController:
    """
    Robust discrete-time PID controller with anti-windup and derivative filtering.
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = 1.5,
        integral_limit: Optional[float] = None,
        filter_alpha: float = 0.80
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)

        self.output_limit = abs(float(output_limit))
        self.integral_limit = self.output_limit if integral_limit is None else abs(float(integral_limit))
        self.filter_alpha = float(filter_alpha)  # Derivative low-pass coefficient (0 to 1)

        # Controller internal state
        self.integral: float = 0.0
        self.prev_error: float = 0.0
        self.filtered_derivative: float = 0.0
        self.first_step: bool = True

    def reset(self) -> None:
        """Resets the integral accumulator and derivative memory."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_derivative = 0.0
        self.first_step = True

    def compute(self, error: float, dt: float) -> float:
        """
        Computes the PID control effort for the given error and time delta dt.
        
        Args:
            error: Setpoint - Current_Measurement (or tracking error)
            dt: Time step duration in seconds
            
        Returns:
            Clamped control command u(t).
        """
        if dt <= 0.0:
            return 0.0

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Integral term with anti-windup clamping
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))
        i_term = self.ki * self.integral

        # 3. Derivative term with low-pass filtering
        if self.first_step:
            derivative = 0.0
            self.first_step = False
        else:
            raw_derivative = (error - self.prev_error) / dt
            self.filtered_derivative = (self.filter_alpha * self.filtered_derivative) + ((1.0 - self.filter_alpha) * raw_derivative)
            derivative = self.filtered_derivative

        d_term = self.kd * derivative
        self.prev_error = error

        # 4. Total unconstrained output
        u_raw = p_term + i_term + d_term

        # 5. Output saturation clamp
        u_clamped = max(-self.output_limit, min(u_raw, self.output_limit))
        return u_clamped
