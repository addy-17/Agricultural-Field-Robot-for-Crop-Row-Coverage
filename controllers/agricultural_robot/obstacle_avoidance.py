"""
obstacle_avoidance.py - In-Row Safety Bubble & Reactive Obstacle Monitor
CSAI4014 AISR Project | Member 3 Deliverable

Role:
    Monitors front proximity distance sensors to prevent the Pioneer 3-AT from
    colliding with field obstacles (rocks, broken branches, animals, farm workers).
    Implements a graduated safety buffer:
      - Normal zone (d > 0.80 m): Full commanded speed
      - Caution zone (0.40 m < d <= 0.80 m): Proportional deceleration
      - Danger zone (d <= 0.40 m): Emergency active braking (stop)
"""

from typing import Tuple


class InRowObstacleSafety:
    """
    Graduated front obstacle safety controller.
    """

    STOP_THRESHOLD_METERS: float = 0.40      # Emergency stop limit
    CAUTION_THRESHOLD_METERS: float = 0.80   # Deceleration zone limit

    def __init__(self, stop_threshold: float = STOP_THRESHOLD_METERS, caution_threshold: float = CAUTION_THRESHOLD_METERS):
        self.stop_dist = stop_threshold
        self.caution_dist = caution_threshold

    def filter_velocity(self, v_desired: float, omega_desired: float, front_distance: float) -> Tuple[float, float, bool]:
        """
        Filters commanded velocities based on forward proximity.
        
        Args:
            v_desired: Nominal linear velocity requested by guidance controller
            omega_desired: Commanded steering rate
            front_distance: Minimum measured distance ahead in meters
            
        Returns:
            Tuple of (v_safe, omega_safe, is_emergency_stop)
        """
        if front_distance <= self.stop_dist:
            # Danger Zone: Immediate Full Stop
            return 0.0, 0.0, True

        elif front_distance < self.caution_dist:
            # Caution Zone: Linearly scale down forward velocity
            scale = (front_distance - self.stop_dist) / (self.caution_dist - self.stop_dist)
            scale = max(0.20, min(scale, 1.0))
            v_safe = v_desired * scale
            return v_safe, omega_desired, False

        else:
            # Safe Zone: Unhindered transit
            return v_desired, omega_desired, False
