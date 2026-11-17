"""
obstacle_avoidance.py - In-Row Safety Bubble & Reactive Obstacle Bypass
CSAI4014 AISR Project | Member 3 Deliverable

Role:
    Monitors front proximity distance sensors to prevent the Pioneer 3-AT from
    colliding with field obstacles (rocks, broken branches, debris).
    Implements a two-tier graduated safety and active bypass mechanism:
      - Normal zone (d > 1.00 m): Full commanded guidance speed
      - Caution zone (0.40 m < d <= 1.00 m): Graduated linear deceleration
      - Active Bypass (< 0.40 m): Autonomous multi-phase detour around obstacle
        followed by automated centerline re-acquisition by Stanley guidance.
"""

import math
from typing import Tuple


class InRowObstacleSafety:
    """
    Graduated front obstacle safety controller.
    """

    STOP_THRESHOLD_METERS: float = 0.80      # Emergency trigger distance (m)
    CAUTION_THRESHOLD_METERS: float = 1.50   # Caution slow-down threshold (m)

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
            Tuple of (v_safe, omega_safe, is_obstacle_triggered)
        """
        if front_distance <= self.stop_dist:
            # Danger Zone: Trigger bypass maneuver
            return 0.0, 0.0, True

        elif front_distance < self.caution_dist:
            # Caution Zone: Linearly scale down forward velocity (smooth deceleration)
            scale = (front_distance - self.stop_dist) / (self.caution_dist - self.stop_dist)
            scale = max(0.20, min(scale, 1.0))
            v_safe = v_desired * scale
            return v_safe, omega_desired, False

        else:
            # Safe Zone: Unhindered transit
            return v_desired, omega_desired, False


class ObstacleBypassManeuver:
    """
    Autonomous multi-phase bypass controller for in-row obstacles.
    Navigates safely around the obstacle within the inter-row lane (1.8m width)
    and hands control back to the row guidance system.
    """

    def __init__(self):
        self.is_active: bool = False
        self.phase: int = 0
        self.elapsed: float = 0.0
        self.direction: float = +1.0  # +1.0 = swerve LEFT, -1.0 = swerve RIGHT

    def start(self, swerve_left: bool = True) -> None:
        """
        Initializes the obstacle bypass maneuver.
        
        Args:
            swerve_left: True to detour via left clearance, False for right.
        """
        self.is_active = True
        self.phase = 1
        self.elapsed = 0.0
        self.direction = +1.0 if swerve_left else -1.0
        side = "LEFT" if swerve_left else "RIGHT"
        print(f"[ObstacleBypass] Initiating active obstacle detour to the {side}...")

    def step(self, dt: float) -> Tuple[float, float, bool]:
        """
        Executes one step of the bypass maneuver.
        
        Phases:
          Phase 1: Angle outward away from obstacle (~28 deg)
          Phase 2: Drive diagonally into safe lateral clearance zone
          Phase 3: Counter-steer back to parallel heading
          Phase 4: Drive straight alongside and past the obstacle
          Phase 5: Angle back toward lane centerline
          
        Returns:
            Tuple of (v_cmd, omega_cmd, is_completed)
        """
        if not self.is_active:
            return 0.0, 0.0, True

        self.elapsed += dt

        # Phase 1: Turn outward away from obstacle (~24 deg)
        if self.phase == 1:
            v = 0.20
            omega = self.direction * 0.40
            if self.elapsed >= 1.00:  # ~24 deg rotation
                self.phase = 2
                self.elapsed = 0.0
                print("[ObstacleBypass] Phase 2: Shifting laterally into safe clearance corridor...")
            return v, omega, False

        # Phase 2: Drive diagonally forward to build ~0.30m lateral offset
        elif self.phase == 2:
            v = 0.32
            omega = 0.0
            if self.elapsed >= 1.30:
                self.phase = 3
                self.elapsed = 0.0
                print("[ObstacleBypass] Phase 3: Aligning parallel to crop line...")
            return v, omega, False

        # Phase 3: Straighten heading parallel to row
        elif self.phase == 3:
            v = 0.20
            omega = -self.direction * 0.40
            if self.elapsed >= 1.00:
                self.phase = 4
                self.elapsed = 0.0
                print("[ObstacleBypass] Phase 4: Passing obstacle alongside...")
            return v, omega, False

        # Phase 4: Cruise straight alongside the obstacle until past it
        elif self.phase == 4:
            v = 0.35
            omega = 0.0
            if self.elapsed >= 1.80:
                self.phase = 5
                self.elapsed = 0.0
                print("[ObstacleBypass] Phase 5: Re-entering row center...")
            return v, omega, False

        # Phase 5: Angle gently back toward row centerline
        elif self.phase == 5:
            v = 0.25
            omega = -self.direction * 0.25
            if self.elapsed >= 1.00:
                self.is_active = False
                print("[ObstacleBypass] Detour complete! Stanley in-row follower resuming centerline tracking.")
                return 0.35, 0.0, True
            return v, omega, False

        self.is_active = False
        return 0.0, 0.0, True

    def reset(self) -> None:
        """Resets the bypass controller."""
        self.is_active = False
        self.phase = 0
        self.elapsed = 0.0
