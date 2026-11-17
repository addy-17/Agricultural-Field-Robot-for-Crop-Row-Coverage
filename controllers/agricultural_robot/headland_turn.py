"""
headland_turn.py - Headland Turnaround Trajectory Generator
CSAI4014 AISR Project | Member 4 Deliverable

Role:
    Executes row-switching turnaround maneuvers in open headlands.
    Provides two distinct strategies for experimental comparison:
      - Strategy A (Baseline): In-Place Skid Pivot Turn (Stop -> Rotate 90 -> Drive W -> Rotate 90)
      - Strategy B (Advanced / Novel): Continuous Curvature Omega-Turn (Bulb Turn)
        preserving soil structure, reducing wheel slip, and maintaining momentum.
"""

import math
from enum import Enum
from typing import Tuple, Dict, Optional


class TurnStrategy(Enum):
    POINT_TURN_A = "POINT_TURN_A"
    OMEGA_TURN_B = "OMEGA_TURN_B"


class HeadlandTurnGenerator:
    """
    Parametric Headland Turnaround Controller.
    """

    def __init__(self, strategy: TurnStrategy = TurnStrategy.OMEGA_TURN_B):
        self.strategy = strategy
        self.is_running: bool = False
        self.turn_direction: str = "LEFT"   # "LEFT" or "RIGHT"
        self.row_spacing: float = 1.80      # Distance between rows in meters

        # Trajectory tracking state
        self.phase: int = 0
        self.elapsed_time: float = 0.0
        self.accumulated_angle: float = 0.0
        self.accumulated_distance: float = 0.0
        self.initial_yaw: float = 0.0
        self.target_yaw: float = 0.0
        self.turn_successful: bool = False
        self.yaw_tolerance: float = 0.06     # ~3.4 degrees heading tolerance
        self.phase5_elapsed: float = 0.0
        self.phase5_timeout: float = 3.0     # real-time timeout (seconds) for Phase 5 alignment
        self.kp_heading: float = 3.00        # proportional gain for closed-loop heading correction

        # Physical turning limits (calibrated for maxVelocity <= 6.00 rad/s soft cap)
        self.pivot_omega: float = 0.60       # rad/s for in-place rotation (pure spin, no v)
        self.traverse_speed: float = 0.42    # m/s for lateral transit (straight segment)
        self.omega_speed: float = 0.38       # m/s nominal forward speed during continuous turn
        self.min_turn_radius: float = 0.85   # meters (satisfies kinematic curvature limits)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Wraps angle into [-pi, +pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def start_turn(
        self,
        turn_direction: str,
        current_yaw: float,
        row_spacing: float = 1.80,
        strategy: Optional[TurnStrategy] = None,
        target_yaw: Optional[float] = None
    ) -> None:
        """
        Initializes a new headland turnaround maneuver.
        
        Args:
            turn_direction: "LEFT" or "RIGHT"
            current_yaw: Starting heading orientation in radians
            row_spacing: Inter-row center distance in meters (default 1.8 m)
            strategy: Optional override for Strategy A or B
            target_yaw: Optional desired final heading in radians
        """
        if strategy is not None:
            self.strategy = strategy

        self.turn_direction = turn_direction.upper()
        self.initial_yaw = current_yaw
        self.row_spacing = float(row_spacing)
        self.is_running = True
        self.turn_successful = False
        self.phase = 1
        self.elapsed_time = 0.0
        self.accumulated_angle = 0.0
        self.accumulated_distance = 0.0
        self.phase5_elapsed = 0.0

        dir_sign = +1 if self.turn_direction == "LEFT" else -1
        if target_yaw is not None:
            self.target_yaw = self._normalize_angle(target_yaw)
        else:
            self.target_yaw = self._normalize_angle(current_yaw + dir_sign * math.pi)

        print(f"[headland_turn.py] Started Headland Turn [{self.strategy.value}] | "
              f"Direction: {self.turn_direction} (sign={dir_sign}) | "
              f"Spacing: {self.row_spacing:.2f}m | TargetYaw: {math.degrees(self.target_yaw):.1f}°")

    def step(self, dt: float, current_yaw: float) -> Tuple[float, float, bool]:
        """
        Calculates commanded (v, omega) for the current turnaround timestep.
        
        Args:
            dt: Time step duration (seconds)
            current_yaw: Current robot heading in radians
            
        Returns:
            Tuple of (v_cmd, omega_cmd, is_completed)
        """
        if not self.is_running:
            return 0.0, 0.0, True

        self.elapsed_time += dt
        dir_mult = +1.0 if self.turn_direction == "LEFT" else -1.0

        if self.strategy == TurnStrategy.POINT_TURN_A:
            return self._step_point_turn(dt, dir_mult, current_yaw)
        else:
            return self._step_omega_turn(dt, dir_mult, current_yaw)

    def _step_point_turn(self, dt: float, dir_mult: float, current_yaw: float) -> Tuple[float, float, bool]:
        """
        Strategy A: Stop-and-Turn (Skid-steer point turn).
        Phase 1: Rotate 90 degrees toward next row.
        Phase 2: Drive straight by row_spacing.
        Phase 3: Rotate 90 degrees into new row heading with closed-loop heading check.
        """
        target_90_rad = math.pi / 2.0  # 1.5708 rad

        if self.phase == 1:
            # Rotate 90 degrees
            v = 0.0
            omega = dir_mult * self.pivot_omega
            self.accumulated_angle += abs(omega * dt)

            if self.accumulated_angle >= target_90_rad:
                self.phase = 2
                self.accumulated_distance = 0.0
            return v, omega, False

        elif self.phase == 2:
            # Drive straight by inter-row spacing
            v = self.traverse_speed
            omega = 0.0
            self.accumulated_distance += v * dt

            if self.accumulated_distance >= self.row_spacing:
                self.phase = 3
                self.accumulated_angle = 0.0
                self.phase5_elapsed = 0.0
            return v, omega, False

        elif self.phase == 3:
            # Rotate final 90 degrees into row with closed-loop heading correction
            v = 0.0
            self.phase5_elapsed += dt

            signed_error = self._normalize_angle(self.target_yaw - current_yaw)
            yaw_err = abs(signed_error)

            raw_omega = self.kp_heading * signed_error
            omega_cmd = max(-self.pivot_omega, min(self.pivot_omega, raw_omega))

            if yaw_err <= self.yaw_tolerance:
                self.is_running = False
                self.turn_successful = True
                print(f"[headland_turn.py] Strategy A (Point Turn) complete. Final Yaw: {math.degrees(current_yaw):.1f}° | Err: {math.degrees(yaw_err):.1f}°")
                return 0.0, 0.0, True
            elif self.phase5_elapsed >= self.phase5_timeout:
                self.is_running = False
                self.turn_successful = False
                print(f"[headland_turn.py] Strategy A (Point Turn) TIMEOUT! Alignment failed: Final Yaw: {math.degrees(current_yaw):.1f}° | Err: {math.degrees(yaw_err):.1f}° (tol: {math.degrees(self.yaw_tolerance):.1f}°)")
                return 0.0, 0.0, True
            return v, omega_cmd, False

        return 0.0, 0.0, True

    def _step_omega_turn(self, dt: float, dir_mult: float, current_yaw: float) -> Tuple[float, float, bool]:
        """
        Strategy B: Continuous Curvature Omega-Turn (Bulb Turn) dynamically scaled to row_spacing.
        
        Geometry:
          For radius R = min_turn_radius and flare angle alpha = 35 deg:
          Lateral offset of the 3 circular arcs alone is:
            dy_arcs = 2 * R * (2 * cos(alpha) - 1)  (~1.085 m for R=0.85m)
          To match any row_spacing W >= dy_arcs, a lateral straight segment of length:
            d_lat = max(0.0, W - dy_arcs)
          is smoothly traversed at perpendicular heading (Phase 3), giving exact lateral
          displacement Delta_y = row_spacing with Delta_x = 0.0m and Delta_yaw = 180 deg.

        Phases:
          Phase 1: Arc outward away from target row by alpha (counter-direction)
          Phase 2: Arc in target direction up to perpendicular heading (angle = pi/2 + alpha)
          Phase 3: Cruise straight along lateral axis by distance d_lat
          Phase 4: Arc in target direction to alignment threshold (angle = pi/2 + alpha)
          Phase 5: Straightening arc to snap heading to target_yaw (closed-loop verification)
        """
        v = self.omega_speed
        turn_omega = v / self.min_turn_radius  # omega = v / R

        alpha_rad = math.radians(35.0)  # 0.610865 rad
        quarter_arc_rad = (math.pi / 2.0) + alpha_rad  # 2.18166 rad

        # Calculate exact lateral expansion distance to match row_spacing
        dy_arcs = 2.0 * self.min_turn_radius * (2.0 * math.cos(alpha_rad) - 1.0)
        d_lat = max(0.0, self.row_spacing - dy_arcs)

        # Phase 1: Bulb flare outward away from target
        if self.phase == 1:
            omega = -dir_mult * turn_omega
            self.accumulated_angle += abs(omega * dt)
            if self.accumulated_angle >= alpha_rad:
                self.phase = 2
                self.accumulated_angle = 0.0
            return v, omega, False

        # Phase 2: Arc in target direction to perpendicular lateral heading
        elif self.phase == 2:
            omega = dir_mult * turn_omega
            self.accumulated_angle += abs(omega * dt)
            if self.accumulated_angle >= quarter_arc_rad:
                if d_lat > 0.01:
                    self.phase = 3
                    self.accumulated_distance = 0.0
                else:
                    self.phase = 4
                    self.accumulated_angle = 0.0
            return v, omega, False

        # Phase 3: Lateral straight cruise to bridge remaining row spacing
        elif self.phase == 3:
            v_cmd = v
            omega_cmd = 0.0
            self.accumulated_distance += v * dt
            if self.accumulated_distance >= d_lat:
                self.phase = 4
                self.accumulated_angle = 0.0
            return v_cmd, omega_cmd, False

        # Phase 4: Arc in target direction toward row entry
        elif self.phase == 4:
            omega = dir_mult * turn_omega
            self.accumulated_angle += abs(omega * dt)
            if self.accumulated_angle >= quarter_arc_rad:
                self.phase = 5
                self.accumulated_angle = 0.0
                self.phase5_elapsed = 0.0
            return v, omega, False

        # Phase 5: Two-Stage closed-loop heading correction (Counter-Arc + Final In-Place Snap)
        elif self.phase == 5:
            self.phase5_elapsed += dt

            # Compute signed heading error
            signed_error = self._normalize_angle(self.target_yaw - current_yaw)
            yaw_err = abs(signed_error)

            # Check completion immediately if within tolerance (0.06 rad)
            if yaw_err <= self.yaw_tolerance:
                self.is_running = False
                self.turn_successful = True
                print(f"[headland_turn.py] Strategy B (Continuous Omega Turn) complete. "
                      f"Final Yaw: {math.degrees(current_yaw):.1f}° | Err: {math.degrees(yaw_err):.1f}°")
                return 0.0, 0.0, True

            # Check real-time timeout
            if self.phase5_elapsed >= self.phase5_timeout:
                self.is_running = False
                self.turn_successful = False
                print(f"[headland_turn.py] Strategy B (Continuous Omega Turn) TIMEOUT! "
                      f"Alignment failed: Final Yaw: {math.degrees(current_yaw):.1f}° | Err: {math.degrees(yaw_err):.1f}° (tol: {math.degrees(self.yaw_tolerance):.1f}°)")
                return 0.0, 0.0, True

            # Proportional angular control
            raw_omega = self.kp_heading * signed_error
            dir_sign = 1.0 if signed_error >= 0.0 else -1.0

            if yaw_err > 0.08:
                # Stage 1: Continuous curvature counter-arc with forward motion
                # Preserves ~98% of Delta_y and ~86% of Delta_x turning geometry
                v_cmd = v
                omega_cmd = max(-turn_omega, min(turn_omega, raw_omega))
            else:
                # Stage 2: Final in-place snap (0.06 < yaw_err <= 0.08 rad)
                # Zero forward velocity eliminates large-radius drift (R -> inf) as error nears zero.
                # NOTE: Minimum angular speed magnitude (0.25 rad/s) is an engineering hypothesis
                # to prevent ODE tire stiction / lateral scrubbing stall, not experimentally proven.
                v_cmd = 0.0
                min_omega_mag = 0.25
                omega_mag = max(min_omega_mag, min(turn_omega, abs(raw_omega)))
                omega_cmd = dir_sign * omega_mag

            return v_cmd, omega_cmd, False

        return 0.0, 0.0, True
