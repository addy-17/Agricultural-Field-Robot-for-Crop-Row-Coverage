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

        # Physical turning limits (agile settings)
        self.pivot_omega: float = 1.00       # rad/s for in-place rotation
        self.traverse_speed: float = 0.65    # m/s for lateral transit
        self.omega_speed: float = 0.60       # m/s nominal forward speed during continuous turn
        self.min_turn_radius: float = 0.85   # meters (satisfies kinematic curvature limits)

    def start_turn(
        self,
        turn_direction: str,
        current_yaw: float,
        row_spacing: float = 1.80,
        strategy: Optional[TurnStrategy] = None
    ) -> None:
        """
        Initializes a new headland turnaround maneuver.
        
        Args:
            turn_direction: "LEFT" or "RIGHT"
            current_yaw: Starting heading orientation in radians
            row_spacing: Inter-row center distance in meters (default 1.8 m)
            strategy: Optional override for Strategy A or B
        """
        if strategy is not None:
            self.strategy = strategy

        self.turn_direction = turn_direction.upper()
        self.initial_yaw = current_yaw
        self.row_spacing = row_spacing
        self.is_running = True
        self.phase = 1
        self.elapsed_time = 0.0
        self.accumulated_angle = 0.0
        self.accumulated_distance = 0.0

        dir_sign = +1 if self.turn_direction == "LEFT" else -1
        print(f"[headland_turn.py] Started Headland Turn [{self.strategy.value}] | Direction: {self.turn_direction} (sign={dir_sign})")

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
            return self._step_point_turn(dt, dir_mult)
        else:
            return self._step_omega_turn(dt, dir_mult)

    def _step_point_turn(self, dt: float, dir_mult: float) -> Tuple[float, float, bool]:
        """
        Strategy A: Stop-and-Turn (Skid-steer point turn).
        Phase 1: Rotate 90 degrees toward next row.
        Phase 2: Drive straight by row_spacing (W = 1.8 m).
        Phase 3: Rotate 90 degrees into new row heading.
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
            return v, omega, False

        elif self.phase == 3:
            # Rotate final 90 degrees into row
            v = 0.0
            omega = dir_mult * self.pivot_omega
            self.accumulated_angle += abs(omega * dt)

            if self.accumulated_angle >= target_90_rad:
                # Turn finished!
                self.is_running = False
                print("[headland_turn.py] Strategy A (Point Turn) complete.")
                return 0.0, 0.0, True
            return v, omega, False

        return 0.0, 0.0, True

    def _step_omega_turn(self, dt: float, dir_mult: float) -> Tuple[float, float, bool]:
        """
        Strategy B: Continuous Curvature Omega-Turn (Bulb Turn).
        Maintains forward velocity v = 0.35 m/s throughout the entire maneuver:
        Phase 1 (Bulb flare): Arc outward away from target row by alpha = 35 deg (0.61 rad).
        Phase 2 (Main loop): Continuous arc in target direction by (180 + 2*alpha) deg (4.42 rad).
        Phase 3 (Entry alignment): Reverse arc by alpha (0.61 rad) to perfectly align with centerline.
        """
        v = self.omega_speed
        turn_omega = v / self.min_turn_radius  # omega = v / R = 0.35 / 0.85 = 0.412 rad/s

        alpha_rad = math.radians(35.0)  # 0.6108 rad
        main_loop_rad = math.pi + (2.0 * alpha_rad)  # 4.363 rad

        if self.phase == 1:
            # Flare outward away from target (counter-direction)
            omega = -dir_mult * turn_omega
            self.accumulated_angle += abs(omega * dt)

            if self.accumulated_angle >= alpha_rad:
                self.phase = 2
                self.accumulated_angle = 0.0
            return v, omega, False

        elif self.phase == 2:
            # Main sweeping turnaround arc into target row
            omega = dir_mult * turn_omega
            self.accumulated_angle += abs(omega * dt)

            if self.accumulated_angle >= main_loop_rad:
                self.phase = 3
                self.accumulated_angle = 0.0
            return v, omega, False

        elif self.phase == 3:
            # Final straightening arc to snap to row heading
            omega = -dir_mult * turn_omega
            self.accumulated_angle += abs(omega * dt)

            if self.accumulated_angle >= alpha_rad:
                self.is_running = False
                print("[headland_turn.py] Strategy B (Continuous Omega Turn) complete.")
                return v, 0.0, True
            return v, omega, False

        return 0.0, 0.0, True
