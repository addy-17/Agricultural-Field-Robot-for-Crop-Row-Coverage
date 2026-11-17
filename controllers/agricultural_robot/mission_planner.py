"""
mission_planner.py - Boustrophedon Coverage Mission Planner & Supervisor FSM
CSAI4014 AISR Project | Member 1 Deliverable

Role:
    Coordinates complete crop-row coverage (CCPP) across all field lanes.
    Maintains a Finite State Machine (FSM) supervising:
      - In-row tracking
      - Headland exit & transition
      - Row-switching turnaround execution
      - Row entry and re-alignment
      - Obstacle stopping & resumption
      - Coverage progress tracking & mission termination
"""

import json
import math
import os
from enum import Enum
from typing import Dict, List, Tuple, Optional


class MissionState(Enum):
    INITIALIZE = "INITIALIZE"
    IN_ROW_TRACKING = "IN_ROW_TRACKING"
    HEADLAND_TRANSITION = "HEADLAND_TRANSITION"
    HEADLAND_TURN = "HEADLAND_TURN"
    ROW_ENTRY = "ROW_ENTRY"
    OBSTACLE_AVOIDANCE = "OBSTACLE_AVOIDANCE"
    MISSION_COMPLETE = "MISSION_COMPLETE"


class CoverageMissionPlanner:
    """
    Boustrophedon Field Coverage Planner and Supervisor State Machine.
    """

    def __init__(self, config_path: Optional[str] = None):
        # Default field configuration matching Member 1 coordinates
        self.field_length_x = 16.0
        self.row_spacing_z = 2.20
        # Trigger headland transition BEFORE the physical row end (rows run -8 to +8).
        # Robot exits rows at x=±8; we begin the exit drive at x=±7.5 and need 2 s
        # to clear into open headland before the turn starts.
        self.headland_x_pos = 7.50   # Positive headland approach trigger (meters)
        self.headland_x_neg = -7.50  # Negative headland approach trigger (meters)

        # Pre-planned coverage sequence of navigation lanes:
        # Lane 2 (z=0.0, heading +X) -> Lane 3 (z=+2.2, heading -X) -> Lane 1 (z=-2.2, heading +X)
        self.lanes: List[Dict[str, any]] = [
            {"id": "Lane_2", "z_center": 0.00, "direction": +1, "start_x": -7.0, "end_x": +7.5},
            {"id": "Lane_3", "z_center": 2.20, "direction": -1, "start_x": +7.5, "end_x": -7.5},
            {"id": "Lane_1", "z_center": -2.20, "direction": +1, "start_x": -7.5, "end_x": +7.5},
        ]

        self.current_lane_idx: int = 0
        self.state: MissionState = MissionState.INITIALIZE
        self.previous_state: MissionState = MissionState.INITIALIZE

        # Headland exit drive: keep state = HEADLAND_TRANSITION for this many FSM steps
        # so the robot drives ~2 m into open headland BEFORE starting the turn.
        # At v=0.35 m/s and 32 ms time-step: 60 steps ≈ 1.92 s ≈ 0.67 m clearance.
        # We use 75 steps (2.4 s) so the robot is well clear of the row walls.
        self.headland_exit_ticks_required: int = 75
        self.headland_exit_ticks: int = 0

        # Row entry state supervision
        self.row_entry_ticks: int = 0
        self.max_row_entry_ticks: int = 250   # ~8.0 s maximum entry acquisition window
        self.is_lost: bool = False

        # Telemetry & Coverage Metrics
        self.distance_traversed: float = 0.0
        self.total_planned_distance: float = len(self.lanes) * self.field_length_x
        self.completed_lanes: List[str] = []
        self.last_pose: Optional[Tuple[float, float]] = None

        self._load_config(config_path)

    def _load_config(self, config_path: Optional[str]) -> None:
        """Loads field dimensions from docs/field_config.json if available."""
        if not config_path:
            # Check standard relative location
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            candidate = os.path.join(base_dir, "docs", "field_config.json")
            if os.path.exists(candidate):
                config_path = candidate

        if config_path and os.path.exists(config_path):
            try:
                with open(config_path, "r") as f:
                    cfg = json.load(f)
                    crop_info = cfg.get("crop_rows", {})
                    self.field_length_x = crop_info.get("length", 16.0)
                    self.row_spacing_z = crop_info.get("center_spacing", 1.8)
                    print(f"[mission_planner.py] Loaded field config from {config_path}")
            except Exception as e:
                print(f"[mission_planner.py] Warning: Could not parse config ({e}). Using defaults.")

    def get_current_lane(self) -> Dict[str, any]:
        """Returns the active navigation lane dictionary."""
        if self.current_lane_idx < len(self.lanes):
            return self.lanes[self.current_lane_idx]
        return self.lanes[-1]

    def get_next_lane(self) -> Optional[Dict[str, any]]:
        """Returns the next navigation lane dictionary if available."""
        if self.current_lane_idx + 1 < len(self.lanes):
            return self.lanes[self.current_lane_idx + 1]
        return None

    def get_target_turn_direction(self) -> str:
        """
        Determines whether the robot should turn LEFT or RIGHT into the next row.
        """
        if self.current_lane_idx >= len(self.lanes) - 1:
            return "NONE"

        curr_z = self.lanes[self.current_lane_idx]["z_center"]
        next_z = self.lanes[self.current_lane_idx + 1]["z_center"]
        curr_dir = self.lanes[self.current_lane_idx]["direction"]

        # If heading in +X: target z > curr z implies turning LEFT (counter-clockwise)
        # If heading in -X: target z < curr z implies turning LEFT
        delta_z = next_z - curr_z
        if curr_dir > 0:
            return "LEFT" if delta_z > 0 else "RIGHT"
        else:
            return "LEFT" if delta_z < 0 else "RIGHT"

    def update(
        self,
        x: float,
        z: float,
        is_row_end: bool,
        is_obstacle: bool,
        turn_completed: bool,
        is_row_entered: bool = False,
        yaw: float = 0.0
    ) -> MissionState:
        """
        Main FSM Transition Function. Evaluates current pose and sensor flags
        to transition the robot to the appropriate operational state.
        
        Args:
            x: Robot current X position (longitudinal field coordinate).
            z: Robot current Z position (lateral row coordinate).
            is_row_end: Boolean flag from Member 2 sensors.
            is_obstacle: Boolean flag from Member 2 sensors.
            turn_completed: Boolean flag from Member 4 headland maneuver.
            is_row_entered: Boolean flag indicating foliage detected inside row.
            yaw: Current robot heading in radians.
            
        Returns:
            Current MissionState.
        """
        # Distance tracking
        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dz = z - self.last_pose[1]
            step_dist = math.hypot(dx, dz)
            if step_dist < 1.0: # Filter out teleport spikes
                self.distance_traversed += step_dist
        self.last_pose = (x, z)

        # Emergency obstacle handling overrides normal state only during IN_ROW_TRACKING
        # (headland transitions and turns must not be interrupted by in-row obstacle safety)
        if is_obstacle and self.state == MissionState.IN_ROW_TRACKING:
            self.previous_state = self.state
            self.state = MissionState.OBSTACLE_AVOIDANCE
            return self.state

        if self.state == MissionState.OBSTACLE_AVOIDANCE:
            if not is_obstacle:
                # Obstacle cleared, resume previous state
                self.state = self.previous_state if self.previous_state != MissionState.OBSTACLE_AVOIDANCE else MissionState.IN_ROW_TRACKING
            return self.state

        # State Machine Transitions
        curr_lane = self.get_current_lane()

        if self.state == MissionState.INITIALIZE:
            # Self-check passed, start tracking first row
            self.state = MissionState.IN_ROW_TRACKING

        elif self.state == MissionState.IN_ROW_TRACKING:
            # Check row boundary / headland triggers
            # Physical crop rows extend from x = -8.0m to x = +8.0m (length 16.0m).
            # The robot must be approaching and near the headland boundary in the lane's direction.
            at_headland = False
            near_headland = False

            if curr_lane["direction"] > 0:
                at_headland = (x >= self.headland_x_pos)
                near_headland = (x >= (self.headland_x_pos - 0.50))
            elif curr_lane["direction"] < 0:
                at_headland = (x <= self.headland_x_neg)
                near_headland = (x <= (self.headland_x_neg + 0.50))

            if at_headland or (is_row_end and near_headland):
                self.state = MissionState.HEADLAND_TRANSITION

        elif self.state == MissionState.HEADLAND_TRANSITION:
            # Stay in HEADLAND_TRANSITION for headland_exit_ticks_required steps so the
            # robot drives forward into the open headland before the turn starts.
            # This makes the rotation clearly visible in the open field area.
            self.headland_exit_ticks += 1
            if self.current_lane_idx >= len(self.lanes) - 1:
                # All lanes finished
                self.state = MissionState.MISSION_COMPLETE
            elif self.headland_exit_ticks >= self.headland_exit_ticks_required:
                # Exit drive complete: now trigger the rotation
                self.headland_exit_ticks = 0
                self.state = MissionState.HEADLAND_TURN

        elif self.state == MissionState.HEADLAND_TURN:
            if turn_completed:
                # Headland turn finished, enter the new row
                self.completed_lanes.append(curr_lane["id"])
                self.current_lane_idx += 1
                self.row_entry_ticks = 0
                self.state = MissionState.ROW_ENTRY

        elif self.state == MissionState.ROW_ENTRY:
            # Meaningful ROW_ENTRY: verify robot position and alignment with next row
            self.row_entry_ticks += 1
            curr_lane = self.get_current_lane()
            target_y = curr_lane["z_center"]
            lane_dir = curr_lane["direction"]
            target_yaw = 0.0 if lane_dir > 0 else math.pi

            # Longitudinal progress past the headland threshold into the crop row (-8.0 to +8.0)
            past_boundary = False
            if lane_dir > 0 and x >= -7.80:
                past_boundary = True
            elif lane_dir < 0 and x <= 7.80:
                past_boundary = True

            lat_err = abs(z - target_y)
            heading_err = abs(math.atan2(math.sin(yaw - target_yaw), math.cos(yaw - target_yaw)))
            aligned = (lat_err <= 0.35) and (heading_err <= 0.40) and past_boundary

            # Transition to in-row tracking ONLY once robot is aligned and has entered correct region
            if (aligned and self.row_entry_ticks >= 10) or (is_row_entered and past_boundary and lat_err <= 0.35 and heading_err <= 0.40):
                self.state = MissionState.IN_ROW_TRACKING
            elif self.row_entry_ticks >= self.max_row_entry_ticks:
                # Do not transition unconditionally to IN_ROW_TRACKING outside the lane.
                # Flag failure so safety recovery halts execution.
                self.is_lost = True

        elif self.state == MissionState.MISSION_COMPLETE:
            pass

        return self.state

    def get_coverage_percentage(self) -> float:
        """Returns the estimated field coverage percentage (0.0 to 100.0%)."""
        ratio = (self.distance_traversed / max(1.0, self.total_planned_distance)) * 100.0
        return min(100.0, max(0.0, ratio))

    def get_status(self) -> Dict[str, any]:
        """Returns a snapshot of the mission planner status."""
        return {
            "state": self.state.value,
            "current_lane": self.get_current_lane()["id"],
            "lane_index": self.current_lane_idx,
            "total_lanes": len(self.lanes),
            "turn_direction": self.get_target_turn_direction(),
            "coverage_pct": self.get_coverage_percentage(),
            "distance_m": self.distance_traversed,
            "is_complete": self.state == MissionState.MISSION_COMPLETE
        }
