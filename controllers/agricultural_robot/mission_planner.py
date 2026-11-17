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
        self.row_spacing_z = 1.80
        self.headland_x_pos = 7.50   # Positive headland boundary (meters)
        self.headland_x_neg = -7.50  # Negative headland boundary (meters)

        # Pre-planned coverage sequence of navigation lanes:
        # Lane 2 (z=0.0, heading +X) -> Lane 3 (z=+1.8, heading -X) -> Lane 1 (z=-1.8, heading +X)
        self.lanes: List[Dict[str, any]] = [
            {"id": "Lane_2", "z_center": 0.00, "direction": +1, "start_x": -7.0, "end_x": +7.5},
            {"id": "Lane_3", "z_center": 1.80, "direction": -1, "start_x": +7.5, "end_x": -7.5},
            {"id": "Lane_1", "z_center": -1.80, "direction": +1, "start_x": -7.5, "end_x": +7.5},
        ]

        self.current_lane_idx: int = 0
        self.state: MissionState = MissionState.INITIALIZE
        self.previous_state: MissionState = MissionState.INITIALIZE

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
        turn_completed: bool
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

        # Emergency obstacle handling overrides normal state
        if is_obstacle and self.state not in (MissionState.OBSTACLE_AVOIDANCE, MissionState.MISSION_COMPLETE):
            self.previous_state = self.state
            self.state = MissionState.OBSTACLE_AVOIDANCE
            return self.state

        if self.state == MissionState.OBSTACLE_AVOIDANCE:
            if not is_obstacle:
                # Obstacle cleared, resume previous state
                self.state = self.previous_state
            return self.state

        # State Machine Transitions
        curr_lane = self.get_current_lane()

        if self.state == MissionState.INITIALIZE:
            # Self-check passed, start tracking first row
            self.state = MissionState.IN_ROW_TRACKING

        elif self.state == MissionState.IN_ROW_TRACKING:
            # Check row boundary / headland triggers
            at_headland = False
            if curr_lane["direction"] > 0 and x >= self.headland_x_pos:
                at_headland = True
            elif curr_lane["direction"] < 0 and x <= self.headland_x_neg:
                at_headland = True

            if is_row_end or at_headland:
                self.state = MissionState.HEADLAND_TRANSITION

        elif self.state == MissionState.HEADLAND_TRANSITION:
            # Transition to headland turning
            if self.current_lane_idx >= len(self.lanes) - 1:
                # All lanes finished!
                self.state = MissionState.MISSION_COMPLETE
            else:
                self.state = MissionState.HEADLAND_TURN

        elif self.state == MissionState.HEADLAND_TURN:
            if turn_completed:
                # Headland turn finished, enter the new row
                self.completed_lanes.append(curr_lane["id"])
                self.current_lane_idx += 1
                self.state = MissionState.ROW_ENTRY

        elif self.state == MissionState.ROW_ENTRY:
            # Once inside new row, transition back to in-row guidance
            self.state = MissionState.IN_ROW_TRACKING

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
