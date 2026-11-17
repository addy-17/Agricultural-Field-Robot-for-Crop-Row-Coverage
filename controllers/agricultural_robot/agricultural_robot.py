"""
agricultural_robot.py - Master Webots Controller & Multi-Module Integrator
CSAI4014 AISR Project | Agricultural Field Robot for Crop-Row Coverage

Main executable coordinating all 5 student modules:
  - Member 1: CoverageMissionPlanner & Pioneer3ATKinematics
  - Member 2: Pioneer3ATMotion & Pioneer3ATSensors
  - Member 3: InRowFollower (Method A PID / Method B Stanley) & InRowObstacleSafety
  - Member 4: HeadlandTurnGenerator (Strategy A Point Turn / Strategy B Omega-Turn)
  - Member 5: TelemetryMetricsLogger
"""

import sys
import os
import math

# Add controller directory to system path
CONTROLLER_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(CONTROLLER_DIR)

try:
    from controller import Robot
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False
    Robot = None

# Import all 5 Member Modules
from kinematics import Pioneer3ATKinematics
from mission_planner import CoverageMissionPlanner, MissionState
from motion import Pioneer3ATMotion
from sensors import Pioneer3ATSensors
from row_follower import MethodAPIDRowFollower, MethodBStanleyRowFollower
from obstacle_avoidance import InRowObstacleSafety, ObstacleBypassManeuver
from headland_turn import HeadlandTurnGenerator, TurnStrategy
from metrics_logger import TelemetryMetricsLogger


class MasterAgriculturalRobot:
    """
    Master Autonomous Crop-Row Coverage Robot System.
    """

    def __init__(self, use_method_b: bool = True):
        self.use_method_b = use_method_b
        self.method_tag = "Method_B" if use_method_b else "Method_A"

        print("=" * 70)
        print("  AGRICULTURAL FIELD ROBOT FOR CROP-ROW COVERAGE")
        print("  CSAI4014 Algorithms for Intelligent Systems and Robotics")
        print(f"  Operational Configuration: {self.method_tag.upper()}")
        print("=" * 70)

        # 1. Instantiate Webots Robot instance
        if WEBOTS_AVAILABLE:
            self.robot = Robot()
            self.time_step = int(self.robot.getBasicTimeStep())
            if self.time_step <= 0:
                self.time_step = 32
        else:
            self.robot = None
            self.time_step = 32

        self.dt = self.time_step / 1000.0  # seconds

        # 2. Member 1: Kinematics & Coverage Mission Planner
        self.kinematics = Pioneer3ATKinematics()
        self.mission_planner = CoverageMissionPlanner()

        # 3. Member 2: Motion & Sensor Abstraction
        self.motion = Pioneer3ATMotion(robot=self.robot)
        self.sensors = Pioneer3ATSensors(robot=self.robot, time_step=self.time_step)

        # 4. Member 3: In-Row Guidance, Safety Bubble & Active Obstacle Bypass
        if self.use_method_b:
            self.guidance = MethodBStanleyRowFollower(nominal_speed=0.55, min_speed=0.28)
        else:
            self.guidance = MethodAPIDRowFollower(nominal_speed=0.52)
        self.safety = InRowObstacleSafety(stop_threshold=0.80, caution_threshold=1.50)
        self.bypass = ObstacleBypassManeuver()
        self.obstacle_cooldown = 0

        # 5. Member 4: Headland Maneuver Generator
        turn_strat = TurnStrategy.OMEGA_TURN_B if self.use_method_b else TurnStrategy.POINT_TURN_A
        self.turn_generator = HeadlandTurnGenerator(strategy=turn_strat)
        self._turn_initialized: bool = False   # guard: start_turn() fires only once per headland

        # 6. Member 5: Real-Time Telemetry Logger
        self.logger = TelemetryMetricsLogger(method_name=self.method_tag)

        self.sim_time = 0.0
        self.step_count = 0

    def run(self):
        """Main synchronous simulation loop."""
        print(f"[Master] System initialized. Entering control loop (dt={self.dt:.3f}s)...")

        # In standalone mode, run 1500 steps to simulate the complete field traversal
        max_standalone_steps = 1500 if not WEBOTS_AVAILABLE else int(1e9)

        while True:
            # Step Webots simulator
            if WEBOTS_AVAILABLE:
                if self.robot.step(self.time_step) == -1:
                    break
            else:
                if self.step_count >= max_standalone_steps:
                    break

            self.step_count += 1
            self.sim_time += self.dt

            # -----------------------------------------------------------------
            # STEP 1: Sensor Perception & Filtering (Member 2)
            # -----------------------------------------------------------------
            self.sensors.update()
            x, y, yaw = self.sensors.get_pose()
            z = self.sensors.latest_pose.get("z", 0.0)

            e_y, e_psi = self.sensors.get_row_errors()
            is_row_end = self.sensors.is_row_end()
            is_row_entered = self.sensors.is_row_entered()
            front_dist = self.sensors.latest_distances.get("min_front", 2.0)

            # -----------------------------------------------------------------
            # STEP 2: Mission Planning & State Supervision (Member 1)
            # -----------------------------------------------------------------
            turn_successful = getattr(self.turn_generator, "turn_successful", True)
            turn_completed = (not self.turn_generator.is_running) and turn_successful
            turn_failed = (not self.turn_generator.is_running) and (not turn_successful) and self._turn_initialized
            if turn_failed:
                self.mission_planner.is_lost = True
            # Obstacle detection with active cooldown to avoid re-triggering while passing.
            # FIX 1: Only monitor/trigger obstacles during IN_ROW_TRACKING;
            # do not interrupt headland transitions or turns.
            if self.obstacle_cooldown > 0:
                self.obstacle_cooldown -= 1
                is_obstacle = False
            elif self.mission_planner.state != MissionState.IN_ROW_TRACKING:
                is_obstacle = False
            else:
                is_obstacle = (front_dist <= 1.05) and (self.step_count > 60)

            # If bypass is already actively steering around an obstacle, keep FSM in avoidance
            if self.bypass.is_active:
                is_obstacle = True

            # ENU world: GPS returns (x=East, y=North/lateral, z=Up/height).
            # Mission planner's 'z' parameter is the LATERAL lane coordinate (y in ENU),
            # NOT the height. Passing 'z' (height ≈ 0.11m constant) was causing the FSM
            # to never detect lane transitions correctly.
            state = self.mission_planner.update(
                x=x,
                z=y,          # <-- ENU FIX: lateral (North/Y) position, not height
                is_row_end=is_row_end,
                is_obstacle=is_obstacle,
                turn_completed=turn_completed,
                is_row_entered=is_row_entered,
                yaw=yaw
            )

            # FIX 4: Check if mission planner flagged safe recovery failure
            if getattr(self.mission_planner, "is_lost", False):
                v_cmd = 0.0
                omega_cmd = 0.0
                self.motion.stop()
                print(f"[SAFETY RECOVERY] Unable to acquire row boundaries at ({x:.2f}, {y:.2f}). Stopping.")
                break

            # FIX 1: Universal field boundary safety check (enforced across ALL movement states)
            if abs(y) > 5.5 or abs(x) > 12.0:
                v_cmd = 0.0
                omega_cmd = 0.0
                self.motion.stop()
                print(f"[SAFETY STOP] Robot at ({x:.2f}, {y:.2f}) outside safe field boundary! Stopping.")
                break

            # -----------------------------------------------------------------
            # STEP 3: Execution Control by State
            # -----------------------------------------------------------------
            v_cmd = 0.55
            omega_cmd = 0.0

            if state == MissionState.INITIALIZE or self.step_count < 30:
                # Startup phase: drive straight forward down the row at nominal speed
                v_cmd = 0.55
                omega_cmd = 0.0

            elif state == MissionState.ROW_ENTRY:
                # FIX 3: Active Waypoint Navigation toward target lane entry
                curr_lane = self.mission_planner.get_current_lane()
                target_y = curr_lane["z_center"]
                lane_dir = curr_lane["direction"]
                target_yaw = 0.0 if lane_dir > 0 else math.pi

                e_y_entry = lane_dir * (y - target_y)
                e_psi_entry = math.atan2(math.sin(yaw - target_yaw), math.cos(yaw - target_yaw))

                v_entry = 0.35
                k_lat = 1.20
                k_yaw = 0.85
                omega_entry = -k_yaw * e_psi_entry - math.atan2(k_lat * e_y_entry, v_entry)
                omega_cmd = max(-0.50, min(0.50, omega_entry))
                v_cmd, omega_cmd, _ = self.safety.filter_velocity(v_entry, omega_cmd, front_dist)

            elif state == MissionState.IN_ROW_TRACKING:
                curr_lane = self.mission_planner.get_current_lane()
                target_y = curr_lane["z_center"]
                lane_dir = curr_lane["direction"]
                target_yaw = 0.0 if lane_dir > 0 else math.pi

                d_l = self.sensors.latest_distances.get("avg_left", 2.0)
                d_r = self.sensors.latest_distances.get("avg_right", 2.0)

                # FIX 3: Prevent row follower from treating max-range sensor readings as zero error
                # If BOTH lateral sensors read open space (>= 1.80m), crop canopy boundaries are lost!
                if d_l >= 1.80 and d_r >= 1.80:
                    e_y_global = lane_dir * (y - target_y)
                    e_psi_global = math.atan2(math.sin(yaw - target_yaw), math.cos(yaw - target_yaw))
                    v_des, omega_des = self.guidance.compute(lateral_error=e_y_global, heading_error=e_psi_global, dt=self.dt)
                    v_cmd, omega_cmd, _ = self.safety.filter_velocity(v_des, omega_des, front_dist)
                else:
                    v_des, omega_des = self.guidance.compute(lateral_error=e_y, heading_error=e_psi, dt=self.dt)
                    v_cmd, omega_cmd, _ = self.safety.filter_velocity(v_des, omega_des, front_dist)

            elif state == MissionState.HEADLAND_TRANSITION:
                # Robot is driving OUT of the row into open headland.
                # The FSM holds this state for ~75 steps (2.4 s) before allowing the turn.
                # Drive at moderate forward speed — clearly shows exit from rows.
                v_cmd = 0.35
                omega_cmd = 0.0
                self._turn_initialized = False   # reset turn guard for next headland

            elif state == MissionState.HEADLAND_TURN:
                # First entry: initialise the rotation manoeuvre with next-row target yaw and dynamic spacing
                if not self._turn_initialized:
                    turn_dir = self.mission_planner.get_target_turn_direction()
                    current_lane = self.mission_planner.get_current_lane()
                    next_lane = self.mission_planner.get_next_lane()
                    target_yaw = None
                    if next_lane:
                        target_yaw = 0.0 if next_lane["direction"] > 0 else math.pi
                        turn_spacing = abs(next_lane["z_center"] - current_lane["z_center"])
                    else:
                        turn_spacing = self.mission_planner.row_spacing_z

                    self.turn_generator.start_turn(
                        turn_direction=turn_dir,
                        current_yaw=yaw,
                        row_spacing=turn_spacing,
                        target_yaw=target_yaw
                    )
                    self._turn_initialized = True
                    print(f"[HEADLAND_TURN] Starting {turn_dir} turn at "
                          f"pos=({x:.2f}, {y:.2f}), yaw={math.degrees(yaw):.1f}°, spacing={turn_spacing:.2f}m")

                # Execute turn step
                v_cmd, omega_cmd, finished = self.turn_generator.step(dt=self.dt, current_yaw=yaw)
                if finished and not getattr(self.turn_generator, "turn_successful", True):
                    v_cmd = 0.0
                    omega_cmd = 0.0
                    self.motion.stop()
                    self.mission_planner.is_lost = True
                    print(f"[SAFETY RECOVERY] Headland turn alignment failed at ({x:.2f}, {y:.2f}). Stopping.")
                    break

            elif state == MissionState.OBSTACLE_AVOIDANCE:
                # Active Obstacle Bypass Maneuver (Member 3)
                if not self.bypass.is_active:
                    d_left = self.sensors.latest_distances.get("avg_left", 0.90)
                    d_right = self.sensors.latest_distances.get("avg_right", 0.90)
                    # Detour toward the side with more clearance (or default Left)
                    swerve_left = (d_left >= d_right) or (d_left >= 0.70)
                    self.bypass.start(swerve_left=swerve_left)

                v_cmd, omega_cmd, bypass_done = self.bypass.step(dt=self.dt)
                if bypass_done:
                    # Detour finished! Resume saved previous state with a cooldown buffer
                    self.obstacle_cooldown = 120  # ~4 seconds buffer to pass obstacle completely
                    self.bypass.reset()
                    prev_state = self.mission_planner.previous_state
                    if prev_state and prev_state != MissionState.OBSTACLE_AVOIDANCE:
                        self.mission_planner.state = prev_state
                    else:
                        self.mission_planner.state = MissionState.IN_ROW_TRACKING

            elif state == MissionState.MISSION_COMPLETE:
                v_cmd = 0.0
                omega_cmd = 0.0
                self.motion.stop()
                print(f"[MISSION COMPLETE] All crop rows covered in {self.sim_time:.2f}s!")
                break

            # -----------------------------------------------------------------
            # STEP 4: Motor Actuation (Member 2)
            # -----------------------------------------------------------------
            w_left, w_right = self.motion.set_velocity(linear_velocity=v_cmd, angular_velocity=omega_cmd)

            # Member 1 Kinematics update for dead-reckoning
            self.kinematics.update_odometry(v=v_cmd, omega=omega_cmd, dt=self.dt)

            # -----------------------------------------------------------------
            # STEP 5: Telemetry Logging & Metrics (Member 5)
            # -----------------------------------------------------------------
            coverage_pct = self.mission_planner.get_coverage_percentage()
            self.logger.log_step(
                time_s=self.sim_time,
                state=state.value,
                x=x,
                y=y,
                z=z,
                yaw_rad=yaw,
                lateral_error=e_y,
                heading_error=e_psi,
                cmd_v=v_cmd,
                cmd_omega=omega_cmd,
                wheel_left=w_left,
                wheel_right=w_right,
                coverage_pct=coverage_pct,
                dt=self.dt
            )

            # Periodic status report in Webots console
            if self.step_count % 30 == 0:
                print(f"[{state.value[:10]}] t={self.sim_time:05.1f}s | "
                      f"Pos=({x:.2f}, {y:.2f}) | e_y={e_y:+.3f}m | "
                      f"v={v_cmd:.2f}m/s | w={omega_cmd:+.3f}rad/s | Cov={coverage_pct:.1f}%")

        # Mission Finished Summary
        self.motion.stop()
        self.logger.print_summary()


def main():
    # Use Method B by default (or check argv for --method A)
    use_method_b = True
    if len(sys.argv) > 1 and "A" in sys.argv[1].upper():
        use_method_b = False

    app = MasterAgriculturalRobot(use_method_b=use_method_b)
    app.run()


if __name__ == "__main__":
    main()
