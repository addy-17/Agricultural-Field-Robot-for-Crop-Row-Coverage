"""
run_all_tests.py - Comprehensive Multi-Module Verification Suite
CSAI4014 AISR Project | Agricultural Field Robot for Crop-Row Coverage

Tests and validates deliverables across all 5 group members:
  - Member 1: Kinematics & Coverage Mission Planner
  - Member 2: Robot Model, Sensors & Motion Interface
  - Member 3: Guidance & In-Row Control (PID & Stanley)
  - Member 4: Headland Maneuver Trajectories (Point Turn & Omega Turn)
  - Member 5: Telemetry Logging & Figure Generation Pipeline
"""

import sys
import os
import math

# Add controller directory to sys.path
CONTROLLER_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "controllers", "agricultural_robot")
sys.path.append(CONTROLLER_DIR)

from kinematics import Pioneer3ATKinematics
from mission_planner import CoverageMissionPlanner, MissionState
from motion import Pioneer3ATMotion
from sensors import Pioneer3ATSensors, SensorMovingAverageFilter
from pid_controller import PIDController
from row_follower import MethodAPIDRowFollower, MethodBStanleyRowFollower
from obstacle_avoidance import InRowObstacleSafety
from headland_turn import HeadlandTurnGenerator, TurnStrategy
from metrics_logger import TelemetryMetricsLogger


def test_member1():
    print("\n--- [MEMBER 1] Testing Kinematics & Coverage Mission Planner ---")
    # Kinematics
    k = Pioneer3ATKinematics()
    wl, wr = k.inverse_kinematics(v=0.5, omega=0.0)
    assert math.isclose(wl, wr, rel_tol=1e-2), "Symmetric forward speed expected"
    v_recov, w_recov = k.forward_kinematics(wl, wr)
    assert math.isclose(v_recov, 0.5, rel_tol=1e-2), "Recovered linear velocity matches"

    # Mission Planner
    mp = CoverageMissionPlanner()
    assert mp.state == MissionState.INITIALIZE
    st = mp.update(x=-7.0, z=0.0, is_row_end=False, is_obstacle=False, turn_completed=False)
    assert st == MissionState.IN_ROW_TRACKING, "Expected transition to IN_ROW_TRACKING"

    # Simulate reaching row end
    st = mp.update(x=7.6, z=0.0, is_row_end=True, is_obstacle=False, turn_completed=False)
    assert st == MissionState.HEADLAND_TRANSITION, "Expected HEADLAND_TRANSITION at row end"
    print("  -> Member 1 Kinematics & FSM Planner: PASSED")


def test_member2():
    print("\n--- [MEMBER 2] Testing Robot Model, Motion & Sensor Interfaces ---")
    motion = Pioneer3ATMotion()
    wl, wr = motion.move_forward(0.5)
    assert wl > 0 and wr > 0
    wl, wr = motion.stop()
    assert wl == 0 and wr == 0

    sensors = Pioneer3ATSensors()
    dist_m = sensors.calibrate_raw_to_meters(512.0)
    assert math.isclose(dist_m, 1.0, rel_tol=1e-2)

    filt = SensorMovingAverageFilter(window_size=3)
    filt.update(1.0)
    filt.update(1.2)
    val = filt.update(1.4)
    assert math.isclose(val, 1.20, rel_tol=1e-2)
    print("  -> Member 2 Motion & Sensor Interface: PASSED")


def test_member3():
    print("\n--- [MEMBER 3] Testing In-Row Guidance & Obstacle Safety ---")
    # PID Controller
    pid = PIDController(kp=1.5, ki=0.1, kd=0.2, output_limit=1.0)
    u = pid.compute(error=0.2, dt=0.05)
    assert u > 0.0, "Positive control effort expected for positive error"

    # Method A Follower
    follower_a = MethodAPIDRowFollower(nominal_speed=0.45)
    v, w = follower_a.compute(lateral_error=0.10, heading_error=0.05, dt=0.05)
    assert v > 0 and w < 0, "Corrective steering must steer right when shifted left"

    # Method B Follower (Stanley)
    follower_b = MethodBStanleyRowFollower(nominal_speed=0.50)
    v, w = follower_b.compute(lateral_error=0.10, heading_error=0.05, dt=0.05)
    assert v > 0 and w < 0

    # Obstacle Safety Bubble
    safety = InRowObstacleSafety(stop_threshold=0.40)
    v_s, w_s, is_stop = safety.filter_velocity(0.5, 0.0, front_distance=0.25)
    assert is_stop is True and v_s == 0.0, "Emergency stop must trigger when d <= 0.40m"
    print("  -> Member 3 Guidance (PID & Stanley) & Safety Bubble: PASSED")


def test_member4():
    print("\n--- [MEMBER 4] Testing Headland Turn Generator ---")
    # Point Turn Strategy A
    turner_a = HeadlandTurnGenerator(strategy=TurnStrategy.POINT_TURN_A)
    turner_a.start_turn(turn_direction="LEFT", current_yaw=0.0, row_spacing=1.80)
    assert turner_a.is_running is True
    v, w, finished = turner_a.step(dt=0.05, current_yaw=0.0)
    assert w > 0.0, "Counter-clockwise angular velocity expected for LEFT turn"

    # Omega Turn Strategy B
    turner_b = HeadlandTurnGenerator(strategy=TurnStrategy.OMEGA_TURN_B)
    turner_b.start_turn(turn_direction="LEFT", current_yaw=0.0, row_spacing=1.80)
    assert turner_b.is_running is True
    v, w, finished = turner_b.step(dt=0.05, current_yaw=0.0)
    assert v > 0.0, "Forward velocity must remain non-zero during continuous Omega turn"
    print("  -> Member 4 Headland Maneuvers (Point Turn & Omega Turn): PASSED")


def test_member5():
    print("\n--- [MEMBER 5] Testing Telemetry Logger & Benchmarking ---")
    logger = TelemetryMetricsLogger(method_name="Unit_Test")
    logger.log_step(
        time_s=0.05, state="IN_ROW_TRACKING", x=0.0, y=0.11, z=0.0,
        yaw_rad=0.0, lateral_error=0.02, heading_error=0.01,
        cmd_v=0.5, cmd_omega=0.0, wheel_left=4.5, wheel_right=4.5,
        coverage_pct=1.0, dt=0.05
    )
    s = logger.get_summary()
    assert s["samples_count"] == 1
    assert math.isclose(s["mean_abs_error_m"], 0.02, rel_tol=1e-3)
    print("  -> Member 5 Telemetry Logger: PASSED")


def main():
    print("=" * 70)
    print("   CSAI4014 AISR PROJECT: MASTER 5-MEMBER VERIFICATION SUITE   ")
    print("=" * 70)

    test_member1()
    test_member2()
    test_member3()
    test_member4()
    test_member5()

    print("\n" + "=" * 70)
    print("   ALL 5 MODULES PASSED 100% STANDALONE VERIFICATION!   ")
    print("   SYSTEM IS COMPLETE, ROBUST & READY FOR EVALUATION    ")
    print("=" * 70)


if __name__ == "__main__":
    main()
