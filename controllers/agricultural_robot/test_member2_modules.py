"""
test_member2_modules.py - Automated Unit Test Suite for Member 2 Deliverables
CSAI4014 AISR Project | Member 2: Robot Model, Sensors & Motion Interface

Validates:
  1. Pioneer 3-AT physical parameters and kinematics (motion.py)
  2. High-level velocity and turning interface (motion.py)
  3. Sensor calibration from raw ADC to meters (sensors.py)
  4. Moving average noise filtering (sensors.py)
  5. Lateral cross-track error (e_y) and heading error (e_psi) (sensors.py)
  6. Row-end (headland transition) and obstacle detection (sensors.py)
"""

import math
import sys
import os

# Ensure local directory is on python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from motion import Pioneer3ATMotion
from sensors import Pioneer3ATSensors, SensorMovingAverageFilter


def run_tests():
    print("=" * 70)
    print("   CSAI4014 AISR PROJECT: MEMBER 2 DELIVERABLES VERIFICATION SUITE   ")
    print("=" * 70)
    
    passed = 0
    total = 0

    # -------------------------------------------------------------------------
    # TEST 1: Physical Parameters and Kinematics Mapping
    # -------------------------------------------------------------------------
    total += 1
    print("\n[TEST 1] Pioneer 3-AT Kinematic Mapping & Saturation...")
    motion = Pioneer3ATMotion()
    assert math.isclose(motion.WHEEL_RADIUS, 0.111, rel_tol=1e-3)
    assert math.isclose(motion.TRACK_WIDTH, 0.400, rel_tol=1e-3)
    assert math.isclose(motion.MAX_MOTOR_SPEED, 6.40, rel_tol=1e-3)

    # Test pure forward speed mapping: v = 0.5 m/s -> omega = 0.5 / 0.111 = 4.5045 rad/s
    w_l, w_r = motion.calculate_wheel_speeds(linear_vel=0.5, angular_vel=0.0)
    expected_w = 0.5 / 0.111
    assert math.isclose(w_l, expected_w, rel_tol=1e-2)
    assert math.isclose(w_r, expected_w, rel_tol=1e-2)
    print(f"  -> Forward speed (0.5 m/s): Left={w_l:.3f} rad/s, Right={w_r:.3f} rad/s [OK]")

    # Test pure turn left: v = 0.0, omega = 1.0 rad/s -> v_L = -0.2, v_R = +0.2 -> w_L = -1.802, w_R = +1.802
    w_l, w_r = motion.calculate_wheel_speeds(linear_vel=0.0, angular_vel=1.0)
    assert math.isclose(w_l, -w_r, rel_tol=1e-2)
    assert w_l < 0.0 and w_r > 0.0
    print(f"  -> Turn left (1.0 rad/s): Left={w_l:.3f} rad/s, Right={w_r:.3f} rad/s [OK]")

    # Test speed saturation limit clamping
    w_l, w_r = motion.calculate_wheel_speeds(linear_vel=5.0, angular_vel=0.0) # Requests ~45 rad/s
    assert math.isclose(w_l, 6.40, rel_tol=1e-3)
    assert math.isclose(w_r, 6.40, rel_tol=1e-3)
    print(f"  -> Velocity saturation: clamped to {w_l:.2f} rad/s max limit [OK]")
    passed += 1

    # -------------------------------------------------------------------------
    # TEST 2: High-Level Motion Interface Functions
    # -------------------------------------------------------------------------
    total += 1
    print("\n[TEST 2] High-Level Motion Helper Functions...")
    motion.move_forward(0.6)
    assert motion.cmd_linear_vel == 0.6 and motion.cmd_angular_vel == 0.0
    print("  -> motion.move_forward(0.6): verified [OK]")

    motion.turn_left(0.75)
    assert motion.cmd_linear_vel == 0.0 and motion.cmd_angular_vel == 0.75
    print("  -> motion.turn_left(0.75): verified [OK]")

    motion.turn_right(0.75)
    assert motion.cmd_linear_vel == 0.0 and motion.cmd_angular_vel == -0.75
    print("  -> motion.turn_right(0.75): verified [OK]")

    motion.stop()
    assert motion.cmd_linear_vel == 0.0 and motion.cmd_angular_vel == 0.0
    assert motion.left_motor_speed == 0.0 and motion.right_motor_speed == 0.0
    print("  -> motion.stop(): zero setpoints verified [OK]")
    passed += 1

    # -------------------------------------------------------------------------
    # TEST 3: Distance Sensor Calibration (ADC to Meters)
    # -------------------------------------------------------------------------
    total += 1
    print("\n[TEST 3] Distance Sensor Calibration Curve...")
    sensors = Pioneer3ATSensors()
    
    # 0 ADC -> 0.05m (min clamp), 512 ADC -> 1.0m, 1024 ADC -> 2.0m (max)
    d_0 = sensors.calibrate_raw_to_meters(0.0)
    d_half = sensors.calibrate_raw_to_meters(512.0)
    d_full = sensors.calibrate_raw_to_meters(1024.0)
    assert math.isclose(d_0, 0.05, rel_tol=1e-2)
    assert math.isclose(d_half, 1.00, rel_tol=1e-2)
    assert math.isclose(d_full, 2.00, rel_tol=1e-2)
    print(f"  -> RAW=0 -> {d_0:.2f}m, RAW=512 -> {d_half:.2f}m, RAW=1024 -> {d_full:.2f}m [OK]")
    passed += 1

    # -------------------------------------------------------------------------
    # TEST 4: Moving Average Filter & Outlier Suppression
    # -------------------------------------------------------------------------
    total += 1
    print("\n[TEST 4] Noise Rejection via Moving Average Filter...")
    ma_filter = SensorMovingAverageFilter(window_size=5)
    test_signals = [0.90, 0.92, 0.88, 1.50, 0.91] # 1.50 is an artificial leaf spike
    smoothed = [ma_filter.update(s) for s in test_signals]
    print(f"  -> Raw signals: {test_signals}")
    print(f"  -> Filtered output: {[round(x, 3) for x in smoothed]}")
    # Final average = (0.90 + 0.92 + 0.88 + 1.50 + 0.91) / 5 = 1.022 m (spike attenuated)
    assert math.isclose(smoothed[-1], 1.022, rel_tol=1e-2)
    passed += 1

    # -------------------------------------------------------------------------
    # TEST 5: Lateral & Heading Error Extraction
    # -------------------------------------------------------------------------
    total += 1
    print("\n[TEST 5] In-Row Centering & Heading Error Extraction...")
    # Scenario A: Exactly centered
    sensors.latest_distances["avg_left"] = 0.90
    sensors.latest_distances["avg_right"] = 0.90
    sensors.latest_distances["left_front"] = 0.90
    sensors.latest_distances["left_rear"] = 0.90
    sensors.latest_distances["right_front"] = 0.90
    sensors.latest_distances["right_rear"] = 0.90
    ey, epsi = sensors.get_row_errors()
    assert math.isclose(ey, 0.0, abs_tol=1e-4)
    assert math.isclose(epsi, 0.0, abs_tol=1e-4)
    print(f"  -> Centered: e_y = {ey:.4f} m, e_psi = {epsi:.4f} rad [OK]")

    # Scenario B: Shifted left by +15 cm
    sensors.latest_distances["avg_left"] = 1.05
    sensors.latest_distances["avg_right"] = 0.75
    ey, _ = sensors.get_row_errors()
    assert math.isclose(ey, 0.15, rel_tol=1e-2)
    print(f"  -> Lateral Shift (+15 cm): e_y = {ey:.4f} m [OK]")

    # Scenario C: Heading tilted counter-clockwise
    sensors.latest_distances["left_front"] = 0.95
    sensors.latest_distances["left_rear"] = 0.85
    sensors.latest_distances["right_rear"] = 0.95
    sensors.latest_distances["right_front"] = 0.85
    _, epsi = sensors.get_row_errors()
    # delta = 0.10, base = 0.40 -> atan2(0.10, 0.40) = 0.24497 rad (approx 14 deg)
    assert math.isclose(epsi, 0.245, rel_tol=1e-2)
    print(f"  -> Heading Misalignment: e_psi = {epsi:.4f} rad ({math.degrees(epsi):.1f} deg) [OK]")
    passed += 1

    # -------------------------------------------------------------------------
    # TEST 6: Row-End and Front Obstacle Flag Triggers
    # -------------------------------------------------------------------------
    total += 1
    print("\n[TEST 6] Row-End & Obstacle Safety Trigger Verification...")
    # Inside row
    sensors.latest_distances["avg_left"] = 0.90
    sensors.latest_distances["avg_right"] = 0.90
    assert sensors.is_row_end() is False
    assert sensors.is_row_entered() is True
    print("  -> Inside row: is_row_end() = False, is_row_entered() = True [OK]")

    # Headland Exit (both distances open up > 1.3m)
    sensors.latest_distances["avg_left"] = 1.70
    sensors.latest_distances["avg_right"] = 1.70
    assert sensors.is_row_end() is True
    assert sensors.is_row_entered() is False
    print("  -> Entered headland: is_row_end() = True, is_row_entered() = False [OK]")

    # Front Obstacle (< 0.4m safety threshold)
    sensors.latest_distances["min_front"] = 0.25
    assert sensors.is_obstacle_detected() is True
    print("  -> Obstacle at 0.25m: is_obstacle_detected() = True [OK]")

    sensors.latest_distances["min_front"] = 1.50
    assert sensors.is_obstacle_detected() is False
    print("  -> Obstacle at 1.50m: is_obstacle_detected() = False [OK]")
    passed += 1

    # -------------------------------------------------------------------------
    # FINAL SUMMARY
    # -------------------------------------------------------------------------
    print("\n" + "=" * 70)
    print(f"VERIFICATION SUMMARY: {passed}/{total} TESTS PASSED (100% SUCCESS)")
    print("MEMBER 2 DELIVERABLES READY FOR REPO COMMIT & INTEGRATION")
    print("=" * 70)


if __name__ == "__main__":
    run_tests()

