"""
test_member3_modules.py - Unit tests for Member 3 modules
CSAI4014 AISR Project | Member 3

Covers: pid_controller.py, row_follower.py, obstacle_avoidance.py
Run (from repo root):   python controllers/agricultural_robot/test_member3_modules.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pid_controller import PIDController
from row_follower import MethodAPIDRowFollower, MethodBStanleyRowFollower
from obstacle_avoidance import InRowObstacleSafety, ObstacleBypassManeuver

DT = 0.05


# ---------------------------------------------------------------- PID
def test_pid_proportional_response():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0, output_limit=10.0)
    assert abs(pid.compute(0.5, DT) - 1.0) < 1e-9


def test_pid_output_saturation():
    pid = PIDController(kp=100.0, ki=0.0, kd=0.0, output_limit=1.5)
    assert pid.compute(5.0, DT) == 1.5
    assert pid.compute(-5.0, DT) == -1.5


def test_pid_integral_anti_windup():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, output_limit=5.0, integral_limit=0.5)
    for _ in range(1000):
        pid.compute(1.0, DT)
    assert abs(pid.integral) <= 0.5 + 1e-9


def test_pid_zero_dt_safe():
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    assert pid.compute(1.0, 0.0) == 0.0


def test_pid_reset_clears_state():
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    pid.compute(1.0, DT)
    pid.compute(2.0, DT)
    pid.reset()
    assert pid.integral == 0.0 and pid.prev_error == 0.0 and pid.first_step


def test_pid_derivative_zero_on_first_step():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0, output_limit=10.0)
    assert pid.compute(1.0, DT) == 0.0


# ------------------------------------------------- row followers: signs
def _followers():
    return [MethodAPIDRowFollower(), MethodBStanleyRowFollower()]


def test_lateral_error_left_steers_right():
    for f in _followers():
        _, omega = f.compute(lateral_error=0.2, heading_error=0.0, dt=DT)
        assert omega < 0.0, type(f).__name__


def test_lateral_error_right_steers_left():
    for f in _followers():
        _, omega = f.compute(lateral_error=-0.2, heading_error=0.0, dt=DT)
        assert omega > 0.0, type(f).__name__


def test_heading_error_left_steers_right():
    for f in _followers():
        _, omega = f.compute(lateral_error=0.0, heading_error=0.3, dt=DT)
        assert omega < 0.0, type(f).__name__


def test_heading_error_right_steers_left():
    for f in _followers():
        _, omega = f.compute(lateral_error=0.0, heading_error=-0.3, dt=DT)
        assert omega > 0.0, type(f).__name__


def test_zero_error_gives_zero_turn():
    for f in _followers():
        _, omega = f.compute(0.0, 0.0, DT)
        assert abs(omega) < 1e-9, type(f).__name__


def test_omega_is_clamped():
    for f in _followers():
        _, omega = f.compute(lateral_error=5.0, heading_error=2.0, dt=DT)
        assert abs(omega) <= f.max_omega + 1e-9, type(f).__name__


def test_method_a_constant_speed():
    f = MethodAPIDRowFollower(nominal_speed=0.55)
    v1, _ = f.compute(0.0, 0.0, DT)
    v2, _ = f.compute(0.3, 0.2, DT)
    assert v1 == v2 == 0.55


def test_method_b_adaptive_speed():
    f = MethodBStanleyRowFollower(nominal_speed=0.55, min_speed=0.28)
    v_small, _ = f.compute(0.0, 0.0, DT)
    v_big, _ = f.compute(0.5, 0.4, DT)
    assert abs(v_small - 0.55) < 1e-9
    assert 0.28 <= v_big < v_small


# ------------------------------------------- closed-loop convergence
def _simulate(follower, y0, psi0, steps=400):
    y, psi = y0, psi0
    for _ in range(steps):
        v, omega = follower.compute(y, psi, DT)
        y += v * math.sin(psi) * DT
        psi += omega * DT
    return y, psi


def test_closed_loop_converges_to_centerline():
    for f in _followers():
        y, psi = _simulate(f, y0=0.30, psi0=0.0)
        assert abs(y) < 0.05, f"{type(f).__name__} y={y:.3f}"
        assert abs(psi) < 0.10, f"{type(f).__name__} psi={psi:.3f}"


# ------------------------------------------------ obstacle safety bubble
def test_safe_zone_passthrough():
    s = InRowObstacleSafety()
    assert s.filter_velocity(0.5, 0.1, 3.0) == (0.5, 0.1, False)


def test_caution_zone_slows_down():
    s = InRowObstacleSafety()
    v, w, trig = s.filter_velocity(0.5, 0.1, 1.2)
    assert 0.0 < v < 0.5 and w == 0.1 and trig is False


def test_stop_zone_triggers_bypass():
    s = InRowObstacleSafety()
    v, w, trig = s.filter_velocity(0.5, 0.1, 0.5)
    assert (v, w, trig) == (0.0, 0.0, True)


def test_stop_threshold_boundary_inclusive():
    s = InRowObstacleSafety()
    _, _, trig = s.filter_velocity(0.5, 0.0, s.stop_dist)
    assert trig is True


def test_caution_scale_never_below_floor():
    s = InRowObstacleSafety()
    v, _, _ = s.filter_velocity(1.0, 0.0, s.stop_dist + 1e-6)
    assert v >= 0.20 * 1.0 - 1e-9


# ------------------------------------------------------ bypass maneuver
def test_bypass_idle_when_not_started():
    b = ObstacleBypassManeuver()
    assert b.step(DT) == (0.0, 0.0, True)


def test_bypass_runs_all_phases_and_completes():
    import io
    import contextlib
    b = ObstacleBypassManeuver()
    with contextlib.redirect_stdout(io.StringIO()):
        b.start(swerve_left=True)
        seen = set()
        done = False
        for _ in range(int(20 / DT)):
            _, _, done = b.step(DT)
            seen.add(b.phase)
            if done:
                break
    assert done and not b.is_active
    assert {1, 2, 3, 4, 5}.issubset(seen)


def test_bypass_direction_sign():
    import io
    import contextlib
    with contextlib.redirect_stdout(io.StringIO()):
        left, right = ObstacleBypassManeuver(), ObstacleBypassManeuver()
        left.start(swerve_left=True)
        right.start(swerve_left=False)
        _, w_left, _ = left.step(DT)
        _, w_right, _ = right.step(DT)
    assert w_left > 0.0 and w_right < 0.0


def test_bypass_reset():
    b = ObstacleBypassManeuver()
    b.is_active, b.phase, b.elapsed = True, 3, 1.0
    b.reset()
    assert not b.is_active and b.phase == 0 and b.elapsed == 0.0


# ------------------------------------------------------------- runner
def run_all():
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except AssertionError as e:
            failed += 1
            print(f"  FAIL  {name}  {e}")
    print(f"\nMember 3 tests: {len(tests) - failed}/{len(tests)} passed")
    return failed == 0


if __name__ == "__main__":
    sys.exit(0 if run_all() else 1)
