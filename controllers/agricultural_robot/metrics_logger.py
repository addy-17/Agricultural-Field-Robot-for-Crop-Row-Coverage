"""
metrics_logger.py - Real-Time Telemetry & Experimental Benchmarking Logger
CSAI4014 AISR Project | Member 5 Deliverable

Role:
    Records high-frequency simulation state telemetry into CSV datasets.
    Computes quantitative evaluation metrics comparing Method A vs. Method B:
      - Trajectory coordinates (x, y, z, yaw)
      - Lateral cross-track error (e_y) and heading error (e_psi)
      - Commanded velocities (v, omega) and wheel motor speeds
      - Mechanical steering work proxy / wheel slip index: integral(|omega| dt)
      - Final summary benchmarking table
"""

import csv
import math
import os
from typing import Dict, List, Optional


class TelemetryMetricsLogger:
    """
    High-frequency telemetry data logger and performance evaluator.
    """

    def __init__(self, method_name: str = "Method_B", output_dir: Optional[str] = None):
        self.method_name = method_name
        self.records: List[Dict[str, float]] = []

        # Benchmark cumulative accumulators
        self.total_time: float = 0.0
        self.total_distance: float = 0.0
        self.squared_lateral_errors: List[float] = []
        self.abs_lateral_errors: List[float] = []
        self.steering_work_integral: float = 0.0
        self.prev_pose: Optional[tuple] = None

        # Setup output filepath
        if output_dir is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            output_dir = os.path.join(base_dir, "data")
        os.makedirs(output_dir, exist_ok=True)

        filename = f"telemetry_{self.method_name.lower()}.csv"
        self.filepath = os.path.join(output_dir, filename)

        # Initialize CSV with header
        self.fieldnames = [
            "time_s", "state", "x", "y", "z", "yaw_rad", "yaw_deg",
            "lateral_error_m", "heading_error_rad", "cmd_v_mps", "cmd_omega_radps",
            "wheel_left_radps", "wheel_right_radps", "distance_m", "coverage_pct"
        ]
        self._init_csv()

    def _init_csv(self) -> None:
        try:
            with open(self.filepath, mode="w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(self.fieldnames)
        except Exception as e:
            print(f"[metrics_logger.py] Warning: Could not init CSV ({e})")

    def log_step(
        self,
        time_s: float,
        state: str,
        x: float,
        y: float,
        z: float,
        yaw_rad: float,
        lateral_error: float,
        heading_error: float,
        cmd_v: float,
        cmd_omega: float,
        wheel_left: float,
        wheel_right: float,
        coverage_pct: float,
        dt: float
    ) -> None:
        """
        Records a single control loop iteration into telemetry buffer and CSV.
        """
        self.total_time = time_s

        # Distance calculation
        if self.prev_pose is not None:
            step_dist = math.hypot(x - self.prev_pose[0], z - self.prev_pose[1])
            if step_dist < 1.0:
                self.total_distance += step_dist
        self.prev_pose = (x, z)

        # In-row error accumulation (only compute tracking error during row following)
        if "ROW" in state.upper():
            self.abs_lateral_errors.append(abs(lateral_error))
            self.squared_lateral_errors.append(lateral_error ** 2)

        # Mechanical roughness / steering effort integral: sum(|omega| * dt)
        self.steering_work_integral += abs(cmd_omega) * dt

        row = [
            f"{time_s:.3f}",
            state,
            f"{x:.4f}",
            f"{y:.4f}",
            f"{z:.4f}",
            f"{yaw_rad:.4f}",
            f"{math.degrees(yaw_rad):.2f}",
            f"{lateral_error:.4f}",
            f"{heading_error:.4f}",
            f"{cmd_v:.3f}",
            f"{cmd_omega:.3f}",
            f"{wheel_left:.3f}",
            f"{wheel_right:.3f}",
            f"{self.total_distance:.3f}",
            f"{coverage_pct:.2f}"
        ]

        try:
            with open(self.filepath, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(row)
        except Exception:
            pass

    def get_summary(self) -> Dict[str, float]:
        """
        Computes final quantitative statistical performance metrics.
        """
        n = len(self.abs_lateral_errors)
        mae = sum(self.abs_lateral_errors) / max(1, n)
        rmse = math.sqrt(sum(self.squared_lateral_errors) / max(1, n))
        max_error = max(self.abs_lateral_errors) if n > 0 else 0.0

        return {
            "method": self.method_name,
            "total_time_s": self.total_time,
            "total_distance_m": self.total_distance,
            "mean_abs_error_m": mae,
            "rmse_error_m": rmse,
            "max_lateral_error_m": max_error,
            "steering_roughness_index": self.steering_work_integral,
            "samples_count": n
        }

    def print_summary(self) -> None:
        """Prints a formatted evaluation card to console."""
        s = self.get_summary()
        print("\n" + "=" * 65)
        print(f"   EXPERIMENTAL BENCHMARK SUMMARY: {s['method'].upper()}   ")
        print("=" * 65)
        print(f"  Total Mission Time:           {s['total_time_s']:.2f} s")
        print(f"  Total Distance Traversed:     {s['total_distance_m']:.2f} m")
        print(f"  Mean Absolute Lateral Error:  {s['mean_abs_error_m'] * 100:.2f} cm")
        print(f"  RMS Tracking Error (RMSE):    {s['rmse_error_m'] * 100:.2f} cm")
        print(f"  Maximum Lateral Deviation:    {s['max_lateral_error_m'] * 100:.2f} cm")
        print(f"  Steering Roughness Metric:    {s['steering_roughness_index']:.2f} rad")
        print(f"  Telemetry File:               {self.filepath}")
        print("=" * 65 + "\n")
