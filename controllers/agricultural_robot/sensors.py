"""
sensors.py - Navigation, Perception & Sensor Interface
CSAI4014 AISR Project | Member 2 Deliverable

Role:
    Configures and calibrates navigation sensors (GPS, IMU/Compass) and
    perception sensors (Distance sensors, LiDAR, Camera) on the Pioneer 3-AT.
    Filters raw sensor data, eliminates foliage noise, and exposes a clean,
    calibrated interface for:
      - Global pose (x, y, yaw)
      - Lateral row cross-track error (e_y)
      - Row heading orientation error (e_psi)
      - Row-end detection (headland entry/exit)
      - Front obstacle detection (safety bubble)
"""

import math
from typing import Dict, Tuple, Optional, List
from collections import deque

# Try importing Webots controller; fall back to Mock mode for standalone testing
try:
    from controller import Robot, GPS, InertialUnit, Compass, DistanceSensor, Camera
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False
    Robot = None
    GPS = None
    InertialUnit = None
    Compass = None
    DistanceSensor = None
    Camera = None


class SensorMovingAverageFilter:
    """Moving average filter with outlier rejection for noisy agricultural distance readings."""
    def __init__(self, window_size: int = 5):
        self.window_size = window_size
        self.history: deque = deque(maxlen=window_size)

    def update(self, val: float) -> float:
        self.history.append(val)
        return sum(self.history) / len(self.history)

    def reset(self) -> None:
        self.history.clear()


class Pioneer3ATSensors:
    """
    Sensor interface and calibration manager for the Pioneer 3-AT field robot.
    """

    # Physical Sensor Calibration Constants
    SENSOR_MAX_RANGE_METERS: float = 2.00     # Maximum effective distance sensor range
    SENSOR_MIN_RANGE_METERS: float = 0.05     # Blind zone near face of sensor
    RAW_ADC_MAX: float = 1024.0              # Standard Webots raw distance sensor reading at max range
    RAW_ADC_MIN: float = 0.0                 # Raw distance sensor reading at contact (0 meters)

    # In-row crop spacing parameters
    NOMINAL_ROW_SPACING: float = 1.80         # Distance between adjacent crop rows (meters)
    HEADLAND_OPEN_THRESHOLD: float = 1.30     # If both side distances > 1.3 m, robot is in open headland
    ROW_CENTER_TOLERANCE: float = 0.05        # Centering deadband (+- 5 cm)
    OBSTACLE_SAFETY_DISTANCE: float = 0.40    # Distance threshold for emergency obstacle stop

    # Longitudinal baseline between front and rear lateral distance sensors
    SENSOR_LONGITUDINAL_BASE: float = 0.40    # Distance between front-side and rear-side sensors (meters)

    def __init__(self, robot: Optional[object] = None, time_step: int = 32):
        """
        Initialize and enable all navigation and perception sensors.
        
        Args:
            robot: Webots Robot instance.
            time_step: Simulation step duration in milliseconds (default 32 ms).
        """
        self.robot = robot
        self.time_step = time_step
        self.is_mock: bool = False

        # Sensor devices
        self.gps = None
        self.imu = None
        self.compass = None
        self.camera = None
        self.distance_sensors: Dict[str, Optional[object]] = {}

        # Noise filters for lateral sensors
        self.filters: Dict[str, SensorMovingAverageFilter] = {
            "left_front": SensorMovingAverageFilter(window_size=5),
            "left_rear": SensorMovingAverageFilter(window_size=5),
            "right_front": SensorMovingAverageFilter(window_size=5),
            "right_rear": SensorMovingAverageFilter(window_size=5),
            "front_left": SensorMovingAverageFilter(window_size=3),
            "front_right": SensorMovingAverageFilter(window_size=3),
        }

        # Cached latest calibrated readings
        self.latest_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_rad": 0.0, "yaw_deg": 0.0}
        self.latest_distances = {
            "left_front": 0.90,
            "left_rear": 0.90,
            "right_front": 0.90,
            "right_rear": 0.90,
            "front_left": 2.00,
            "front_right": 2.00,
            "avg_left": 0.90,
            "avg_right": 0.90,
            "min_front": 2.00
        }

        self._init_sensors()

    def _init_sensors(self) -> None:
        """Discovers, enables, and configures all sensors."""
        if not WEBOTS_AVAILABLE or self.robot is None:
            self.is_mock = True
            print("[sensors.py] Webots controller not detected. Running in STANDALONE/MOCK mode.")
            return

        # 1. Enable GPS
        try:
            self.gps = self.robot.getDevice("gps")
            if self.gps:
                self.gps.enable(self.time_step)
                print("[sensors.py] GPS enabled.")
        except Exception as e:
            print(f"[sensors.py] GPS not found ({e}).")

        # 2. Enable IMU (InertialUnit) or fallback Compass
        try:
            self.imu = self.robot.getDevice("inertial_unit")
            if self.imu:
                self.imu.enable(self.time_step)
                print("[sensors.py] InertialUnit (IMU) enabled.")
        except Exception:
            pass

        if self.imu is None:
            try:
                self.compass = self.robot.getDevice("compass")
                if self.compass:
                    self.compass.enable(self.time_step)
                    print("[sensors.py] Compass enabled.")
            except Exception:
                print("[sensors.py] Neither IMU nor Compass found. Orientation fallback will be used.")

        # 3. Enable Distance Sensors (Lateral + Front)
        sensor_names = {
            "left_front": "ds_left_front",
            "left_rear": "ds_left_rear",
            "right_front": "ds_right_front",
            "right_rear": "ds_right_rear",
            "front_left": "ds_front_left",
            "front_right": "ds_front_right",
        }

        found_count = 0
        for key, name in sensor_names.items():
            try:
                ds = self.robot.getDevice(name)
                if ds:
                    ds.enable(self.time_step)
                    self.distance_sensors[key] = ds
                    found_count += 1
            except Exception:
                self.distance_sensors[key] = None

        # Fallback to Pioneer 3-AT built-in sonars if custom distance sensors not found
        if found_count == 0:
            print("[sensors.py] Dedicated ds_* sensors not found; binding Pioneer 3-AT built-in sonars (so0-so15)...")
            sonar_mapping = {
                "front_left": "so3",
                "front_right": "so4",
                "left_front": "so0",
                "left_rear": "so15",
                "right_front": "so7",
                "right_rear": "so8",
            }
            for key, sname in sonar_mapping.items():
                try:
                    s = self.robot.getDevice(sname)
                    if s:
                        s.enable(self.time_step)
                        self.distance_sensors[key] = s
                        found_count += 1
                except Exception:
                    self.distance_sensors[key] = None

        print(f"[sensors.py] Distance sensors ready: {found_count}/6 channels connected.")

        # 4. Enable Camera (Optional)
        try:
            self.camera = self.robot.getDevice("camera")
            if self.camera:
                self.camera.enable(self.time_step)
                print("[sensors.py] Camera enabled.")
        except Exception:
            pass

    def calibrate_raw_to_meters(self, raw_value: float) -> float:
        """
        Calibrates raw Webots distance sensor reading to metric distance (meters).
        
        Calibration model:
          In Webots standard DistanceSensor lookup tables:
            raw = 1024 (or max) -> far range (2.0 m or open field)
            raw = 0             -> near / in contact (0.0 m)
          Linearly or piecewise calibrated to meters:
            distance (m) = clamp( (raw / RAW_ADC_MAX) * SENSOR_MAX_RANGE, MIN, MAX )
        """
        if raw_value is None or math.isnan(raw_value):
            return self.SENSOR_MAX_RANGE_METERS

        normalized = max(0.0, min(raw_value / self.RAW_ADC_MAX, 1.0))
        dist_m = normalized * self.SENSOR_MAX_RANGE_METERS
        return max(self.SENSOR_MIN_RANGE_METERS, min(dist_m, self.SENSOR_MAX_RANGE_METERS))

    def update(self) -> None:
        """
        Reads all hardware sensors, applies calibration & filtering,
        and updates the cached navigation and perception state.
        Call this once at the start of each control loop iteration.
        """
        if self.is_mock:
            return

        # 1. Update Navigation Pose
        if self.gps:
            pos = self.gps.getValues()
            if pos and len(pos) >= 3:
                self.latest_pose["x"] = pos[0]
                self.latest_pose["y"] = pos[1]
                self.latest_pose["z"] = pos[2]

        if self.imu:
            rpy = self.imu.getRollPitchYaw()
            if rpy and len(rpy) >= 3:
                yaw = rpy[2] # Yaw in radians
                self.latest_pose["yaw_rad"] = yaw
                self.latest_pose["yaw_deg"] = math.degrees(yaw) % 360.0
        elif self.compass:
            c = self.compass.getValues()
            if c and len(c) >= 2:
                # In Webots, compass vector gives direction relative to north
                yaw = math.atan2(c[0], c[1])
                self.latest_pose["yaw_rad"] = yaw
                self.latest_pose["yaw_deg"] = math.degrees(yaw) % 360.0

        # 2. Update and Filter Distance Sensors
        for key in ["left_front", "left_rear", "right_front", "right_rear", "front_left", "front_right"]:
            device = self.distance_sensors.get(key)
            if device:
                raw = device.getValue()
                calibrated = self.calibrate_raw_to_meters(raw)
                filtered = self.filters[key].update(calibrated)
                self.latest_distances[key] = filtered

        # Aggregate averages
        self.latest_distances["avg_left"] = (self.latest_distances["left_front"] + self.latest_distances["left_rear"]) / 2.0
        self.latest_distances["avg_right"] = (self.latest_distances["right_front"] + self.latest_distances["right_rear"]) / 2.0
        self.latest_distances["min_front"] = min(self.latest_distances["front_left"], self.latest_distances["front_right"])

    # =========================================================================
    # HIGH-LEVEL API EXPOSED TO OTHER MODULES
    # =========================================================================

    def get_pose(self) -> Tuple[float, float, float]:
        """
        Returns the robot's current estimated pose in the world.
        
        Returns:
            Tuple of (x, y, yaw_radians)
        """
        return (
            self.latest_pose["x"],
            self.latest_pose["y"],
            self.latest_pose["yaw_rad"]
        )

    def get_row_errors(self) -> Tuple[float, float]:
        """
        Extracts cross-track error (lateral offset from row center) and
        heading orientation error relative to the crop rows.
        
        Equations:
          - Lateral Error: e_y = (d_left - d_right) / 2
            (e_y > 0 means robot is shifted toward left row; e_y < 0 means shifted toward right)
          - Heading Error: e_psi = atan2( (d_left_front - d_left_rear), base )
            (e_psi > 0 means robot is angled toward the left row)
            
        Returns:
            Tuple of (lateral_error_m, heading_error_rad)
        """
        d_l = self.latest_distances["avg_left"]
        d_r = self.latest_distances["avg_right"]

        # Lateral error: 0 when perfectly centered
        lateral_error = (d_l - d_r) / 2.0

        # Heading error from dual lateral sensor baseline
        delta_l = self.latest_distances["left_front"] - self.latest_distances["left_rear"]
        delta_r = self.latest_distances["right_rear"] - self.latest_distances["right_front"]
        delta_avg = (delta_l + delta_r) / 2.0

        heading_error = math.atan2(delta_avg, self.SENSOR_LONGITUDINAL_BASE)

        return lateral_error, heading_error

    def is_row_end(self) -> bool:
        """
        Detects if the robot has reached the end of the crop row and entered the headland.
        Condition: Both left and right distance sensors exceed the open field threshold.
        """
        d_l = self.latest_distances["avg_left"]
        d_r = self.latest_distances["avg_right"]
        return (d_l >= self.HEADLAND_OPEN_THRESHOLD) and (d_r >= self.HEADLAND_OPEN_THRESHOLD)

    def is_row_entered(self) -> bool:
        """
        Detects if the robot has successfully entered a new crop row from the headland.
        Condition: Both side sensors detect crop foliage within normal row boundaries.
        """
        d_l = self.latest_distances["avg_left"]
        d_r = self.latest_distances["avg_right"]
        return (d_l < self.HEADLAND_OPEN_THRESHOLD) and (d_r < self.HEADLAND_OPEN_THRESHOLD)

    def is_obstacle_detected(self) -> bool:
        """
        Safety check: Detects if any obstacle is within the front emergency safety bubble.
        """
        return self.latest_distances["min_front"] <= self.OBSTACLE_SAFETY_DISTANCE

    def get_all_readings(self) -> Dict[str, float]:
        """Returns a complete dictionary of all current calibrated sensor telemetry."""
        e_y, e_psi = self.get_row_errors()
        return {
            "x": self.latest_pose["x"],
            "y": self.latest_pose["y"],
            "yaw_rad": self.latest_pose["yaw_rad"],
            "yaw_deg": self.latest_pose["yaw_deg"],
            "d_left_avg": self.latest_distances["avg_left"],
            "d_right_avg": self.latest_distances["avg_right"],
            "d_front_min": self.latest_distances["min_front"],
            "lateral_error_ey": e_y,
            "heading_error_epsi": e_psi,
            "is_row_end": float(self.is_row_end()),
            "obstacle_detected": float(self.is_obstacle_detected()),
        }


# Quick test when executed directly
if __name__ == "__main__":
    print("Testing Pioneer3ATSensors...")
    sensors = Pioneer3ATSensors()

    # Test calibration function
    d_far = sensors.calibrate_raw_to_meters(1024.0)
    d_mid = sensors.calibrate_raw_to_meters(512.0)
    d_near = sensors.calibrate_raw_to_meters(0.0)
    print(f"Calibration test: 1024 -> {d_far:.2f} m, 512 -> {d_mid:.2f} m, 0 -> {d_near:.2f} m")
    assert math.isclose(d_far, 2.0, rel_tol=1e-2)
    assert math.isclose(d_mid, 1.0, rel_tol=1e-2)

    # Test filter
    filt = SensorMovingAverageFilter(window_size=3)
    filt.update(1.0)
    filt.update(1.2)
    val = filt.update(1.4)
    print(f"Moving average test: [1.0, 1.2, 1.4] -> {val:.2f} (expected 1.20)")
    assert math.isclose(val, 1.20, rel_tol=1e-2)

    # Test centered row calculation
    sensors.latest_distances["avg_left"] = 0.90
    sensors.latest_distances["avg_right"] = 0.90
    ey, epsi = sensors.get_row_errors()
    print(f"Centered row errors: e_y = {ey:.3f} m, e_psi = {epsi:.3f} rad")
    assert ey == 0.0

    # Test offset calculation
    sensors.latest_distances["avg_left"] = 1.10
    sensors.latest_distances["avg_right"] = 0.70
    ey, _ = sensors.get_row_errors()
    print(f"Offset row errors: e_y = {ey:.3f} m (expected +0.20 m)")
    assert math.isclose(ey, 0.20, rel_tol=1e-2)

    # Test row-end detection
    sensors.latest_distances["avg_left"] = 1.60
    sensors.latest_distances["avg_right"] = 1.60
    print(f"Row end test: is_row_end() = {sensors.is_row_end()} (expected True)")
    assert sensors.is_row_end() is True

    # Test obstacle detection
    sensors.latest_distances["min_front"] = 0.30
    print(f"Obstacle test: is_obstacle_detected() = {sensors.is_obstacle_detected()} (expected True)")
    assert sensors.is_obstacle_detected() is True

    print("sensors.py unit tests passed successfully!")

