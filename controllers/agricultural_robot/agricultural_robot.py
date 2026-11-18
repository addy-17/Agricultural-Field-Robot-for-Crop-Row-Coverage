"""
agricultural_robot.py - Master Webots controller (FIXED: pose-based lane coverage)
Wall-sensing, smooth-arc lane coverage of lanes y=0, +2.2, -2.2. Change LAPS to repeat more/less.
"""
import sys, os, math
CONTROLLER_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(CONTROLLER_DIR)

try:
    from controller import Robot
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False
    Robot = None

from motion import Pioneer3ATMotion
from sensors import Pioneer3ATSensors
from smooth_navigator import SmoothNavigator
from metrics_logger import TelemetryMetricsLogger

LAPS = 2   # 1 lap = lanes 0, +2.2, -2.2 once. Set higher to repeat more.


class MasterAgriculturalRobot:
    def __init__(self, laps=LAPS):
        self.robot = Robot() if WEBOTS_AVAILABLE else None
        self.time_step = int(self.robot.getBasicTimeStep()) if WEBOTS_AVAILABLE else 32
        if self.time_step <= 0:
            self.time_step = 32
        self.dt = self.time_step / 1000.0
        self.motion = Pioneer3ATMotion(robot=self.robot)
        self.sensors = Pioneer3ATSensors(robot=self.robot, time_step=self.time_step)
        self.nav = SmoothNavigator(laps=laps)
        self.logger = TelemetryMetricsLogger(method_name="Method_B")
        self.sim_time = 0.0
        self.step_count = 0
        self.total_lanes = self.nav.total

    def run(self):
        while True:
            if WEBOTS_AVAILABLE:
                if self.robot.step(self.time_step) == -1:
                    break
            elif self.step_count >= 1500:
                break
            self.step_count += 1
            self.sim_time += self.dt

            self.sensors.update()
            x, y, yaw = self.sensors.get_pose()
            z = self.sensors.latest_pose.get("z", 0.0)
            d = self.sensors.latest_distances     # filtered wall / front distance sensors

            v, w, state = self.nav.update(x, y, yaw, d, self.dt)
            if state == "MISSION_COMPLETE":
                self.motion.stop()
                print(f"[MISSION COMPLETE] {self.nav.done} lanes in {self.sim_time:.1f}s")
                break

            wl, wr = self.motion.set_velocity(linear_velocity=v, angular_velocity=w)
            e_y = self.sensors.get_row_errors()[0]    # wall-sensed cross-track error
            cov = 100.0 * self.nav.done / self.total_lanes
            self.logger.log_step(time_s=self.sim_time, state=state, x=x, y=y, z=z, yaw_rad=yaw,
                                 lateral_error=e_y, heading_error=0.0, cmd_v=v, cmd_omega=w,
                                 wheel_left=wl, wheel_right=wr, coverage_pct=cov, dt=self.dt)
            if self.step_count % 30 == 0:
                print(f"[{state[:12]:12s}] t={self.sim_time:6.1f}s lane#{self.nav.done+1}/{self.total_lanes} "
                      f"pos=({x:6.2f},{y:5.2f}) yaw={math.degrees(yaw):7.1f} v={v:.2f} w={w:+.2f}")
        self.motion.stop()
        self.logger.print_summary()


if __name__ == "__main__":
    MasterAgriculturalRobot().run()
