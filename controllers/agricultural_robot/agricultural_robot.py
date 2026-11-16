"""
agricultural_robot.py - Main Webots Robot Controller
CSAI4014 AISR Project | Agricultural Field Robot for Crop-Row Coverage

Main executable for the Pioneer 3-AT 4-wheel mobile robot in Webots.
Integrates:
  - Pioneer3ATMotion (wheel kinematics & low-level motor abstraction)
  - Pioneer3ATSensors (GPS, IMU, sonar/distance arrays, noise filter)
"""

import sys
import os

# Add controller directory to system path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from controller import Robot
    WEBOTS_AVAILABLE = True
except ImportError:
    WEBOTS_AVAILABLE = False
    Robot = None

from motion import Pioneer3ATMotion
from sensors import Pioneer3ATSensors


def main():
    print("=" * 60)
    print("  AGRICULTURAL FIELD ROBOT CONTROLLER (PIONEER 3-AT)")
    print("  CSAI4014 - Algorithms for Intelligent Systems and Robotics")
    print("=" * 60)

    if not WEBOTS_AVAILABLE:
        print("[ERROR] Webots Python API 'controller' not found.")
        print("Please run this controller inside Webots or verify WEBOTS_HOME.")
        return

    # Instantiate Webots Robot
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    if time_step <= 0:
        time_step = 32

    print(f"[Controller] Initializing Pioneer 3-AT subsystems (TimeStep: {time_step} ms)...")

    # Initialize Member 2 Motion & Sensors Hardware Abstraction Layer
    motion = Pioneer3ATMotion(robot=robot)
    sensors = Pioneer3ATSensors(robot=robot, time_step=time_step)

    print("[Controller] Pioneer 3-AT online and ready. Starting autonomous in-row navigation...")

    # Control loop parameters
    FORWARD_SPEED = 0.45       # m/s nominal transit speed inside crop row
    KP_STEERING = 1.20         # Proportional steering gain for lateral row centering
    KP_HEADING = 0.60          # Heading alignment gain
    MAX_ANGULAR_VEL = 1.20     # rad/s steering saturation clamp

    step_count = 0

    # Main Webots Simulation Step Loop
    while robot.step(time_step) != -1:
        step_count += 1

        # 1. Update all sensors (GPS, IMU, Distance sensors with noise filtering)
        sensors.update()

        # 2. Extract state telemetry
        x, y, yaw = sensors.get_pose()
        e_y, e_psi = sensors.get_row_errors()
        obstacle = sensors.is_obstacle_detected()
        row_end = sensors.is_row_end()

        # 3. Decision & Control Logic
        if obstacle:
            # Emergency Stop / Safety Bubble triggered
            motion.stop()
            if step_count % 30 == 0:
                print(f"[SAFETY] Step {step_count}: Obstacle detected ahead! Pioneer 3-AT braking.")
        elif row_end:
            # Reached headland boundary
            motion.set_velocity(0.20, 0.0)
            if step_count % 30 == 0:
                print(f"[HEADLAND] Step {step_count}: End of row reached at x={x:.2f}, y={y:.2f}. Slowing for turn.")
        else:
            # In-Row Closed Loop Guidance:
            # Steering correction to minimize lateral offset (e_y) and heading error (e_psi)
            steering_omega = (KP_STEERING * (-e_y)) + (KP_HEADING * (-e_psi))
            # Clamp angular velocity
            steering_omega = max(-MAX_ANGULAR_VEL, min(steering_omega, MAX_ANGULAR_VEL))

            # Command velocities to Pioneer 3-AT 4 wheels
            motion.set_velocity(linear_velocity=FORWARD_SPEED, angular_velocity=steering_omega)

            # Periodic status telemetry logging
            if step_count % 25 == 0:
                readings = sensors.get_all_readings()
                print(f"[Pioneer 3-AT] Step {step_count:04d} | Pos: ({x:.2f}, {y:.2f}) | "
                      f"e_y: {e_y:+.3f}m | Left_d: {readings['d_left_avg']:.2f}m | "
                      f"Right_d: {readings['d_right_avg']:.2f}m | Steer: {steering_omega:+.3f} rad/s")

    print("[Controller] Webots simulation stopped.")


if __name__ == "__main__":
    main()
