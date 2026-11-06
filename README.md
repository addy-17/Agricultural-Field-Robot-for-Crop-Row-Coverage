# Agricultural Field Robot for Crop-Row Coverage
**Course:** CSAI4014 - Algorithms for Intelligent Systems and Robotics (AISR)  
**Degree:** B.Tech CSE (AI & ML), Semester VII  
**Simulation Platform:** Webots (Outdoor Agricultural Field World)  
**Robot Platform:** Pioneer 3-AT (4-Wheel Skid-Steer Mobile Robot)  
**Language:** Python 3  

---

## 🌾 Project Overview
This project develops an autonomous agricultural field robot capable of complete crop-row coverage. Navigating parallel crop rows, the robot performs:
1. **In-Row Guidance:** Centering between crop rows using lateral distance sensor arrays and PID / Stanley steering controllers.
2. **Headland Turnaround Maneuvers:** Smooth non-holonomic turnaround trajectories ($\Omega$-turn / continuous bulb turn vs. in-place pivot turn) to switch between adjacent rows without damaging crops.
3. **Safety & Obstacle Avoidance:** Proximity detection for emergency braking upon sensing rogue obstacles.
4. **Complete Coverage Scheduling:** Boustrophedon (back-and-forth) path planning across all crop rows.
5. **Real-Time Telemetry & Benchmarking:** Quantitative evaluation of tracking error, mission completion time, energy efficiency, and field coverage percentage.

---

## 📁 Repository Structure
```text
.
├── .gitignore
├── README.md
├── AISR_Project_Instructions_B1-B2_14-10-2026.pdf
├── AISR_Team_Division_and_Task_Assignment.md
├── docs/
│   └── sensor_calibration_notes.md
├── worlds/
│   └── crop_field.wbt
└── controllers/
    └── agricultural_robot/
        ├── __init__.py
        ├── motion.py
        ├── sensors.py
        └── test_member2_modules.py
```

---

## 🛠️ Modules & Deliverables

### Member 2: Robot Model, Sensors & Motion Interface
- **Robot & World:** [`worlds/crop_field.wbt`](worlds/crop_field.wbt) — Webots agricultural field with 4 parallel crop rows (spacing $1.8\text{ m}$, length $16\text{ m}$), soil ground terrain, and headland turn clearings.
- **Motion Interface:** [`controllers/agricultural_robot/motion.py`](controllers/agricultural_robot/motion.py) — Wheel kinematics, velocity limits, and high-level motion helper methods (`move_forward`, `move_backward`, `turn_left`, `turn_right`, `stop`, `set_velocity`).
- **Sensors Interface:** [`controllers/agricultural_robot/sensors.py`](controllers/agricultural_robot/sensors.py) — GPS, IMU, Compass, lateral distance sensors, front collision sensors, noise filtering, and error extraction ($e_y, e_\psi$).
- **Calibration Notes:** [`docs/sensor_calibration_notes.md`](docs/sensor_calibration_notes.md) — Coordinate frames, sensor mounting, calibration curves, and verification test benchmarks.
- **Automated Test Suite:** [`controllers/agricultural_robot/test_member2_modules.py`](controllers/agricultural_robot/test_member2_modules.py) — Standalone unit tests (6/6 passing).

---

## 🚀 Quickstart & Verification

### Running the Member 2 Automated Unit Tests
No Webots installation required for standalone testing:
```bash
python controllers/agricultural_robot/test_member2_modules.py
```

### Running in Webots
1. Open **Webots**.
2. Go to **File -> Open World...** and select `worlds/crop_field.wbt`.
3. Press the **Play** button to run the simulation.
