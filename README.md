# Agricultural Field Robot for Crop-Row Coverage
**Course:** CSAI4014 - Algorithms for Intelligent Systems and Robotics (AISR)  
**Degree:** B.Tech CSE (AI & ML), Semester VII  
**Simulation Platform:** Webots (Outdoor Agricultural Field World)  
**Robot Platform:** Pioneer 3-AT (4-Wheel Skid-Steer Mobile Robot)  
**Language:** Python 3  

--- 

## 🌾 Project Overview
This project presents a fully autonomous agricultural field robot engineered for complete crop-row coverage in precision agriculture. Navigating narrow parallel crop rows without relying on external RTK-GPS, the Pioneer 3-AT mobile robot executes:
1. **In-Row Guidance:** Lateral distance sensor fusion with dual-loop PID and non-linear Stanley steering controllers.
2. **Headland Turnaround Maneuvers:** Continuous curvature $\Omega$-turn (Bulb Turn) eliminating soil tearing, preserving crop root beds, and saving 15.4% mission transit time compared to baseline in-place skid turns.
3. **Reactive Safety Bubble:** Front proximity collision detection with proportional deceleration and emergency active braking ($d \le 0.40\text{ m}$).
4. **Complete Coverage Planning (CCPP):** Boustrophedon path planning across all crop rows supervised by a Finite State Machine (FSM).
5. **Quantitative Telemetry & Benchmarking:** Statistical comparative evaluation across tracking accuracy, mission duration, field coverage, and mechanical steering energy.

---

## 📁 Repository Structure
```text
Agricultural-Field-Robot-for-Crop-Row-Coverage/
├── README.md                                          <- Project overview & quickstart guide
├── run_all_tests.py                                   <- Automated master test suite (all modules)
├── AISR_Project_Instructions_B1-B2_14-10-2026.pdf     <- Course instructions & evaluation rubrics
├── AISR_Team_Division_and_Task_Assignment.md          <- 5-member modular task & writing allocation
├── data/
│   ├── telemetry_method_a.csv                        <- Telemetry dataset (Method A Baseline)
│   └── telemetry_method_b.csv                        <- Telemetry dataset (Method B Advanced)
├── docs/
│   ├── field_config.json                             <- Machine-readable agricultural field dimensions
│   ├── sensor_calibration_notes.md                   <- M2 sensor calibration & verification notes
│   ├── presentation_slides.md                        <- 17-slide presentation deck script
│   ├── results_benchmark_table.md                    <- Section 9 comparative performance table
│   ├── peer_assessment_and_meeting_logs.md           <- Meeting logs & individual peer reviews
│   └── figures/
│       ├── trajectory_comparison.png                 <- 2D field overhead path comparison
│       ├── lateral_error_comparison.png              <- Cross-track error (e_y) vs. time plot
│       └── metrics_comparison_barchart.png           <- Quantitative performance bar chart
├── worlds/
│   └── crop_field.wbt                                <- Webots outdoor agricultural field with Pioneer 3-AT
├── scripts/
│   └── plot_results.py                               <- Data analysis & publication figure generator
└── controllers/
    └── agricultural_robot/
        ├── __init__.py                               <- Module exports
        ├── agricultural_robot.py                     <- Master Webots controller (multithreaded loop)
        ├── kinematics.py                             <- Pioneer 3-AT kinematics & odometry [M1]
        ├── mission_planner.py                        <- Boustrophedon planner & supervisor FSM [M1]
        ├── motion.py                                 <- Wheel motor abstraction & kinematics [M2]
        ├── sensors.py                                <- Sensor conditioning, calibration & filter [M2]
        ├── pid_controller.py                         <- Robust discrete PID with anti-windup [M3]
        ├── row_follower.py                           <- PID & Stanley in-row steering laws [M3]
        ├── obstacle_avoidance.py                     <- Reactive safety bubble & braking [M3]
        ├── headland_turn.py                          <- Point Turn & Omega-Turn generators [M4]
        ├── metrics_logger.py                         <- Real-time CSV telemetry data logger [M5]
        └── test_member2_modules.py                   <- M2 unit test suite
```

---

## 👥 Modular Team Distribution & Ownership

| Member | Technical Domain | Key Deliverables | Report Sections | Slide Deck Slots |
| :--- | :--- | :--- | :--- | :--- |
| **Member 1 (Lead)** | **Mission Planning & Kinematics** | `kinematics.py`, `mission_planner.py` | Abstract, Sec 1, 2, 13 | Slides 1, 2, 3 |
| **Member 2** | **Robot Model, Sensors & Motion** | `worlds/crop_field.wbt`, `motion.py`, `sensors.py`, `calibration_notes` | Sec 3, 4 (Novelty) | Slides 4, 5, 6 |
| **Member 3** | **Guidance & In-Row Control** | `pid_controller.py`, `row_follower.py`, `obstacle_avoidance.py` | Sec 5, 6 | Slides 7, 8, 9 |
| **Member 4** | **Headland Turns & Master Integrator** | `headland_turn.py`, `agricultural_robot.py` | Sec 7, 8 | Slides 10, 11, 12 (Live Demo) |
| **Member 5** | **Benchmarking & Telemetry Analysis** | `metrics_logger.py`, `scripts/plot_results.py`, `figures/` | Sec 9, 12 | Slides 13, 14, 15 |
| **ALL** | **Joint Integration & Review** | System validation, meeting logs, peer evaluation | Sec 10, 11, Declaration | Slides 16, 17 (Q&A) |

---

## 📊 Experimental Results Summary (Section 9)

| Metric | Method A (Baseline PID + In-Place Pivot) | Method B (Stanley Lookahead + Continuous $\Omega$-Turn) | Improvement |
| :--- | :---: | :---: | :---: |
| **Total Mission Duration** | 92.4 s | **78.2 s** | **15.4% Faster** |
| **Mean Absolute Lateral Error ($e_y$)** | 3.65 cm | **1.45 cm** | **60.3% Error Reduction** |
| **Tracking RMSE ($e_y$)** | 4.35 cm | **1.82 cm** | **58.2% Accuracy Gain** |
| **Maximum Lateral Deviation** | 8.20 cm | **3.40 cm** | **58.5% Tighter Margin** |
| **Field Coverage Percentage** | 98.4% | **99.6%** | **Complete Coverage** |
| **Steering Mechanical Roughness Index** | 48.6 rad | **26.4 rad** | **45.7% Smoother** |
| **Soil Gouging & Rutting** | Severe at pivot points | **Zero (Continuous Curvature)** | **Protects Crop Root Beds** |

---

## 🚀 How to Run & Verify

### 1. Automated Standalone Verification (No Webots GUI required)
Run all 5 member module tests simultaneously:
```bash
python run_all_tests.py
```

### 2. Generate Figures & Benchmark Tables
```bash
python scripts/plot_results.py
```

### 3. Run in Webots Simulator
1. Open **Webots**.
2. Click **File -> Open World...** and select `worlds/crop_field.wbt`.
3. Press **Play (`▶`)** on the top toolbar to run the autonomous simulation.
