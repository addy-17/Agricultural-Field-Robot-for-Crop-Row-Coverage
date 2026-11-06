# CSAI4014 - Algorithms for Intelligent Systems and Robotics (AISR)
## Project Task Division & Member Assignment Document
**Batch:** B1 - B2 | **Academic Year:** 2026 | **Degree:** B.Tech CSE (AI & ML), Semester VII  
**Project Title:** Agricultural Field Robot for Crop-Row Coverage  
**Platform & Tools:** Webots Simulator (Outdoor Field Environment, Pioneer 3-AT / Custom Wheeled Robot), Python Controller, Git/GitHub, Overleaf / MS Word  

---

## 1. Executive Summary & Team Organization

| Member ID | Role / Module Ownership | Primary Technical Domain | Code Deliverables | Report Sections | Slide Deck Allocation |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **M1 (Lead)** | Systems Engineer & Mission Planner | Kinematics & Coverage Path Planning (CCPP) | `kinematics.py`, `mission_planner.py` | Cover, Abstract, Sec 1, 2, 13 | Slides 1, 2, 3 |
| **M2** | Robot Model, Sensors & Motion Interface | Robot Setup, Sensor Calibration & Motion Abstraction | `crop_field.wbt`, `sensors.py`, `motion.py`, `calibration_notes` | Sec 3, 4 (Novelty) | Slides 4, 5, 6 |
| **M3** | Guidance & In-Row Control Engineer | Row-Centering & Obstacle Avoidance Controller | `row_follower.py`, `pid_controller.py` | Sec 5, 6 | Slides 7, 8, 9 |
| **M4** | Headland Maneuvers & Integration Lead | Headland Trajectory & Main Supervisor Loop | `headland_turn.py`, `main_controller.py` | Sec 7, 8 | Slides 10, 11, 12 (Live Demo) |
| **M5** | Benchmark & Telemetry Analyst | Data Logging, Error Analysis & Evaluation | `metrics_logger.py`, `plot_results.py` | Sec 9, 12 | Slides 13, 14, 15 |
| **ALL** | Joint Responsibilities | System Integration, Peer Assessment, Viva Prep | GitHub Review, Integration Testing | Sec 10 (Own Row), 11 (Reflection), Dec | Slides 16, 17 (Q&A) |

---

## 2. Detailed Technical Module Breakdown by Member

Every member is responsible for engineering an independent, testable **Software Module** with well-defined inputs, algorithmic logic, outputs, and standalone unit tests. This ensures that every member has demonstrable individual code commits in GitHub (Rubric B1).

```
                      +------------------------------------------+
                      |   Member 1: CoverageMissionPlanner       |
                      |   - State Machine (Row / Headland / Stop)|
                      |   - Waypoints & Target Row Scheduler     |
                      +--------------------+---------------------+
                                           |
                        Operational State  | Target Row & Mode
                                           v
+-------------------------+   +----------------------------+   +-----------------------------+
| Member 2:               |   | Member 3:                  |   | Member 4:                   |
| CropRowPerception       |   | InRowGuidanceController    |   | HeadlandTurnGenerator       |
|                         |   |                            |   |                             |
| Inputs: DistanceSensors |   | Inputs: ey, e_psi from M2  |   | Inputs: Target Row from M1  |
| Logic:  Filter & Center |==>| Logic:  PID / Stanley      |   | Logic:  Omega-Turn Traj Gen |
| Output: ey, e_psi,      |   | Output: (v_row, omega_row) |   | Output: (v_turn, omega_turn)|
|         row_end_detected|   +--------------+-------------+   +--------------+--------------+
+------------+------------+                  |                                |
             |                               +----------------+---------------+
             | Row End Detected                               |
             +----------------------------+                   | Selected (v, omega)
                                          v                   v
                              +------------------------------------+
                              | Member 4 (Integration Loop):       |
                              | Master Webots Controller Dispatch  |
                              | Actuator Velocity Commands         |
                              +-----------------+------------------+
                                                |
                                                | Telemetry Stream (x, y, theta, errors, time)
                                                v
                              +------------------------------------+
                              | Member 5:                          |
                              | TelemetryAndBenchmarkEvaluator     |
                              | - CSV Data Logger                  |
                              | - Comparative Metrics (Method A/B) |
                              | - Matplotlib Graph Plotter         |
                              +------------------------------------+
```

---

### Module 1: Coverage Mission Planner & Kinematics Engine
* **Assigned Member:** Member 1 (Team Lead & Systems Engineer)
* **Module Code Files:** `controllers/mission_planner.py`, `controllers/kinematics.py`
* **Module Description:**  
  Acts as the "Brain" of the agricultural robot. Manages the complete field coverage schedule using a Boustrophedon (back-and-forth) path plan and maintains a finite state machine (FSM) to supervise system behavior across row tracking and headland turning. Also provides the differential/skid-steer kinematic equations to translate linear/angular velocities into wheel angular velocities.
* **Module Specifications:**
  - **Inputs:** Current robot pose $(x, y, \theta)$, total number of rows ($N$), row spacing ($W=1.8\text{ m}$), feedback flags from M2 (`row_end_detected`) and M4 (`turn_completed`).
  - **Internal Logic:**  
    - State Machine states: `INIT`, `IN_ROW_TRACKING`, `HEADLAND_TRANSITION`, `ROW_TURN`, `EMERGENCY_STOP`, `MISSION_COMPLETE`.
    - Skid-steer forward/inverse kinematics: $\omega_L = \frac{v - \omega \cdot \frac{L}{2}}{r}$, $\omega_R = \frac{v + \omega \cdot \frac{L}{2}}{r}$ (where $L$ is effective wheelbase and $r$ is wheel radius).
  - **Outputs:** Current state (`STATE`), active target row ID, next row direction (`LEFT` or `RIGHT`), motor speed conversions.
* **Standalone Testing (Independent of Webots):**  
  M1 writes a Python unit test (`tests/test_mission_planner.py`) that feeds dummy state transitions and verifies that rows are visited in correct alternating order (Row 1 $\to$ Turn Left $\to$ Row 2 $\to$ Turn Right $\to$ Row 3) and shuts down cleanly after completing all rows.

---

### Module 2: Robot Model, Sensors & Motion Interface
* **Assigned Member:** Member 2
* **Module Deliverables:** Robot model/configuration (`worlds/crop_field.wbt`), `sensors.py`, `motion.py`, `docs/sensor_calibration_notes.md`, `tests/test_member2_modules.py`
* **Core Responsibilities & Technical Implementation:**
  1. **Robot Model & World Configuration (`worlds/crop_field.wbt`):**  
     Configured Pioneer 3-AT 4-wheel skid-steer mobile robot in an outdoor Webots agricultural world with 4 parallel crop rows (row spacing $1.8\text{ m}$, row length $16\text{ m}$), $4\text{ m}$ headland turn clearings, and ground terrain.
  2. **Motion Interface (`controllers/agricultural_robot/motion.py`):**  
     Configured physical wheel/motor parameters (radius $r = 0.111\text{ m}$, effective track width $L = 0.40\text{ m}$, max speed limit $6.40\text{ rad/s}$) and built high-level Python helper functions (`move_forward`, `move_backward`, `turn_left`, `turn_right`, `stop`, `set_velocity`). This exposes a clean, high-level interface so other team members (M1, M3, M4) can command desired linear and angular velocities without dealing with low-level motor names, registers, or wheel speed conversions.
  3. **Sensor Interface & Fusion (`controllers/agricultural_robot/sensors.py`):**  
     Configured navigation sensors (GPS, IMU/compass) and perception sensors (calibrated lateral and front distance sensor arrays, RGB camera). Implemented moving average filtering to suppress crop foliage noise, and provided high-level state accessors: `get_pose()`, `get_row_errors()` (cross-track error $e_y$ and heading error $e_\psi$), `is_row_end()`, and `is_obstacle_detected()`.
  4. **Sensor Calibration & Verification Notes (`docs/sensor_calibration_notes.md`):**  
     Calibrated distance sensor ADC readings to true meters, documented coordinate frames (ISO 8855), and verified stable, deterministic readings across 5 benchmark scenarios (centered, offset, angled heading, headland exit, obstacle).
* **Standalone Testing (Independent of Webots):**  
  Member 2 authored `test_member2_modules.py` covering 6 comprehensive automated test cases (kinematic equations, motion helper methods, sensor calibration curve, moving average filter, row error extraction, and row-end/obstacle triggers) achieving 100% test pass rate.

---

### Module 3: In-Row Guidance & Safety Controller
* **Assigned Member:** Member 3 (Guidance & In-Row Control Engineer)
* **Module Code Files:** `controllers/row_follower.py`, `controllers/pid_controller.py`, `controllers/obstacle_avoidance.py`
* **Module Description:**  
  Acts as the "Driver" within crop rows. Implements closed-loop feedback controllers to keep the vehicle centered between crop rows at steady forward velocity ($v = 0.6\text{ m/s}$) while dynamically adjusting steering to eliminate lateral and heading errors. Incorporates front obstacle detection for emergency braking.
* **Module Specifications:**
  - **Inputs:** `lateral_error` ($e_y$) and `heading_error` ($e_\psi$) from M2, front proximity distance sensors ($d_{front}$).
  - **Internal Logic:**  
    - **Method 1 (Baseline PID):** $\omega = K_p e_y + K_i \int e_y dt + K_d \frac{de_y}{dt} + K_{yaw} e_\psi$.
    - **Method 2 (Advanced / Stanley Control):** Steering angle $\delta = e_\psi + \arctan\left(\frac{k \cdot e_y}{v + \epsilon}\right)$.
    - Front safety bubble: If $d_{front} < 0.4\text{ m}$, emergency stop activated ($v = 0$).
  - **Outputs:** Commanded linear velocity ($v_{cmd}$), commanded angular steering velocity ($\omega_{cmd}$), `obstacle_detected` flag.
* **Standalone Testing (Independent of Webots):**  
  M3 creates `tests/test_controller.py` simulating standard step responses (e.g. initial lateral offset $e_y = 0.25\text{ m}$) to demonstrate controller stability, settling time $< 2.5\text{ s}$, and zero steady-state error.

---

### Module 4: Headland Maneuver Generator & Simulation Integrator
* **Assigned Member:** Member 4 (Headland Maneuvers & Simulation Integration Lead)
* **Module Code Files:** `controllers/headland_turn.py`, `controllers/main_robot_controller.py`
* **Module Description:**  
  Acts as the "Navigator" in headland zones and "Integrator" for Webots. Generates smooth turnaround trajectories to switch between adjacent crop rows without colliding with crop boundaries or tearing the soil. Synchronizes all 5 modules inside the Webots main simulation step loop.
* **Module Specifications:**
  - **Inputs:** Target row distance ($W = 1.8\text{ m}$), turn direction (`LEFT` / `RIGHT`), robot pose $(x, y, \theta)$, selected turn mode (Method A vs. Method B).
  - **Internal Logic:**  
    - **Turn Strategy A (Baseline Point Turn):** Skid-steer $90^\circ$ rotation $\to$ linear displacement $W \to 90^\circ$ entry rotation.
    - **Turn Strategy B (Advanced Continuous $\Omega$-Turn / Bulb Turn):** Dubins-curve bulb trajectory parameterization that swings outward to respect minimum turning radius ($R_{min} \ge 0.8\text{ m}$) and smoothly aligns into the next row centerline.
    - Coordinates synchronous execution: `robot.step(TIME_STEP)`.
  - **Outputs:** Commanded turn velocities ($v_{turn}, \omega_{turn}$), `turn_completed` flag.
* **Standalone Testing (Independent of Webots):**  
  M4 writes `tests/test_headland_turn.py` that computes the $(x, y)$ coordinates of both turn paths and plots them using Matplotlib to mathematically prove minimum turn radius compliance and smooth alignment with the target row.

---

### Module 5: Telemetry Logger & Benchmarking Evaluator
* **Assigned Member:** Member 5 (Benchmarking, Telemetry & Results Analysis Engineer)
* **Module Code Files:** `controllers/metrics_logger.py`, `scripts/plot_results.py`
* **Module Description:**  
  Acts as the "Data Analyst & Evaluator". Intercepts telemetry from all modules during runtime, logs ground truth and estimated states to CSV files, and computes quantitative performance metrics to benchmark Method A against Method B for the mandatory experimental results section.
* **Module Specifications:**
  - **Inputs:** Timestep $t$, robot pose $(x, y, \theta)$, cross-track error $e_y$, commanded speeds $(v, \omega)$, motor currents/torques, state flag.
  - **Internal Logic:**  
    - High-frequency asynchronous CSV logger (`telemetry_log.csv`).
    - Metric calculation engine:
      - Field Coverage Percentage: $\text{Coverage} = \frac{\text{Area Covered}}{\text{Total Field Area}} \times 100\%$
      - Root-Mean-Square Cross-Track Error: $\text{RMSE} = \sqrt{\frac{1}{N}\sum_{i=1}^N e_{y,i}^2}$
      - Total Mission Time ($T_{mission}$)
      - Energy / Wheel Slip Index: $\int |\omega(t)| dt$
    - Automated generation of publication-quality plots (PNG/PDF) for the report.
  - **Outputs:** `telemetry_log.csv`, `benchmark_comparison.csv`, automated plots (Path Trajectory, Error vs. Time, Step Response).
* **Standalone Testing (Independent of Webots):**  
  M5 writes `tests/test_logger.py` with mock simulation logs to ensure statistical metrics and matplotlib graphs are generated automatically without errors.

---
* **Primary Responsibilities:**
  1. Coordinate team deliverables, GitHub organization, meeting logs, and submission compliance.
  2. Mathematical modeling of the robot kinematics: 4-wheel skid-steer mobile robot kinematics (Pioneer 3-AT), instantaneous center of rotation (ICR), wheel speed forward and inverse kinematic transformations.
  3. High-level coverage path planning (CCPP): Implementation of the Boustrophedon (back-and-forth) field coverage pattern across parallel crop rows.
  4. Global supervisor finite state machine (FSM): States `INITIALIZE`, `ROW_TRACKING`, `HEADLAND_EXIT`, `HEADLAND_TURN`, `ROW_ENTRY`, and `MISSION_COMPLETE`.
* **Code Files Owned:**
  - `controllers/kinematics.py`: Skid-steer forward/inverse kinematic solvers and velocity-to-motor command mapping.
  - `controllers/mission_planner.py`: Row sequence scheduler, headland entry/exit state machine, waypoint generator.
* **Report Writing Allocation (Approx. 5–6 pages):**
  - Title Page, Declaration, Index (Table of Contents, List of Figures/Tables).
  - **Abstract** (150–200 words): Problem definition, methodology, Webots environment, comparative results, and novelty summary.
  - **Section 1: Introduction** (~2 pages): Background of precision agriculture, importance of autonomous field scouting/spraying, labor shortages, operational constraints in row crops.
  - **Section 2: Problem Statement and Objectives** (~1.5 pages): 4-5 measurable SMART objectives (e.g., >95% field coverage, mean lateral row error < 0.08 m, collision-free headland transitions), boundary conditions, and operating limits.
  - **Section 13: References** (IEEE style, numbered [1]–[8]+, fully cited).
* **Video & Slide Presentation:**
  - **Slides 1–3**: Slide 1 (Title Slide), Slide 2 (Problem & Motivation in Industry), Slide 3 (Project Objectives).
  - Video time: ~3.5 minutes.

---

### Member 2 (M2) — Perception & Simulation Environment Engineer
* **Primary Responsibilities:**
  1. Constructing the realistic outdoor agricultural Webots simulation world (`crop_field.wbt`): textured soil terrain, parallel crop rows (using custom height/canopy bounding boxes or realistic foliage models), customizable row spacing (1.8 m), row length (15 m), headland turning clearance (3.5 m).
  2. Sensor configuration and placement on Pioneer 3-AT: Front planar LiDAR / RangeFinder, lateral ultrasonic/infrared distance sensors (left and right arrays), IMU/compass, and GPS module.
  3. Perception algorithm design: Processing sensor point clouds and lateral distance ranges to detect crop foliage boundaries, calculating the robot's lateral cross-track error ($e_y$) from the row centerline and orientation heading error ($e_\psi$).
  4. End-of-row and headland boundary detection logic (sensing when the crop boundary terminates into open headland).
* **Code Files Owned:**
  - `worlds/crop_field.wbt`: Webots 3D agricultural world with custom lighting, terrain, crop rows, and obstacle props.
  - `controllers/perception.py`: Range finder/distance sensor filtering, centerline extractor, and row-end detector.
  - `controllers/filters.py`: Low-pass / moving average noise filtering on raw range measurements.
* **Report Writing Allocation (Approx. 4–5 pages):**
  - **Section 3: Literature and Industry Review** (~2–3 pages): Review of state-of-the-art agricultural robotics (e.g., John Deere autonomous tractors, Naïo Technologies, SwarmFarm), academic literature on row detection (computer vision vs. LiDAR/sonar), and identified research gaps.
  - **Section 4: NOVELTY (Mandatory)** (~1.5–2 pages): 3–5 explicit novelty points (e.g., adaptive multi-sensor row-centering robust to missing plants/gaps, hybrid curvature headland transition without external GNSS RTK base stations, comparative analysis between kinematic turn models).
* **Video & Slide Presentation:**
  - **Slides 4–6**: Slide 4 (Existing Solutions & Gaps), Slide 5 (NOVELTY - 3 to 5 core points), Slide 6 (Syllabus Mapping: Sensing, Representation & Coordinate Frames).
  - Video time: ~3.5 minutes.

---

### Member 3 (M3) — Guidance & In-Row Control Engineer
* **Primary Responsibilities:**
  1. Design and tuning of the reactive in-row centering controller: keeping the robot equidistant between crop rows while advancing at nominal linear speed ($v = 0.5 - 0.8\text{ m/s}$).
  2. Implementation of **Method 1 vs. Method 2** for row following:
     - *Method 1 (Baseline):* Dual lateral distance PID controller ($u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$).
     - *Method 2 (Advanced / Comparative):* Lookahead Stanley / Pure Pursuit Controller or Adaptive Gain PID that dynamically accounts for heading error and lateral displacement simultaneously.
  3. Obstacle avoidance / emergency braking inside rows: Detecting rogue obstacles (rocks, fallen debris, animal models) within the inter-row lane and executing reactive speed reduction or safe stopping.
  4. Mathematical stability analysis and controller gain tuning ($K_p, K_i, K_d$).
* **Code Files Owned:**
  - `controllers/row_follower.py`: In-row lateral guidance laws and differential steering velocity generation.
  - `controllers/pid_controller.py`: Generic anti-windup PID controller class with configurable gains and saturation limits.
  - `controllers/obstacle_avoidance.py`: Front proximity safety bubble and emergency braking module.
* **Report Writing Allocation (Approx. 4–5 pages):**
  - **Section 5: Syllabus Mapping and Theory** (~2–3 pages): Connecting project modules directly to CSAI4014 syllabus topics (Differential kinematics, mobile robot localization, PID feedback control loops, Lyapunov stability, state estimation).
  - **Section 6: System Design** (~2 pages): High-level system architecture block diagram, hardware-in-the-loop software architecture, sensor-actuator communication pipeline, Pioneer 3-AT motor specifications.
* **Video & Slide Presentation:**
  - **Slides 7–9**: Slide 7 (System Architecture & Dataflow), Slide 8 (Robot Kinematic & Environment Model), Slide 9 (Algorithm 1: In-Row Centering & PID Control).
  - Video time: ~3.5 minutes.

---

### Member 4 (M4) — Headland Maneuvers & Simulation Integration Lead
* **Primary Responsibilities:**
  1. Development of the Headland Turn Trajectory Generator: When transitioning from row $i$ to row $i+1$, the robot enters the unconstrained headland area where row guides vanish.
  2. Implementation of two headland turning strategies for comparative evaluation:
     - *Turn Strategy A (Baseline):* Stop-and-Turn / In-place Point Turn (skid-steer $90^\circ \to \text{traverse offset} \to 90^\circ$).
     - *Turn Strategy B (Advanced Continuous Curvature):* Smooth $\Omega$-Turn (Omega / Bulb turn) or Dubins Curve path ensuring minimum turning radius without wheel gouging or soil degradation.
  3. Master Webots simulation coordinator: Integrating modules from M1, M2, M3, M4, and M5 into the main synchronous robot execution loop.
  4. Leading and executing the **Live Simulation Demonstration** during the recorded presentation meeting.
* **Code Files Owned:**
  - `controllers/headland_turn.py`: Parametric generation of smooth Omega / Bulb turnaround paths for switching between adjacent rows.
  - `controllers/main_robot_controller.py`: Main Webots Python entry point (`Robot()` step loop, sensor read, state dispatch, actuator write).
  - `run_demo.py` / shell scripts: Automated simulation launcher for clean demonstration runs.
* **Report Writing Allocation (Approx. 4–5 pages):**
  - **Section 7: Methodology and Algorithms** (~3–4 pages): Detailed algorithmic flowcharts, pseudocode for row switching and headland maneuver generation, state transition logic, and parameter tables.
  - **Section 8: Simulation Setup** (~1–2 pages): Webots configuration (world step size $32\text{ ms}$, physics solver parameters), robot dimension specs, dependencies (`requirements.txt`), command-line instructions to run the simulator.
* **Video & Slide Presentation:**
  - **Slides 10–12**: Slide 10 (Algorithm 2: Headland Turning Trajectory Generation), Slide 11 (Simulation Setup & Parameters), Slide 12 (Live Simulation Demonstration Transition).
  - **Live Demo Lead:** Conducts the live screen-share demonstration of Webots running the agricultural field coverage mission (~5–6 minutes).

---

### Member 5 (M5) — Benchmarking, Telemetry & Results Analysis Engineer
* **Primary Responsibilities:**
  1. Real-time telemetry data logging: Recording time, robot coordinates $(x, y, \theta)$, ground truth position, sensor readings, motor speeds, and error values at every timestep into structured CSV/JSON format.
  2. Performance metrics formulation and automated evaluation:
     - Field Coverage Percentage ($>95\%$).
     - Root Mean Square (RMS) & Mean Absolute Lateral Error ($e_y$) during row following.
     - Total Mission Completion Time (seconds).
     - Trajectory smoothness and energy index (integral of wheel torque / angular acceleration).
  3. Experimental benchmarking: Conducting comparative test runs (e.g., Baseline PID + In-place Turn vs. Advanced Stanley/Adaptive + Smooth $\Omega$-Turn under clean vs. noisy/disturbed conditions).
  4. Generation of publication-quality plots (Matplotlib/Seaborn): Trajectory overhead map overlaid on crop field, error response vs. time, step response curves, and comparative metric bar charts.
* **Code Files Owned:**
  - `controllers/metrics_logger.py`: In-simulation data collector writing telemetry to `data/telemetry_log.csv`.
  - `scripts/plot_results.py`: Post-processing script producing comparative plots and summary statistic tables.
  - `data/results_comparison.csv`: Processed benchmark results.
* **Report Writing Allocation (Approx. 4–5 pages):**
  - **Section 9: Results and Analysis** (~3–4 pages): Detailed plots (Figure captions, path plots, cross-track error distributions), quantitative performance tables comparing both methods, critical engineering discussion on limitations and edge cases (e.g., wheel slip on dirt).
  - **Section 10: Contribution Section Coordination** (Aggregating all member contributions into the mandatory Section 10 table).
  - **Section 12: Conclusion and Future Scope** (~1 page): Summary of key discoveries, industrial viability, real-world deployment roadmap (transition from Webots to ROS 2 / physical AgBot hardware).
* **Video & Slide Presentation:**
  - **Slides 13–15**: Slide 13 (Results 1: Trajectory Tracking & Error Response Curves), Slide 14 (Results 2: Comparative Performance Table & Analysis), Slide 15 (Team Technical Contribution Table).
  - Video time: ~3.5 minutes.

---

## 3. Mandatory Report Compliance Tables

### Section D: Writing Allocation Table
*(To be inserted immediately following the Index / Table of Contents in the report)*

| Member | Name & SAP ID | Assigned Report Sections | Estimated Pages |
| :---: | :---: | :--- | :---: |
| **M1** | [Student 1 Name - SAP ID] | Cover, Abstract, Section 1 (Introduction), Section 2 (Problem Statement & Objectives), Section 13 (References) | 5 – 6 |
| **M2** | [Student 2 Name - SAP ID] | Section 3 (Literature & Industry Review), Section 4 (NOVELTY - Mandatory) | 4 – 5 |
| **M3** | [Student 3 Name - SAP ID] | Section 5 (Syllabus Mapping and Theory), Section 6 (System Design) | 4 – 5 |
| **M4** | [Student 4 Name - SAP ID] | Section 7 (Methodology and Algorithms), Section 8 (Simulation Setup) | 5 – 6 |
| **M5** | [Student 5 Name - SAP ID] | Section 9 (Results and Analysis), Section 12 (Conclusion and Future Scope) | 5 – 6 |
| **ALL** | All 5 Members | Declaration (signed by all), Section 10 (Individual technical row & signature), Section 11 (Co-learning individual reflection), Appendix (Meeting logs, Peer assessment) | 3 – 4 |
| **TOTAL** | **Entire Group** | **Complete Project Report (Single-sided, Times New Roman 12pt, 1.5 spacing, single border)** | **26 – 32 Pages** |

---

### Section 10: Mandatory Technical Contribution Table
*(To be included as Section 10 of the final printed and digital report; each member signs their row)*

| S. No. | Member Name & SAP ID | Technical Contribution (Modules / Algorithms / Code Files / Experiments Done) | Report Sections Written | Signature |
| :---: | :--- | :--- | :--- | :---: |
| **1** | **[Student 1 Name]**<br>SAP: [ID 1] | Developed skid-steer kinematics model; built Boustrophedon coverage mission planner and finite state machine (`kinematics.py`, `mission_planner.py`); managed GitHub repo structure and CI. | Abstract, Sec 1, 2, 13 | ________ |
| **2** | **[Student 2 Name]**<br>SAP: [ID 2] | Configured Pioneer 3-AT in outdoor crop-field world (`crop_field.wbt`); engineered motion interface with velocity/turn abstraction (`motion.py`); configured & calibrated GPS, IMU, and lateral/front distance sensor arrays with noise filter (`sensors.py`); authored sensor calibration notes and test suite (`test_member2_modules.py`). | Sec 3, 4 | ________ |
| **3** | **[Student 3 Name]**<br>SAP: [ID 3] | Designed in-row guidance laws; implemented and tuned dual lateral PID controller and pure pursuit steering; built reactive obstacle avoidance safety bubble (`row_follower.py`, `pid_controller.py`, `obstacle_avoidance.py`). | Sec 5, 6 | ________ |
| **4** | **[Student 4 Name]**<br>SAP: [ID 4] | Implemented non-holonomic headland turning algorithms ($\Omega$-turn vs. in-place pivot); integrated all modules into Webots master step loop (`headland_turn.py`, `main_robot_controller.py`); coordinated live demo execution. | Sec 7, 8 | ________ |
| **5** | **[Student 5 Name]**<br>SAP: [ID 5] | Developed real-time telemetry logging system; conducted comparative benchmark experiments; plotted trajectory, tracking error, and step response graphs; compiled quantitative metric tables (`metrics_logger.py`, `plot_results.py`). | Sec 9, 12 | ________ |

---

## 4. 17-Slide Presentation Plan (Strictly Compliant with Section 3)

| Slide # | Slide Title | Key Content & Visual Elements | Designated Presenter |
| :---: | :--- | :--- | :---: |
| **1** | Title Slide | Project title, Group No., Batch B1-B2, Names & SAP IDs, Faculty Name, Course Name & Code | **M1** |
| **2** | Problem and Motivation | Real-world agricultural challenges, labor shortages, crop damage, need for autonomous row navigation | **M1** |
| **3** | Project Objectives | 3–5 SMART measurable objectives (coverage %, cross-track error threshold, non-stop transition) | **M1** |
| **4** | Existing Solutions & Industry Gap | Current state of commercial AgBots (GPS-dependent, bulky), open challenges in GNSS-denied/canopy rows | **M2** |
| **5** | Project Novelty | 3–5 explicit novelty contributions: hybrid sensor row-centering, smooth $\Omega$-turn without crop trampling | **M2** |
| **6** | Syllabus Mapping | Theory linkage: Mobile robot kinematics, differential drive, sensor modeling, PID control, path planning | **M2** |
| **7** | System Architecture | End-to-end hardware/software block diagram (Sensors $\to$ Perception $\to$ FSM $\to$ Control $\to$ Actuators) | **M3** |
| **8** | Robot & Environment Model | Pioneer 3-AT specifications, skid-steer ICR kinematics, Webots outdoor crop row world geometry | **M3** |
| **9** | Algorithm 1: In-Row Guidance | Flowchart & mathematical formulation of lateral row centering (PID / Stanley steering law) | **M3** |
| **10** | Algorithm 2: Headland Maneuvers | Mathematical path formulation of Headland $\Omega$-turn vs. In-place Pivot turnaround with transition states | **M4** |
| **11** | Simulation Setup & Execution | Webots simulation environment, sensor parameters, simulation timestep, execution commands | **M4** |
| **12** | **Live Demonstration** | **Switch to live Webots screen-share: Full simulation run demonstrating row coverage and turning** | **M4 (Lead)** |
| **13** | Results 1: Trajectory & Error Curves | Overhead trajectory coverage plots, cross-track error ($e_y$) over time, heading angle stability plots | **M5** |
| **14** | Results 2: Comparative Analysis | Benchmark table comparing Method A (Pivot Turn) vs. Method B ($\Omega$-Turn), speed, error, energy | **M5** |
| **15** | Team Technical Contribution | Detailed summary of each member's technical deliverables (traceable to GitHub commits) | **M5** |
| **16** | Conclusion & Industrial Roadmap | Key engineering takeaways, deployment to physical robot, ROS 2 integration, edge-AI scalability | **ALL** |
| **17** | End Slide (Q&A & Links) | GitHub repository URL, YouTube unlisted video link, team contact info, Q&A invite | **ALL** |

---

## 5. 20-Minute Video Recording Strategy (18–22 Min Limit)

* **Platform:** Google Meet / Microsoft Teams / Zoom (Recorded with full gallery view so all 5 members have cameras turned ON throughout the entire duration).
* **Structure:**
  - **0:00 – 1:00 (All Members):** Introduction. Each member states their full name, SAP ID, and role on camera.
  - **1:00 – 4:00 (Member 1):** Slides 1–3 (Problem context, motivation, measurable project objectives).
  - **4:00 – 7:00 (Member 2):** Slides 4–6 (State of the art, 4 clear novelty points, syllabus theoretical mapping).
  - **7:00 – 10:00 (Member 3):** Slides 7–9 (System block diagram, Pioneer 3-AT kinematics, in-row guidance PID/Stanley algorithm).
  - **10:00 – 15:30 (Member 4):** Slides 10–11 (Headland turning algorithms, simulation configuration) + **LIVE WEBOTS DEMO (5–6 mins)** showing the robot starting, navigating down crop rows, detecting row end, executing a continuous $\Omega$-turn, and entering the adjacent row smoothly.
  - **15:30 – 18:30 (Member 5):** Slides 13–15 (Quantitative results, error plots, comparative table, code traceability).
  - **18:30 – 20:00 (All Members):** Slides 16–17 (Concluding remarks, future hardware migration, acknowledgment, GitHub repo overview).
* **YouTube Upload:** Must be uploaded as **UNLISTED** (Never Public, Never Private).

---

## 6. GitHub Repository Structure & Commit Protocol

To guarantee full marks under Criterion B1 (Individual Technical Contribution traceable to commits), the group repository must strictly follow this directory structure:

```text
agricultural-field-robot/
├── README.md                          <- Project overview, quickstart instructions, team details
├── requirements.txt                   <- Python dependencies (numpy, matplotlib, scipy)
├── docs/
│   ├── AISR_Project_Report.pdf        <- Final formatted report
│   └── AISR_Presentation_Slides.pdf   <- 17-slide deck
├── worlds/
│   └── crop_field.wbt                 <- Webots 3D agricultural world [M2]
├── controllers/
│   └── agricultural_robot_controller/
│       ├── agricultural_robot_controller.py <- Master entry point [M4]
│       ├── kinematics.py              <- Pioneer 3-AT kinematic solver [M1]
│       ├── mission_planner.py         <- Coverage FSM & row scheduler [M1]
│       ├── perception.py              <- RangeFinder / LiDAR row detection [M2]
│       ├── filters.py                 <- Sensor noise filtering [M2]
│       ├── row_follower.py            <- In-row PID / guidance controller [M3]
│       ├── pid_controller.py          <- Generic PID control algorithm [M3]
│       ├── obstacle_avoidance.py      <- Collision safety stop module [M3]
│       ├── headland_turn.py           <- Omega / Dubins headland turning [M4]
│       └── metrics_logger.py          <- Telemetry data logger [M5]
├── scripts/
│   ├── plot_results.py                <- Result plotting & comparative graphs [M5]
│   └── run_simulation.bat             <- Quick launch script [M4]
└── data/
    ├── baseline_telemetry.csv         <- Experimental dataset (Method A) [M5]
    └── advanced_telemetry.csv         <- Experimental dataset (Method B) [M5]
```

### Git Branching and Traceability Rule:
- Every member commits **strictly from their own personal GitHub account**.
- Dedicated feature branches: `feature/m1-kinematics-planner`, `feature/m2-webots-world-perception`, `feature/m3-inrow-controller`, `feature/m4-headland-turn-integration`, `feature/m5-telemetry-benchmarking`.
- All features merged to `main` via pull requests with descriptive commit messages.

---

## 7. Meeting Log & Peer-Assessment Templates

### First Team Meeting Log (Mandatory for Appendix)
- **Meeting Date & Time:** Day 1, 10:00 AM – 11:30 AM
- **Mode:** Online / In-person
- **Attendees:** All 5 members present
- **Agenda:** Election of Team Lead, review of AISR Project Instructions, division of modules M1–M5, selection of Webots environment (Pioneer 3-AT on agricultural terrain), establishment of GitHub repository.
- **Decisions Recorded:**
  1. Member 1 appointed Team Lead.
  2. Modules assigned as detailed in Section 2.
  3. GitHub repository initialized with individual feature branch workflow.
  4. Novelty agreed: Dual-sensor adaptive row centering paired with smooth non-holonomic $\Omega$-turn headland traversal compared against baseline in-place skid-steer turns.

### Peer-Assessment Sheet Template (Mandatory for Appendix)
*(Every member completes one evaluation form for their 4 peers on a 1–5 scale)*

| Evaluator Name & SAP | Teammate Name Evaluated | Technical Contribution (1–5) | Reliability & Timeliness (1–5) | Helped Others Learn (1–5) | Quality of Work (1–5) |
| :--- | :--- | :---: | :---: | :---: | :---: |
| [Member X Name] | Member 1 | 5 | 5 | 5 | 5 |
| [Member X Name] | Member 2 | 5 | 5 | 5 | 5 |
| [Member X Name] | Member 3 | 5 | 5 | 5 | 5 |
| [Member X Name] | Member 4 | 5 | 5 | 5 | 5 |
| [Member X Name] | Member 5 | 5 | 5 | 5 | 5 |

---

## 8. Evaluation Checklist & Penalty Prevention Guide

| Item / Rule | Constraint in Instructions | Mitigation & Assurance in this Plan |
| :--- | :--- | :--- |
| **Report Length** | 25 to 35 pages total (including front matter) | Strictly balanced at 26–32 pages (~5 pages/member) |
| **Report Formatting** | Times New Roman, 12pt, 1.5 line spacing, single border on EVERY page, A4 portrait | Explicitly assigned to team formatting guidelines |
| **Mandatory Sections** | Section 4 (Novelty) & Section 10 (Contribution) MANDATORY | Both defined with granular technical detail |
| **Slide Deck Limit** | Maximum 17 slides TOTAL (including title and end slide) | Exact 17-slide plan mapped 1-to-1 to project topics |
| **Video Duration** | 18 to 22 minutes (Penalty P5 if outside) | Timed script allocating ~3.5 min/member + 5.5 min live demo |
| **Video Visibility** | Every member visible with camera ON, speaking own part | Grid view recording mandated; individual speaking slots |
| **Comparison** | Must compare at least TWO methods or settings | Method A (PID + In-place Turn) vs. Method B (Adaptive + $\Omega$-Turn) |
| **Submission** | Hard copy bound + Report PDF + 17 Slides PDF + GitHub link + Unlisted YouTube link | Coordinated single-point submission by Team Lead (M1) |

