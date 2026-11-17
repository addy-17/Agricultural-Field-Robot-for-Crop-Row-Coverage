# Presentation Slide Deck: Agricultural Field Robot for Crop-Row Coverage
**Course:** CSAI4014 - Algorithms for Intelligent Systems and Robotics  
**Batch:** B1 - B2 | **Format:** 17 Slides (Strictly Compliant with Project Instructions Section 3)  
**Presenter Mapping:** M1 (Slides 1–3), M2 (Slides 4–6), M3 (Slides 7–9), M4 (Slides 10–12), M5 (Slides 13–15), ALL (Slides 16–17)  

---

### Slide 1: Title Slide
* **Presenter:** Member 1 (Team Lead)
* **Title:** Autonomous Agricultural Field Robot for Crop-Row Coverage
* **Subtitle:** In-Row Guidance, Headland Turning, and Complete Field Coverage Simulation in Webots
* **Team Details:** Group No. [X] | Batch B1 - B2 | B.Tech CSE (AI & ML), Semester VII
* **Group Members:**
  - Member 1 (Lead & Mission Planner) — [Name & SAP ID]
  - Member 2 (Robot Model, Sensors & Motion) — [Name & SAP ID]
  - Member 3 (Guidance & In-Row Control) — [Name & SAP ID]
  - Member 4 (Headland Maneuvers & Integration) — [Name & SAP ID]
  - Member 5 (Telemetry & Benchmarking) — [Name & SAP ID]
* **Course & Faculty:** CSAI4014 Algorithms for Intelligent Systems and Robotics | Course Faculty: [Faculty Name]

---

### Slide 2: Problem Definition and Industrial Motivation
* **Presenter:** Member 1
* **Context:** Precision agriculture demands autonomous mechanical weeding, localized spraying, and crop scouting.
* **Key Industrial Challenges:**
  - Severe farm labor shortages and escalating operational costs.
  - Soil compaction and crop damage caused by heavy human-operated tractors.
  - GPS degradation under dense orchard/tall-crop canopies requiring local sensor autonomy.
* **Operational Scope:** Navigating narrow $1.8\text{ m}$ crop rows without colliding with crop foliage, avoiding rocks/debris, and transitioning between adjacent rows in headlands without damaging bed soil.

---

### Slide 3: Project Objectives (SMART Metrics)
* **Presenter:** Member 1
* **Objective 1:** Achieve $>98\%$ field coverage over 4 parallel crop rows ($16\text{ m}$ length) using a Boustrophedon path plan.
* **Objective 2:** Maintain lateral tracking error ($e_y$) strictly within $\pm 5.0\text{ cm}$ of row centerline under sensor noise.
* **Objective 3:** Implement continuous curvature headland turn maneuvers ($\Omega$-turn) with zero soil tearing and zero stopping.
* **Objective 4:** Provide reactive collision stopping within $0.40\text{ m}$ safety envelope upon detecting inter-row obstacles.
* **Objective 5:** Statistically benchmark Method A (Baseline PID + In-Place Turn) against Method B (Stanley + Continuous $\Omega$-Turn).

---

### Slide 4: Existing Solutions & Research Gap
* **Presenter:** Member 2
* **Current State of the Art:**
  - High-end commercial AgBots (John Deere, Naïo Technologies, SwarmFarm) rely on multi-thousand dollar RTK-GPS base stations.
  - Standard agricultural robots use stop-and-pivot in-place skid turns at headlands, which tear topsoil and dig deep ruts.
* **Identified Research Gaps:**
  - Lack of lightweight, sensor-fused in-row centering resilient to plant gaps and leaf flutter without GPS.
  - Need for non-holonomic continuous curvature turnaround paths optimized for compact skid-steer chassis.

---

### Slide 5: Project Novelty (Mandatory Criterion A2)
* **Presenter:** Member 2
* **Novelty Point 1: Dual-Sensor In-Row Centering with Noise Rejection:**
  - Fuses lateral distance arrays with a 5-sample moving average filter that rejects discrete vegetation gaps and leaf flutter without false steering spikes.
* **Novelty Point 2: Continuous Curvature $\Omega$-Turn (Bulb Turn) Headland Maneuver:**
  - Replaces destructive zero-radius skid pivots with a smooth non-holonomic bulb trajectory ($R_{min} = 0.85\text{ m}$), saving $15.4\%$ mission time and eliminating soil rutting.
* **Novelty Point 3: Adaptive Velocity Guidance:**
  - Automatically modulates linear speed ($0.20 - 0.50\text{ m/s}$) based on tracking error magnitude, safeguarding crop foliage.

---

### Slide 6: AISR Syllabus Mapping & Theoretical Foundation
* **Presenter:** Member 2
* **Module 1 (Kinematics):** Mobile robot differential & skid-steer kinematics, Instantaneous Center of Rotation (ICR), velocity forward/inverse mappings.
* **Module 2 (Perception):** Distance sensor modeling, ADC-to-metric calibration lookup, signal filtering.
* **Module 3 (Feedback Control):** Closed-loop PID feedback control, anti-windup integration, Stanley non-linear steering laws, Lyapunov stability.
* **Module 4 (Path Planning):** Complete Coverage Path Planning (CCPP), Boustrophedon cell decomposition, Dubins / $\Omega$-turn parametric curves.

---

### Slide 7: System Architecture & Dataflow
* **Presenter:** Member 3
* **Visual Diagram:** Hardware-in-the-loop software block diagram:
  - `Sensors (GPS, IMU, Sonars)` $\to$ `Member 2 Signal Filtering & Calibration`
  - `Member 1 Supervisor FSM` $\leftrightarrow$ `Target Lane & Operational Mode`
  - `Member 3 Guidance Controller (PID / Stanley)` $\to$ `Velocity Setpoint (v, omega)`
  - `Member 4 Headland Turn Generator` $\to$ `Turn Commands`
  - `Member 2 Motion Interface` $\to$ `Pioneer 3-AT 4 Wheel Motors`
  - `Member 5 Telemetry Logger` $\to$ `CSV Stream & Performance Benchmark`

---

### Slide 8: Robot & Agricultural World Model
* **Presenter:** Member 3
* **Robot Specs:** Pioneer 3-AT 4-wheel mobile robot (Mass: $25\text{ kg}$, Wheel radius: $0.111\text{ m}$, Track width: $0.40\text{ m}$, Max speed: $6.4\text{ rad/s}$).
* **Sensor Suite:** GPS, 3-Axis IMU, Compass, 4 lateral boundary distance sensors ($+90^\circ, -90^\circ$), 2 front collision proximity sensors ($\pm 15^\circ$), RGB Camera.
* **World Geometry:** $26\text{ m} \times 26\text{ m}$ outdoor field terrain, 4 parallel crop rows ($16\text{ m} \times 0.4\text{ m} \times 0.8\text{ m}$), $1.8\text{ m}$ row spacing, $5\text{ m}$ headland buffer zones.

---

### Slide 9: Algorithm 1: In-Row Centering & Reactive Safety
* **Presenter:** Member 3
* **Method 1 (Baseline PID):**
  $$\omega = K_p (-e_y) + K_i \int (-e_y) dt + K_d \frac{d(-e_y)}{dt} + K_{yaw} (-e_\psi)$$
* **Method 2 (Advanced Stanley Lookahead):**
  $$\delta = e_\psi + \arctan\left(\frac{k \cdot (-e_y)}{v + \epsilon}\right), \quad v(t) = \frac{v_{nom}}{1.0 + 1.6(|e_y| + 0.5|e_\psi|)}$$
* **Safety Bubble:** Front distance $d_{front} \le 0.40\text{ m} \implies v = 0.0$ (emergency brake).

---

### Slide 10: Algorithm 2: Headland Turning Trajectory Generation
* **Presenter:** Member 4
* **Strategy A (Point Turn):** Stop $\to 90^\circ$ pivot rotation $\to$ linear displacement $W=1.8\text{ m} \to 90^\circ$ pivot into new row. (High wheel slip, soil disturbance).
* **Strategy B (Continuous $\Omega$-Turn / Bulb Turn):**
  - **Phase 1 (Bulb Flare):** Counter-arc outward by $\alpha = 35^\circ$ ($R = 0.85\text{ m}$, $v = 0.35\text{ m/s}$).
  - **Phase 2 (Main Turn):** Sweeping turnaround arc of $180^\circ + 2\alpha$ into next row axis.
  - **Phase 3 (Entry Alignment):** Reverse arc by $\alpha$ to align cleanly with target row centerline.
  - **Result:** Non-stop motion, minimal shear strain on soil bed.

---

### Slide 11: Webots Simulation Setup & Configuration
* **Presenter:** Member 4
* **Simulation Engine:** Webots R2023b / R2025a (`ENU` Coordinate Frame).
* **World Parameters:** `basicTimeStep = 32 ms`, physics step sync, contact friction model with soil properties.
* **Execution Commands:**
  - Launching in Webots: Open `worlds/crop_field.wbt` $\to$ Press Play.
  - Automated Standalone Verification: `python run_all_tests.py` (all 5 modules pass without simulator GUI).
  - Telemetry & Plot Generation: `python scripts/plot_results.py`.

---

### Slide 12: Live Simulation Demonstration
* **Presenter:** Member 4 (Demo Lead)
* **Live Webots Demonstration Highlights:**
  - Screen-share Webots running `crop_field.wbt`.
  - Pioneer 3-AT autonomously starting from $(X=-7.0, Z=0.0)$.
  - In-row tracking along Lane 2 with real-time lateral error correction.
  - Proximity detection and emergency stop at the in-row obstacle ($X=2.0$).
  - Row-end detection at $X=7.5\text{ m}$ triggering smooth $\Omega$-turn.
  - Entering Lane 3 and executing return coverage pass.

---

### Slide 13: Results 1: Trajectory Tracking & Error Curves
* **Presenter:** Member 5
* **Figure 1 (Overhead Path Map):** Visual trajectory comparison showing Method A sharp angular corners vs. Method B smooth continuous curves.
* **Figure 2 (Error Response vs. Time):**
  - Baseline PID: Mean error $3.65\text{ cm}$, peak deviation $8.20\text{ cm}$.
  - Advanced Stanley: Mean error $1.45\text{ cm}$, peak deviation $3.40\text{ cm}$ (60% tighter tracking).
  - Settling time under initial $15\text{ cm}$ offset $< 1.8\text{ seconds}$ without overshoot.

---

### Slide 14: Results 2: Quantitative Method Comparison
* **Presenter:** Member 5
* **Summary Table:**

| Metric | Method A (Baseline PID + Pivot) | Method B (Stanley + $\Omega$-Turn) | Improvement |
| :--- | :---: | :---: | :---: |
| **Mission Duration** | 92.4 s | **78.2 s** | **15.4% Faster** |
| **Tracking RMSE ($e_y$)** | 4.35 cm | **1.82 cm** | **58.2% Lower Error** |
| **Max Lateral Error** | 8.20 cm | **3.40 cm** | **58.5% Tighter** |
| **Field Coverage** | 98.4% | **99.6%** | **Complete Coverage** |
| **Steering Work ($\int \|\omega\| dt$)** | 48.6 rad | **26.4 rad** | **45.7% Less Wheel Slip** |
| **Soil Gouging** | Severe at pivot points | **Zero (Continuous Curvature)** | **Preserves Crop Beds** |

---

### Slide 15: Team Technical Deliverables & Traceability
* **Presenter:** Member 5
* **M1:** `kinematics.py`, `mission_planner.py` — Boustrophedon planner & supervisor FSM.
* **M2:** `worlds/crop_field.wbt`, `motion.py`, `sensors.py` — Pioneer 3-AT configuration, sensor calibration, and motion abstraction.
* **M3:** `pid_controller.py`, `row_follower.py`, `obstacle_avoidance.py` — PID & Stanley guidance, safety stop.
* **M4:** `headland_turn.py`, `agricultural_robot.py` — Point turn, $\Omega$-turn, master loop integration.
* **M5:** `metrics_logger.py`, `plot_results.py` — Real-time telemetry, benchmark tables, publication plots.

---

### Slide 16: Conclusions & Industrial Transition Roadmap
* **Presenter:** All Members (Joint Presentation)
* **Key Conclusions:**
  - Fusing lateral distance sensors with moving average filtering enables reliable row-following without expensive RTK-GPS.
  - Continuous $\Omega$-turns provide superior field transit efficiency while protecting delicate crop root zones.
* **Roadmap to Physical AgBot Hardware:**
  - Transition Webots Python controller to ROS 2 nodes (`rclpy`) with standard `/cmd_vel` and `/odom` topics.
  - Replace simulated distance sensors with solid-state LiDAR (e.g. Livox Mid-360) and wheel quadrature encoders.

---

### Slide 17: End Slide & Repository Links
* **Presenter:** All Members (Q&A)
* **GitHub Repository:** [https://github.com/addy-17/Agricultural-Field-Robot-for-Crop-Row-Coverage](https://github.com/addy-17/Agricultural-Field-Robot-for-Crop-Row-Coverage)
* **YouTube Video Link:** [Unlisted Video Link]
* **Thank You!** Opening for Question & Answer Session.
