# Meeting Logs & Peer Assessment Records
**Course:** CSAI4014 - Algorithms for Intelligent Systems and Robotics  
**Project:** Agricultural Field Robot for Crop-Row Coverage  
**Batch:** B1 - B2 | **Academic Year:** 2026  

---

## 1. Project Meeting Logs

### Meeting 1: Project Kickoff & Task Allocation
* **Date & Time:** 15 October 2026, 10:00 AM – 11:30 AM
* **Mode:** Google Meet (Online)
* **Attendees:** Member 1, Member 2, Member 3, Member 4, Member 5 (All present)
* **Agenda:**
  1. Review of AISR project guidelines and evaluation rubrics.
  2. Election of Team Lead (Member 1 appointed).
  3. Discussion on the Pioneer 3-AT platform in Webots.
  4. Division of 5 modular software components.
* **Decisions Recorded:**
  - Selected Webots as the primary simulation platform.
  - Divided project into 5 modules: M1 (Kinematics & Mission Planning), M2 (Robot Model, Sensors & Motion), M3 (In-Row Guidance & Safety), M4 (Headland Maneuvers & Master Integration), M5 (Telemetry & Benchmarking).
  - Created team GitHub repository: `Agricultural-Field-Robot-for-Crop-Row-Coverage`.

---

### Meeting 2: Sensor Calibration & Kinematics Validation
* **Date & Time:** 19 October 2026, 04:00 PM – 05:30 PM
* **Mode:** In-Person (Robotics Lab)
* **Attendees:** All 5 members present
* **Agenda:**
  1. Review of Member 2's sensor calibration curves and coordinate conventions.
  2. Review of Member 1's Pioneer 3-AT kinematics equations and ICR slip factor.
  3. Interface definition: standardizing $(v, \omega)$ commands across all modules.
* **Decisions Recorded:**
  - Agreed on a $1.8\text{ m}$ crop-row spacing and $16\text{ m}$ row length.
  - Standardized distance sensor lookup table: $0 - 1024 \to 0.05 - 2.0\text{ m}$.
  - Member 2's `motion.py` and `sensors.py` established as the foundational hardware abstraction layer.

---

### Meeting 3: Algorithm Implementation & Headland Strategy Selection
* **Date & Time:** 23 October 2026, 02:00 PM – 04:00 PM
* **Mode:** Google Meet (Online)
* **Attendees:** All 5 members present
* **Agenda:**
  1. Review of Member 3's in-row guidance controllers (Method A PID vs. Method B Stanley).
  2. Review of Member 4's headland turning strategies (In-place Point Turn vs. Continuous $\Omega$-Turn).
  3. Planning comparative experimental benchmarking.
* **Decisions Recorded:**
  - Confirmed the two-method comparative framework:
    - Method A: Baseline PID In-Row + In-Place Pivot Turn.
    - Method B: Advanced Stanley Lookahead + Continuous $\Omega$-Turn.
  - Set obstacle safety threshold to $0.40\text{ m}$ with smooth proportional deceleration.

---

### Meeting 4: Multi-Module Integration & Final Verification
* **Date & Time:** 27 October 2026, 11:00 AM – 01:00 PM
* **Mode:** Hybrid (Lab & Teams)
* **Attendees:** All 5 members present
* **Agenda:**
  1. Full system integration in `agricultural_robot.py`.
  2. Execution of `run_all_tests.py` and `scripts/plot_results.py`.
  3. Rehearsal of the 20-minute video presentation meeting.
  4. Finalizing report sections and slide deck formatting.
* **Decisions Recorded:**
  - Verified 100% pass rate across all unit test suites.
  - Member 4 designated as live demo screen-share presenter.
  - Assigned report formatting and page border verification to all members.

---

## 2. Peer-Assessment Evaluation Sheets

*(Each member evaluated across 4 rubric dimensions on a scale of 1 to 5)*
- **Dimension 1:** Technical Contribution (1–5)
- **Dimension 2:** Reliability & Timeliness (1–5)
- **Dimension 3:** Helped Others Learn (Co-Learning) (1–5)
- **Dimension 4:** Quality of Deliverables (1–5)

### Evaluation Summary Table

| Evaluator | Target Teammate | Technical Contribution (1-5) | Reliability / On Time (1-5) | Helped Others Learn (1-5) | Quality of Work (1-5) | Overall Score (/20) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: |
| **All Peers Avg** | **Member 1 (Lead)** | 5.0 | 5.0 | 4.9 | 5.0 | **19.9 / 20** |
| **All Peers Avg** | **Member 2** | 5.0 | 4.9 | 4.8 | 5.0 | **19.7 / 20** |
| **All Peers Avg** | **Member 3** | 4.9 | 5.0 | 4.9 | 5.0 | **19.8 / 20** |
| **All Peers Avg** | **Member 4** | 5.0 | 4.9 | 5.0 | 5.0 | **19.9 / 20** |
| **All Peers Avg** | **Member 5** | 4.9 | 5.0 | 4.9 | 5.0 | **19.8 / 20** |

---

## 3. Co-Learning Individual Reflections (Section 11)

### Member 1 Reflection
> *"Through this project, I deepened my understanding of non-holonomic mobile robot kinematics and complete coverage path planning. I learned how to model skid-steer instantaneous centers of rotation on outdoor soil. Collaborating with Member 2 taught me how sensor noise manifests in physical simulators, while I helped the team structure the global finite state machine so our modules could execute seamlessly without race conditions."*

### Member 2 Reflection
> *"I learned how to configure mobile robots, physical actuators, and sensory arrays in Webots. Calibrating distance sensor response curves and implementing moving average filtering showed me how crucial sensor conditioning is before passing signals to feedback controllers. Member 3 helped me understand how cross-track error influences controller stability, and I shared with the team how to decouple motor actuation using a clean motion interface."*

### Member 3 Reflection
> *"Designing both PID and Stanley steering controllers gave me hands-on insight into closed-loop stability, gain tuning, and derivative kick suppression. Working with Member 4 showed me how transitions between in-row guidance and headland turning require clean reset of integral accumulators. I enjoyed teaching my peers how Stanley lookahead angles inherently compensate for heading misalignment."*

### Member 4 Reflection
> *"Developing continuous curvature $\Omega$-turn trajectories gave me deep appreciation for geometric path planning and soil conservation mechanics. Rehearsing the live Webots simulation demo with Member 5 helped us ensure all telemetry logging hooks operated synchronously at 32 ms timesteps. I assisted Member 1 in defining clear headland boundary entry/exit triggers."*

### Member 5 Reflection
> *"Logging real-time simulation states and analyzing quantitative performance metrics was immensely rewarding. Comparing Method A and Method B mathematically proved how continuous curvature paths save mission time and reduce wheel slip. I learned about skid-steer power dissipation from Member 1 and helped all team members format their technical figures and tables for our project report."*
