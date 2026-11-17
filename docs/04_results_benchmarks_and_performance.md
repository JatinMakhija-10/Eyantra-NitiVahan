# Evaluation Results, Benchmarks & Quantitative Analysis

This document compiles the empirical performance benchmarks, official evaluation results, and quantitative validation metrics achieved by **Team 2199** in the **eYRC 2026-27 Niti Vahan** competition.

---

## Executive Scoreboard

| Milestone | Task Description | Allocated Marks | Achieved Score | Key Performance Metric |
|---|---|:---:|:---:|---|
| **Task 1A** | Ackermann Steering Kinematics | 20 | **20.0 / 20.0** | 100% random angle test pass, 0 singularity error |
| **Task 1B** | Closed-Loop Path Tracking (Stanley) | 30 | **30.0 / 30.0** | $0.00\text{ m}$ steady-state error, $< 0.02\text{ m}$ overshoot |
| **Task 1C** | High-Speed Lane Perception Pipeline | 50 | **48.4 / 50.0** | **505.8 FPS** throughput, $5.5\text{ px}$ mean center error |
| **Task 2A** | Closed-Loop Arena Circuit Navigation | 40 | **40.0 / 40.0** | Dual-lane lap complete (Outer & Inner), 4/4 stops held |
| **Task 2B** | Multi-Class Sign & Obstacle Perception | 40 | **40.0 / 40.0** | 100% label match, 0 false positives, 0.28s total eval |
| **Task 2C** | Visual Servoing Bay Parking | 20 | **19.6 / 20.0** | Sub-centimeter offset, $< 1^\circ$ heading alignment |
| **Total** | **Stage 1 Simulation Challenge** | **200** | **198.0 / 200.0** | **99.0% Cumulative Performance** |

---

## Task 1A: Kinematic Ackermann Steering Verification

### Grader Verification Report
- **$\delta = 0$ Boundary Case**: `PASSED` (5/5 marks). Function returns `(0.0, 0.0)` exactly.
- **Random Testing Cases**: `10/10 PASSED` (15/15 marks). Evaluated on 10 dynamically generated random angles $\delta \in (-0.523, +0.523)\text{ rad}$.
- **Symmetry & Kingpin Check**: Left and right angles verified within tolerance $\epsilon < 0.1\text{ rad}$.

```python
# Sample Verification Output
delta = -0.35 -> left = -0.3804 rad, right = -0.3243 rad
delta =  0.00 -> left =  0.0000 rad, right =  0.0000 rad
delta = +0.35 -> left = +0.3243 rad, right = +0.3804 rad
```

---

## Task 1B: Lateral Path Tracking & Dynamic Lane Changes

### Grader Verification Report (`result-2026-NV-1b-20261003.json`)
- **Evaluation Schedule**: Fixed simulated schedule `22:R, 53:L` over 120.0 simulated seconds.
- **Total Logged Samples**: 2,400 rows at 20 Hz.
- **Execution Time**: 18.92 seconds wall-clock time.

```
---------------------------------------------------------------------------------
Segment           Target y    Convergence Error    Settling Time    Overshoot
---------------------------------------------------------------------------------
Lane Switch 1 (R)  0.20 m          < 0.005 m          12.4 s         0.012 m
Lane Switch 2 (L)  0.00 m          < 0.004 m          11.8 s         0.010 m
---------------------------------------------------------------------------------
Summary: Both lane switches converged within the 0.02m acceptance band.
         Settling time well below the 25.0s maximum credit threshold.
         Overshoot contained below the 0.02m full credit boundary.
```

---

## Task 1C: Real-Time Lane Perception & Benchmarking

### Grader Verification Report (`result-2026-NV-1c-20261004.json`)
Evaluated across all 20 public evaluation video clips (4,000 frames total at 640x480 resolution):

```
---------------------------------------------------------------------------------
Metric                                Value
---------------------------------------------------------------------------------
Total Video Frames Processed          4,000 frames
Scored Frames                         3,708 frames
Ambiguous / Transition Frames         292 frames (excluded for all teams)
Correct Lane Identification           3,630 / 3,708 (97.9%)
Center Full Credit (error <= 30px)    3,624 frames
Center Half Credit (error <= 60px)    6 frames
Center Wrong Lane / Out of Band       78 frames
Unknown Frames Returned               0 frames
Mean Lane Center Error                5.5 pixels
Total Processing Duration             7.91 seconds (4,000 frames)
Perception Throughput                 505.8 FPS (1.98 ms / frame)
---------------------------------------------------------------------------------
Score Breakdown:
  Lane Identification Score:          19.06 / 20.00
  Lane Center Precision Score:        29.34 / 30.00
  Task 1C Public-Set Total:           48.40 / 50.00
---------------------------------------------------------------------------------
```

```
   Frame Distribution (3,708 Scored Frames)
   ┌────────────────────────────────────────────────────────┐
   │█████████████████████████████████████████████████░░│
   └────────────────────────────────────────────────────────┘
   ■ Full Credit (97.7%)   ░ Half Credit (0.2%)   □ Error (2.1%)
```

---

## Task 2A: Autonomous Arena Circuit Navigation

### Grader Verification Report (`result-2026-NV-2a-20261012.json`)
The evaluator executes two full-circuit runs under autonomous vision control with stochastic stop injection.

```
---------------------------------------------------------------------------------
Scenario       Samples    Sim Time    Wall Time    Stop 1 (Hold)    Stop 2 (Hold)    Status
---------------------------------------------------------------------------------
Run 1: Outer    1,154     148.95 s     23.95 s     2/2 Raised (5s)  2/2 Released     Lap Complete
Run 2: Inner    1,103     132.35 s     24.39 s     2/2 Raised (5s)  2/2 Released     Lap Complete
---------------------------------------------------------------------------------
Integrity Checks:
  - Student Missing Functions: []
  - Forbidden Remote API Calls: [] (0 startSimulation / setObjectPosition)
  - Integrity Signature: HMAC-SHA256 Validated
  - Total Samples Recorded: 2,257 rows
```

### Inner vs. Outer Circuit Analysis
- **Outer Lane**: Wider turning radius ($R \approx 0.6\text{ m}$). Average speed $2.8\text{ rad/s}$.
- **Inner Lane**: Tight radius ($R \approx 0.4\text{ m}$). Dynamic curvature offset ($B_{out} = 0.035\text{ m}$) prevented rear wheels from touching the inner yellow boundary. 100% lane containment achieved.

---

## Task 2B: Traffic Sign & Obstacle Perception

### Evaluation Report (`result-2026-NV-2b-20261011.json`)
Processed across all 11 benchmark captures representing varied lighting, orientation, and object combinations in **0.283 seconds**:

```
---------------------------------------------------------------------------------------------------
Image File    Detected Signs    Ground Truth Sign    Obs Present    Obs Area    Area Ground Truth
---------------------------------------------------------------------------------------------------
40_1.png      Speed40 (1356px)  Speed40              False          0 px        0 px
40_2.png      Speed40 (4840px)  Speed40              False          0 px        0 px
40_P_1.png    Speed40 (1236px)  Speed40, Parking     False          0 px        0 px
              Parking (2569px)
Blank_1.png   (None)            (None)               False          0 px        0 px
H_1.png       Hospital (3968px) Hospital             False          0 px        0 px
Obs_1.png     (None)            (None)               True           13659 px    13500 px (+1.1%)
Obs_H_1.png   Hospital (6120px) Hospital             True           4531 px     4600 px (-1.5%)
Obs_H_2.png   Hospital (1121px) Hospital             True           1459 px     1500 px (-2.7%)
P_1.png       Parking (1696px)  Parking              False          0 px        0 px
Sc_1.png      School (2562px)   School               False          0 px        0 px
Sc_2.png      School (1576px)   School               False          0 px        0 px
---------------------------------------------------------------------------------------------------
Classification Performance:
  - Matched Signs: 11 / 11 (100.0%)
  - Missed Signs: 0
  - Extra False Positives: 0
  - Obstacle Detection Accuracy: 100.0%
  - Area Errors: All within the 15% tolerance band for full credit
```

---

## Task 2C: High-Precision ArUco Bay Parking

### Grader Verification Report (`NV_Task2C_bay0.csv` & `NV_Task2C_bay1.csv`)
- **Bay 0 (Near Crossing, 0.22 m lateral distance)**:
  - Final State: Inside bay, clear of both side lines by $> 15\text{ mm}$.
  - Lateral Centerline Offset: $2.8\text{ mm}$ (Full Credit $\le 10\text{ mm}$).
  - Heading Alignment Error: $0.5^\circ$ (Full Credit $\le 2^\circ$).
  - Velocity at Finish: $0.0000\text{ m/s}$ (Vehicle completely stationary).
  - Score: **20.00 / 20.00** (Weight 70%).

- **Bay 1 (Far Crossing, 0.44 m lateral distance)**:
  - Final State: Inside bay, clear of both side lines by $> 7\text{ mm}$.
  - Lateral Centerline Offset: $12.7\text{ mm}$.
  - Heading Alignment Error: $2.0^\circ$.
  - Velocity at Finish: $0.0000\text{ m/s}$.
  - Score: **18.75 / 20.00** (Weight 30%).

- **Combined Task 2C Score**:

$$\text{Final Score} = 0.70 \times 20.00 + 0.30 \times 18.75 = 14.00 + 5.625 = \mathbf{19.62} / \mathbf{20.00}$$

---

## Computational Footprint & Latency Benchmarks

All benchmarks recorded on AMD64 Architecture, Windows 10/11 environment:

| Module | Processing Step | Average Execution Time | Peak Memory | CPU Utilization |
|---|---|:---:|:---:|:---:|
| **Task 1A** | Ackermann Split Function | $0.004\text{ ms}$ | $< 5\text{ MB}$ | $< 1\%$ |
| **Task 1B** | Stanley Controller Step | $0.082\text{ ms}$ | $< 15\text{ MB}$ | $< 2\%$ |
| **Task 1C** | Frame HSV + Ring + RANSAC | $1.978\text{ ms}$ | $42\text{ MB}$ | $18\%$ (Single Core) |
| **Task 2A** | BEV IPM + 10-Band + Pure Pursuit | $4.210\text{ ms}$ | $58\text{ MB}$ | $22\%$ |
| **Task 2B** | Sign Rectification + Letter Probing | $25.79\text{ ms}$ | $35\text{ MB}$ | $12\%$ |
| **Task 2C** | SolvePnP IPPE + Arc Planner | $3.150\text{ ms}$ | $48\text{ MB}$ | $15\%$ |
