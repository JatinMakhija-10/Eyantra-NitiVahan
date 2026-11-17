# Task 2C: High-Precision ArUco Bay Parking & Visual Servoing

This module implements autonomous precision docking into narrow parking bays ($0.22\text{ m}$ bay width for an $0.18\text{ m}$ vehicle chassis) guided by an **ArUco fiducial marker** (DICT_4X4_50) and a $(y, \psi)$ state-space trajectory planner.

---

## 1. System Pipeline

```
           Front Camera Frame (640x480 BGR)
                          │
                          ▼
             [ArUco Fiducial Detection]
             - Dictionary: cv2.aruco.DICT_4X4_50
             - Pattern Side: 0.096 m
                          │
                          ▼
            [SolvePnP IPPE Ambiguity Filter]
            - cv2.solvePnPGeneric (SOLVEPNP_IPPE_SQUARE)
            - Trust heading only if err_2 / err_1 >= 1.50
            - Gated complementary blend with bicycle odometry
                          │
                          ▼
          [Optimal Crossing Angle Optimization]
          - Evaluates remaining forward room (forward - CROSS_END)
          - Computes theta_c in [17 deg, 42 deg] via line-arc geometry
                          │
                          ▼
            [Tangential Arc Path Execution]
            - Blends crossing into circular arc (R = 0.14 m)
            - Feedforward curvature: kappa_ff = sgn(y) / R_arc
            - Proportional heading correction: (theta_wanted - psi) / D_head
                          │
                          ▼
             [Multi-Phase Velocity Throttling]
             - > 0.90 m: Cruise (2.4 rad/s)
             - 0.50 - 0.90 m: Mid (1.2 rad/s)
             - 0.13 - 0.50 m: Creep (1.2 rad/s)
             - <= 0.130 m: Stop & wait 12 settling frames -> parked=True
```

---

## 2. Bay Geometry & Tolerances

```
                  Bay 0 (Near Bay)                      Bay 1 (Far Bay)
            ◄────── 0.22 m width ──────►          ◄────── 0.22 m width ──────►
           ┌────────────────────────────┐        ┌────────────────────────────┐
           │   [Marker 0]               │        │   [Marker 1]               │
           │                            │        │                            │
           │      ┌──────────────┐      │        │      ┌──────────────┐      │
           │ 20mm │ Vehicle      │ 20mm │        │ 20mm │ Vehicle      │ 20mm │
           │      │ 0.18 m wide  │      │        │      │ 0.18 m wide  │      │
           │      └──────────────┘      │        │      └──────────────┘      │
           │                            │        │                            │
           └────────────────────────────┘        └────────────────────────────┘
           Lateral Crossing: 0.22 m              Lateral Crossing: 0.44 m
           Initial Heading: 0.0 rad (Parallel)   Initial Heading: 0.0 rad (Parallel)
```

With only $\approx 20\text{ mm}$ of clearance on either side, arriving at an angle of just $5^\circ$ shifts the vehicle corners by $14\text{ mm}$, putting the wheels over the line. The trajectory planner guarantees that the vehicle arrives **on the centerline and collinear with the bay axis** before crossing the bay mouth ($X = 0.37\text{ m}$).

---

## 3. Execution & Testing

### Running Single Bay Parking:
1. Open `Task_2C.ttt` in **CoppeliaSim Edu 4.10**. Leave simulation stopped.
2. Run for Bay 0 or Bay 1:
```sh
conda activate NV_2199
cd task2c
python task2c.py --bay 0    # Park in near bay
python task2c.py --bay 1    # Park in far bay
```
3. Press **▶ Play** in CoppeliaSim.

### Full Automated Evaluation:
```sh
eyantra-autoeval evaluate --year 2026 --theme NV --task 2c
```
This tests both bays consecutively, writes `NV_Task2C_bay0.csv` and `NV_Task2C_bay1.csv`, and signs the evaluation.

---

## 4. Evaluation Results (`NV_Task2C_bay0.csv` & `NV_Task2C_bay1.csv`)

| Bay | Lateral Crossing | Centerline Offset | Heading Error | Boundary Clearance | Score | Weight |
|---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Bay 0** | $0.22\text{ m}$ | **$2.8\text{ mm}$** | **$0.5^\circ$** | $> 15.0\text{ mm}$ | **20.00 / 20.00** | 70% |
| **Bay 1** | $0.44\text{ m}$ | **$12.7\text{ mm}$** | **$2.0^\circ$** | $> 7.4\text{ mm}$ | **18.75 / 20.00** | 30% |
| **Total** | | | | | **19.62 / 20.00** | **98.1%** |
