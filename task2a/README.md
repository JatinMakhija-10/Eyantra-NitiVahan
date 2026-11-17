# Task 2A: Autonomous Arena Circuit Navigation & Stop Control

This module implements closed-loop vision-based autonomous navigation of the concentric arena loop in **CoppeliaSim**, featuring metric **Inverse Perspective Mapping (IPM)**, multi-band clustering, curvature-compensated **Pure Pursuit** steering, and autonomous stop discipline.

---

## 1. System Pipeline

In Task 2A, the vehicle is driven purely from its forward-facing monocular camera (`visionSensor`), requiring tight synchronization between perception and actuation.

```
       Front Camera Frame (640x480 BGR)
                     │
                     ▼
       [Analytical Pinhole Model & IPM]
       - Tilt: 30.54 deg, HFOV: 95 deg, Horizon: y=67
       - Warps ground ahead to 480x480 Metric BEV
                     │
                     ▼
       [10-Band Transverse Clustering]
       - Vertical slices extract boundary centroids
       - Dynamic lane-width dilation: W * sqrt(1 + (dx/ds)^2)
       - Historical voting (_state["vote"]) prevents lane swap on bends
                     │
                     ▼
       [Pure Pursuit with Rear Cut-In Offset]
       - Lookahead target at L_d = 0.32 m ahead
       - Outer bias offset B_out = 0.035 m protects rear wheels on tight bends
       - Combined Pure Pursuit + Scale-free cross-track error K_e
                     │
                     ▼
       [Speed Modulation & Stop Logic]
       - Dynamic throttle: 3.0 rad/s on straights -> 1.5 rad/s on hairpins
       - Level-triggered NV_stop signal halts vehicle with 0.0 rad/s
                     │
                     ▼
       [Ackermann Split Actuation]
       - Left & right angles dispatched to /steeringLeft, /steeringRight
```

---

## 2. Inner vs. Outer Circuit Dynamics

The evaluation tests the vehicle in two distinct scenarios:
1. **Outer Lane ($40\%$ of marks)**: Larger turning radius ($R \approx 0.6\text{ m}$), wide field of view.
2. **Inner Lane ($60\%$ of marks)**: Tighter radius ($R \approx 0.4\text{ m}$), narrow camera horizon.

### The Rear-Wheel Cut-In Solution:
On the tight inner lane, Ackermann off-tracking causes rear wheels to cut inwards by $\Delta r \approx L^2 / (2R)$. To prevent the rear tires from touching the inside yellow boundary line, an outward offset is injected:

$$y_L^* = y_L - \text{sgn}(\kappa) \cdot B_{out} \cdot \min\left(1.0, \frac{|\kappa|}{2.0}\right)$$

where $B_{out} = 0.035\text{ m}$.

---

## 3. Execution & Testing

### Running the Navigation Loop:
1. Launch **CoppeliaSim Edu 4.10** and open `Task_2A.ttt`.
2. Run `task2a.py` (it waits until CoppeliaSim is started):
```sh
conda activate NV_2199
cd task2a
python task2a.py
```
3. Press **▶ Play** in CoppeliaSim.

### Development Tools (`dev_tools.py`):
```sh
python dev_tools.py grab 15       # Save every 15th camera frame to frames/
python dev_tools.py bev frame.png # Generate top-down bird's-eye view verification
python dev_tools.py test frames/  # Run lane detection offline on saved frames
python dev_tools.py stop 5        # Test NV_stop assertion for 5 seconds
```

### Full Automated Evaluation:
```sh
eyantra-autoeval evaluate --year 2026 --theme NV --task 2a
```
This runs both outer and inner laps, verifies all stop points, and writes the signed recording `NV_Task2A_run.csv` and result JSON.

---

## 4. Evaluation Results (`result-2026-NV-2a-20261012.json`)

| Evaluation Circuit | Samples Recorded | Sim Time | Stop Points | Lap Status |
|---|:---:|:---:|:---:|:---:|
| **Outer Lane Lap** | 1,154 samples | $148.95\text{ s}$ | 2 / 2 held ($5\text{ s}$) | **Lap Complete** |
| **Inner Lane Lap** | 1,103 samples | $132.35\text{ s}$ | 2 / 2 held ($5\text{ s}$) | **Lap Complete** |
| **Total Marks** | **40.0 / 40.0** | | | **100% Full Score** |
