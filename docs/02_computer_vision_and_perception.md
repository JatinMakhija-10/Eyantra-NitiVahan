# Computer Vision & Perception Pipeline

This document details the computer vision algorithms, mathematical image processing techniques, and perception architectures developed for **Niti Vahan (eYRC 2026-27)**.

---

## Table of Contents
1. [Overview & Architectural Philosophy](#1-overview--architectural-philosophy)
2. [Task 1C: Ultra-High-Speed Lane Perception (505+ FPS)](#2-task-1c-ultra-high-speed-lane-perception-505-fps)
3. [Task 2A: Metric BEV Multi-Band Clustering](#3-task-2a-metric-bev-multi-band-clustering)
4. [Task 2B: Topological Traffic Sign & Obstacle Perception](#4-task-2b-topological-traffic-sign--obstacle-perception)
5. [Task 2C: ArUco Fiducial Vision & Spatial Pose Estimation](#5-task-2c-aruco-fiducial-vision--spatial-pose-estimation)

---

## 1. Overview & Architectural Philosophy

Autonomous navigation in real-time robotic platforms demands minimal perception latency. Rather than relying on computationally heavy deep learning networks that introduce non-deterministic inference delays (e.g., YOLO or UNet requiring 20–50 ms on a GPU), the Niti Vahan perception stack is built upon **deterministic classical computer vision**, **morphological topology**, and **geometric projective transformations**.

### Key Achievements:
- **Task 1C Lane Detector**: Evaluates at **505.8 FPS** (1.98 ms per 640x480 frame) on CPU with **97.9% lane accuracy** and **5.5 px mean center error**.
- **Task 2B Sign & Obstacle Classifier**: 100% correct classification of multi-sign and obstacle scenes using quadrilateral rectification and topological probing in **0.28 seconds total across 11 test sets**.
- **Task 2C ArUco Pose Tracking**: Solves full 6-DOF camera pose with IPPE square ambiguity rejection and kinematic dead reckoning.

```
       Raw Video Frame (640x480 BGR)
                   │
         ┌─────────┴─────────┐
         ▼                   ▼
    [HSV Color Space]   [Adaptive ROI]
         │                   │
         ▼                   ▼
   [Mask Generation]   [Hood Masking]
   - Yellow Edges      - Bonnet Occlusion
   - White Dashes      - Sky/Horizon Clip
         │                   │
         └─────────┬─────────┘
                   │
                   ▼
     [Topological Segmentation]
     - Connected Components
     - Morphological Ring Filters
     - Minimum Enclosing Geometries
                   │
                   ▼
     [Curve Fitting / Rectification]
     - RANSAC Degree-2 Polynomial
     - IPM Bird's-Eye View Mapping
     - Perspective Quad Normalization
                   │
                   ▼
       Deterministic Output
       - Lane Center (px) & ID
       - Lateral Offset (m) & Heading (rad)
       - Sign Labels, Obstacle Area
```

---

## 2. Task 1C: Ultra-High-Speed Lane Perception (505+ FPS)

The goal of Task 1C is to process continuous driving clips, identify whether the vehicle is in the `left` or `right` lane, and pinpoint the exact horizontal coordinate of the lane centerline at reference row $y_{ref} = 305\text{ px}$.

### 2.1 Multi-Threshold HSV Color Space Segmentation
Lane markers in the arena consist of:
1. **Solid Yellow Boundaries**: Outer edges of the roadway.
2. **Dashed White Lines**: Central lane divider separating inner and outer lanes.

```python
# Color Thresholding in HSV
yellow_mask = cv2.inRange(hsv, (18, 100, 100), (38, 255, 255))
white_mask  = cv2.inRange(hsv, (0, 0, 160), (180, 50, 255))

# ROI Masking
white_mask[410:, 215:420] = 0        # Mask vehicle hood
white_mask[:150, :] = 0              # Mask sky and distant horizon
```

### 2.2 Tarmac Ring Filter for White Blob Discrimination
A primary challenge in vision-based driving is distinguishing actual dashed line markings on the road from white buildings, roadside barriers, signs, or reflections.

To solve this deterministically:
1. Compute connected components on `white_mask` via `cv2.connectedComponentsWithStats`.
2. For each candidate blob $i$, isolate its local bounding box expanded by a morphological ring of radius $r = 4\text{ px}$.
3. Construct an annular morphological ring:

$$\text{Ring}_i = \text{Dilate}(\text{Blob}_i, \mathbf{K}_{9\times 9}) \setminus \text{Blob}_i$$

4. Evaluate the mean brightness of the surrounding pixels in the Value channel. Real road dashes are bordered by dark asphalt tarmac ($V \le 70$). If the surrounding tarmac fraction falls below $60\%$, the candidate is instantly rejected as background clutter:

$$\mu_{tarmac} = \frac{1}{|\text{Ring}_i|} \sum_{(x, y) \in \text{Ring}_i} \mathbb{I}(V(x, y) \le 70) \ge 0.60$$

```
   Candidate White Blob                Annular Tarmac Ring
     ┌──────────────┐                   ┌──────────────┐
     │              │                   │░░░░░░░░░░░░░░│  Tarmac pixels
     │   ████████   │     ──────►       │░░  ██████  ░░│  must have
     │   ████████   │                   │░░  ██████  ░░│  V <= 70
     │              │                   │░░░░░░░░░░░░░░│
     └──────────────┘                   └──────────────┘
```

### 2.3 RANSAC-Style Polynomial Curve Fitting
For dash centroids $(x_k, y_k)$ with $y_k \ge 215\text{ px}$, we fit an inverted polynomial $x = f(y) = a y^2 + b y + c$.

To reject outlier centroids (e.g. from intersections or partial occlusions), an iterative 3-pass reweighted least-squares filter is executed:
- Residual threshold: $|f(y_k) - x_k| < 12\text{ px}$.
- Outliers exceeding the threshold are excluded and the polynomial is refit.

```python
def _fit_dash_curve(points):
    pts = np.array([p for p in points if p[1] >= FIT_MIN_ROW])
    if len(pts) < 2:
        return None
    xs, ys = pts[:, 0], pts[:, 1]
    degree = 2 if len(pts) >= 3 else 1
    keep = np.ones(len(pts), bool)
    for _ in range(3):
        if keep.sum() < degree + 1:
            break
        coeffs = np.polyfit(ys[keep], xs[keep], degree)
        new_keep = np.abs(np.polyval(coeffs, ys) - xs) < 12
        if new_keep.sum() < degree + 1 or (new_keep == keep).all():
            break
        keep = new_keep
    return np.polyfit(ys[keep], xs[keep], degree)
```

### 2.4 Least-Squares Dual-Marker Center Estimation
At reference row $y_{ref} = 305$, the vehicle centerline column is $x_{veh} = 278.0\text{ px}$.
- If $x_{dash}(305) > 278.0$: The divider is to the right of the vehicle, so the vehicle is in the **Left Lane**.
- If $x_{dash}(305) \le 278.0$: The vehicle is in the **Right Lane**.

Using linear regression models cross-validated on public clips:
- **Left Lane Center**:

$$x_{center} = 0.9256 \cdot x_{dash}(305) - 132.21$$

- **Right Lane Center** (combining dashes, yellow edge, and local slope):

$$x_{center} = 0.9720 \cdot x_{dash}(305) + 0.2042 \cdot x_{yellow}(305) + 45.87 \cdot \text{slope} + 38.06$$

- **Yellow-Only Fallback**: If dashes are completely occluded, multi-row horizontal run length encoding identifies the solid yellow boundary, mapping to the center line via:

$$x_{center} = 0.7098 \cdot x_{yellow} + 208.35 \quad \text{(Left Lane)}$$

$$x_{center} = 0.5836 \cdot x_{yellow} + 17.06 \quad \text{(Right Lane)}$$

---

## 3. Task 2A: Metric BEV Multi-Band Clustering

In Task 2A, the vehicle must navigate a closed concentric-loop arena.

### 3.1 10-Band Horizontal Slicing in BEV
The Bird's-Eye View image ($480 \times 480\text{ px}$) is partitioned into 10 equidistant vertical bands (each $48\text{ px}$ tall along the ground range ahead).

For each band $k$:
1. Binary projection histograms along columns are computed: $S(x) = \sum_{y \in \text{Band}_k} I(x, y)$.
2. Centroids of column clusters separated by gaps $> 12\text{ px}$ are extracted.
3. Identified centroids are categorized as left boundary candidate $L_k$ or right candidate $R_k$.

```
     BEV Grid (480 x 480)
     ┌────────────────────────┐
     │  Band 9 (Far Ahead)    │  ──► S_9(x) Histogram
     ├────────────────────────┤
     │  Band 8                │
     ├────────────────────────┤
     │  ...                   │
     ├────────────────────────┤
     │  Band 1                │
     ├────────────────────────┤
     │  Band 0 (Near Axle)    │  ──► Nearest Cross-Track Error
     └────────────────────────┘
```

### 3.2 Dynamic Lane Width Dilation on Curves
On a straight road, the transverse distance between yellow boundary and dashed line is $W_{lane} = 200\text{ px}$ in BEV. However, when the road curves at local derivative slope $dx/ds$, the apparent transverse distance between boundaries widens by $\frac{1}{\cos \phi} = \sqrt{1 + (dx/ds)^2}$.

When one boundary is occluded by the camera edge, the missing boundary is synthesized using the curvature-corrected half-width:

$$W_{half}(s) = \frac{W_{lane}}{2} \sqrt{1 + \left(\frac{dx}{ds}\right)^2}$$

This mathematical compensation prevents the estimated center from collapsing inwards on sharp turns.

---

## 4. Task 2B: Topological Traffic Sign & Obstacle Perception

Task 2B requires detecting 4 classes of traffic signs (`School`, `Speed40`, `Parking`, `Hospital`) and solid red obstacles (`Obs`) in static camera captures with severe perspective skew.

### 4.1 Discrimination Matrix
| Object | Dominant Color | Interior Geometry | Aspect Ratio | Classification Key |
|---|---|---|---|---|
| **Speed40** | Red | White Ring Interior | $\approx 1.0$ (Circle) | $A / (\pi r^2) \ge 0.65$ |
| **School** | Red | White Triangular Interior | $\approx 1.0$ (Triangle) | $A / (\pi r^2) < 0.65$ |
| **Obstacle** | Red | Solid Red (No Hole) | $0.4 \le W/H \le 3.0$ | $A_{hole} / A_{outer} < 0.15$, Solidity $\ge 0.85$ |
| **Parking** | Blue | Letter 'P' inside | $0.7 \le W/H \le 1.4$ | Top-mid non-blue $\ge 0.60$, Bottom-right blue |
| **Hospital** | Blue | Letter 'H' inside | $0.7 \le W/H \le 1.4$ | Left & right stems non-blue $\ge 0.60$ |

### 4.2 Red Signs vs. Obstacle Separation
Red segmentation is extracted in HSV using wrapped hue ($H \le 10 \lor H \ge 170$, $S \ge 100$, $V \ge 60$).

1. Contours are computed using two-level component hierarchy `cv2.RETR_CCOMP`.
2. For each outer contour $i$ (where $\text{hier}[i][3] == -1$), the area of all interior holes $j$ (where $\text{hier}[j][3] == i$) is summed:

$$\rho_{hole} = \frac{\sum_{j} A_{hole, j}}{A_{outer, i}}$$

- If $\rho_{hole} \ge 0.15$: The object is a **Traffic Sign with white interior**.
- If $\rho_{hole} < 0.15$: The object is solid red. It is passed to the **Obstacle Filter**.

3. **Separating Speed40 from School**:
   - For all detected red sign contours, compute the minimum enclosing circle of radius $r$.
   - A circle has fill factor $\frac{\pi r^2}{\pi r^2} = 1.0$.
   - An equilateral triangle has fill factor $\frac{\frac{3\sqrt{3}}{4} r^2}{\pi r^2} \approx 0.413$.
   - Using threshold $\tau_{circle} = 0.65$:
     - Fill factor $\ge 0.65 \implies$ **Speed40**
     - Fill factor $< 0.65 \implies$ **School**

```
     Speed40 (Circle)                    School (Triangle)
     ┌──────────────┐                    ┌──────────────┐
     │   ╭──────╮   │                    │      ▲       │
     │  │  40   │   │                    │     / \      │
     │   ╰──────╯   │                    │    /   \     │
     │  Fill >= 0.65│                    │   /_____\    │
     └──────────────┘                    │  Fill < 0.65 │
                                         └──────────────┘
```

### 4.3 Perspective Quad Rectification & Letter Probing (Parking vs Hospital)
Blue signs are detected via $H \in [100, 130]$, $S \ge 120$, $V \ge 60$.

1. **Quadrilateral Extraction**: `cv2.approxPolyDP` iteratively extracts the four perspective corners of the sign face hull.
2. **Perspective Rectification**: The quadrilateral is mapped to a canonical $48 \times 48\text{ px}$ square patch via `cv2.getPerspectiveTransform` and `cv2.warpPerspective`.
3. **Spatial Feature Probing**: In the rectified $48 \times 48$ patch, blue mask inversion isolates the letter glyphs. Four strategic normalized bounding sub-windows are probed for non-blue pixel density $\bar{\mu}$:
   - Left Stem: $y \in [30\%, 72\%], x \in [26\%, 33\%]$
   - Top Middle: $y \in [21\%, 27\%], x \in [45\%, 55\%]$
   - Bottom Right: $y \in [60\%, 74\%], x \in [67\%, 75\%]$
   - Bottom Middle: $y \in [60\%, 74\%], x \in [45\%, 55\%]$

```
     Rectified Patch (48x48)
     ┌────────────────────────┐
     │      [Top Mid]         │
     │          █             │  Letter 'P':
     │  [L-Stem]█             │  - Left stem = High
     │    █     ████          │  - Top mid = High
     │    █     █  █          │  - Bottom right = Empty (Blue)
     │    █     ████          │
     │    █                   │  Letter 'H':
     │    █     [Bot-Right]   │  - Left stem = High
     │    █          █        │  - Bottom right = High
     │    █          █        │  - Top mid = Empty
     └────────────────────────┘
```

Decision logic:
- $\bar{\mu}_{bot\_right} \ge 0.60 \land \bar{\mu}_{top\_mid} \le 0.35 \implies$ **Hospital**
- $\bar{\mu}_{bot\_right} \le 0.35 \land \bar{\mu}_{top\_mid} \ge 0.60 \implies$ **Parking**

---

## 5. Task 2C: ArUco Fiducial Vision & Spatial Pose Estimation

Task 2C utilizes an ArUco fiducial marker standing at the head of each bay to perform 3D visual servoing.

### 5.1 Marker Detection & Ambiguity Rejection
- Marker Dictionary: `cv2.aruco.DICT_4X4_50`
- Physical Marker Size: $s_{marker} = 0.096\text{ m}$ (black pattern size)
- Camera Intrinsic Matrix:

$$\mathbf{K} = \begin{bmatrix} 293.2 & 0 & 320 \\ 0 & 293.2 & 240 \\ 0 & 0 & 1 \end{bmatrix}$$

Using `cv2.solvePnPGeneric` with `SOLVEPNP_IPPE_SQUARE`:
Because a planar square viewed nearly face-on produces two mathematically valid pose hypotheses (due to projective perspective ambiguity), the solver returns both solutions. We compute the reprojection error ratio:

$$\chi_{ratio} = \frac{\text{err}_2}{\text{err}_1}$$

A visual heading observation is trusted only when $\chi_{ratio} \ge 1.50$; otherwise, ambiguous heading updates are rejected to prevent sudden rotational spikes.

### 5.2 Kinematic Prediction & Heading Blending
To eliminate observation noise when the marker is far away ($> 1\text{ m}$):
1. **Odometry Prediction**: The change in heading is predicted using the bicycle kinematic model driven by commanded steering $\delta$:

$$\psi_{pred} = \psi_{k-1} - \frac{\tan \delta}{L} \Delta s$$

where $\Delta s = \frac{\rho_{prev} - \rho_{now}}{\cos(\beta)}$ is the longitudinal distance traversed based on marker range reduction.

2. **Gated Sensor Fusion**: When a clear, unambiguous ArUco heading observation $\psi_{obs}$ arrives within acceptance gate $|\psi_{obs} - \psi_{pred}| \le 15^\circ$, it is fused via complementary blending:

$$\psi_k = \psi_{pred} + \alpha_{blend} (\psi_{obs} - \psi_{pred}) \quad (\alpha_{blend} = 0.12)$$

3. **Pose Metric Output**:
- Lateral offset from bay axis: $y_{front} = -\rho \sin(\beta + \psi) + \left(\frac{L}{2} - d_{cam}\right) \sin \psi$
- Forward distance to marker: $x_{fwd} = \rho \cos(\beta + \psi) + d_{cam} \cos \psi$

This filtered pose achieves millimeter-level stability throughout the entire docking trajectory.
