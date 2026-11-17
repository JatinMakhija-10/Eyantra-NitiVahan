# Task 2B: Multi-Class Traffic Sign & Obstacle Perception

This module implements a deterministic computer vision perception pipeline capable of detecting and classifying 4 classes of road signs (`Speed40`, `School`, `Parking`, `Hospital`) and static red obstacles (`Obstacle`) from monocular images under severe perspective distortion and partial occlusion.

---

## 1. Classification & Perception Architecture

Rather than relying on resource-intensive deep learning models that suffer from GPU dependencies and non-deterministic latencies, this pipeline employs **computational topology, projective quadrilateral rectification, and normalized spatial probing**.

```
                           Input Image (BGR)
                                  │
                 ┌────────────────┴────────────────┐
                 ▼                                 ▼
         [Red Segmentation]                [Blue Segmentation]
         (H <= 10 | H >= 170)              (100 <= H <= 130)
                 │                                 │
                 ▼                                 ▼
     [Contour Hierarchy Analysis]          [Quadrilateral Detection]
     - Outer contour area                  - ApproxPolyDP (epsilon * peri)
     - Sum of interior hole areas          - Minimum area rect fallback
                 │                                 │
        ┌────────┴────────┐                        ▼
        ▼                 ▼              [Perspective Rectification]
  Hole Area < 15%   Hole Area >= 15%     - Warps quad to 48x48 square patch
        │                 │                        │
        ▼                 ▼                        ▼
    [Obstacle]       [Red Signs]             [Spatial Probing]
  - Solidity >= 0.85 - Circle Fill >= 0.65   - 'P': Empty bottom-right
  - Extent >= 0.65     -> Speed40            - 'H': Dual vertical stems
                     - Circle Fill < 0.65
                       -> School
```

---

## 2. Discrimination Heuristics

| Target Object | Color Rule | Topological Feature | Geometric Metric | Class Logic |
|---|---|---|---|---|
| **Obstacle** | Red | Solid Red (No hole) | Solidity $\ge 0.85$, Extent $\ge 0.65$ | $A_{hole} / A_{outer} < 0.15$ |
| **Speed40** | Red | White Ring Interior | Aspect ratio $\approx 1.0$ (Circle) | $A / (\pi r_{enc}^2) \ge 0.65$ |
| **School** | Red | White Triangular Interior | Aspect ratio $\approx 1.0$ (Triangle) | $A / (\pi r_{enc}^2) < 0.65$ |
| **Parking** | Blue | Letter 'P' Glyph | $48 \times 48$ Rectified Patch | Top-mid probe $\ge 0.60$, Bot-right probe $\le 0.35$ |
| **Hospital** | Blue | Letter 'H' Glyph | $48 \times 48$ Rectified Patch | Bot-right probe $\ge 0.60$, Top-mid probe $\le 0.35$ |

---

## 3. Execution & Testing

### Running the Evaluation Suite:
```sh
conda activate NV_2199
cd task2b
eyantra-autoeval evaluate --year 2026 --theme NV --task 2b
```

### Official Evaluation Report (`result-2026-NV-2b-20261011.json`):
Evaluated across all 11 benchmark test captures in **0.28 seconds**:

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
Score: 40.0 / 40.0 marks (100% Label Match, 0 False Positives, All Areas within 15% tolerance).
```
