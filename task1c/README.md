# Task 1C: Ultra-High-Speed Lane Perception Pipeline

This module implements an ultra-low-latency computer vision pipeline that processes continuous monocular video streams (640x480 @ 20 FPS), identifies lane classification (`left` or `right`), and estimates the exact horizontal pixel coordinate of the lane centerline at reference row $y_{ref} = 305$.

---

## 1. Computer Vision Architecture

The perception pipeline executes at **505.8 FPS** (1.98 ms per frame on CPU) by relying on deterministic morphological operations and linear model regressions:

```
  Input Frame (640x480 BGR)
             │
             ▼
  [HSV Thresholding]
  - Yellow boundary: (18, 100, 100) -> (38, 255, 255)
  - White dashes:    (0, 0, 160)     -> (180, 50, 255)
             │
             ▼
  [Bonnet & Horizon Masking]
  - Bonnet: rows [410:], cols [215:420]
  - Sky/Horizon: rows [:150]
             │
             ▼
  [Tarmac Ring Morphological Filter]
  - Annular dilation ring (9x9) around candidate dashes
  - Validates surrounding asphalt tarmac (V <= 70 for >= 60% pixels)
             │
             ▼
  [RANSAC Degree-2 Polynomial Fitting]
  - Inverted curve x = f(y) on dash centroids
  - Iterative 3-pass outlier rejection (|residual| < 12 px)
             │
             ▼
  [Dual-Marker Least-Squares Model]
  - Left Lane:   x_c = 0.9256 * x_dash - 132.21
  - Right Lane:  x_c = 0.9720 * x_dash + 0.2042 * x_yellow + 45.87 * slope + 38.06
  - Fallback:    Horizontal run-length encoding on solid yellow boundary
```

---

## 2. Source Code & Usage

### Key Functions (`lane_detection.py`):
- `detect_lane(frame)`: Main entry point returning `{"center_x": int, "lane": str}`.
- `_marking_masks(hsv)`: Isolate yellow boundary edges and white dashes.
- `_dash_points(white, hsv)`: Extracts centroids of white paint on dark tarmac via annular ring filtering.
- `_fit_dash_curve(points)`: Fits $x = f(y)$ polynomial through valid dash centroids.
- `_estimate_from_dashes(dash_fit, yellow_mask)`: Solves lane ID and center coordinate.

### Running with Visual Overlay:
```sh
conda activate NV_2199
cd task1c
python lane_detection.py public --show
```

### Batch Evaluation across all 20 clips:
```sh
eyantra-autoeval evaluate --year 2026 --theme NV --task 1c
```

---

## 3. Quantitative Evaluation Benchmarks (`result-2026-NV-1c-20261004.json`)

Evaluated on 4,000 frames across 20 video clips:

| Benchmark Metric | Measured Performance | Evaluation Requirement |
|---|:---:|:---:|
| **Processing Speed** | **505.8 FPS** (1.98 ms/frame) | Real-time ($\ge 20\text{ FPS}$) |
| **Lane ID Accuracy** | **97.9%** (3,630 / 3,708 scored frames) | $\ge 90\%$ |
| **Lane Center Precision** | **97.8%** (3,624 full credit $\le 30\text{ px}$) | Tolerant to $30\text{ px}$ |
| **Mean Center Error** | **5.5 pixels** (on 640x480 frame) | $< 15\text{ px}$ |
| **Lane Credit Score** | **19.06 / 20.00 marks** | Maximum 20 |
| **Center Credit Score** | **29.34 / 30.00 marks** | Maximum 30 |
| **Total Task 1C Score** | **48.40 / 50.00 marks** | **Maximum 50** |
