# Mathematical Foundations & Control Theory

This document provides rigorous mathematical derivations, kinematic models, stability analyses, and geometric transformation proofs utilized across the **Niti Vahan** autonomous driving pipeline for **e-Yantra 2026-27 (IIT Bombay)**.

---

## Table of Contents
1. [Ackermann Steering Geometry with Kingpin Offset](#1-ackermann-steering-geometry-with-kingpin-offset)
2. [Vehicle Kinematics & The Bicycle Model](#2-vehicle-kinematics--the-bicycle-model)
3. [Closed-Loop Path Tracking: Stanley Controller](#3-closed-loop-path-tracking-stanley-controller)
4. [Adaptive Pure Pursuit with Off-Tracking Compensation](#4-adaptive-pure-pursuit-with-off-tracking-compensation)
5. [Analytical Inverse Perspective Mapping (IPM) & Homography](#5-analytical-inverse-perspective-mapping-ipm--homography)
6. [State-Space Trajectory Generation for Precision Bay Parking](#6-state-space-trajectory-generation-for-precision-bay-parking)

---

## 1. Ackermann Steering Geometry with Kingpin Offset

### 1.1 The Classical Ackermann Condition
To prevent lateral tire scrubbing during steady-state cornering, all four wheels must roll instantaneously about a single common Instantaneous Centre of Rotation (ICR).

For a vehicle with wheelbase $L$ and effective track width $W_{eff}$, turning at a virtual central steering angle $\delta$:

$$\tan \delta = \frac{L}{R}$$

where $R$ is the turning radius measured from the vehicle centerline to the rear axle. The classical kinematic relationships for the inner ($\delta_i$) and outer ($\delta_o$) wheel steering angles are:

$$\cot \delta_o - \cot \delta_i = \frac{W_{eff}}{L}$$

```
                ▲ X (Forward)
                │
         Left Wheel       Right Wheel
         ┌──────┐           ┌──────┐
         │  δ_L │           │  δ_R │
     ────┼──────┼─────┬─────┼──────┼────  Front Axle
         │      │     │     │      │
         └──────┘     │     └──────┘
                      │
         ◄─── O ───►  │  ◄─── O ───►
         Kingpin-to-  │  Kingpin-to-
         wheel offset │  wheel offset
                      │
                      │ L (Wheelbase)
                      │
         ┌──────┐     │     ┌──────┐
         │      │     │     │      │
     ────┴──────┴─────┴─────┴──────┴────  Rear Axle (Undriven/Freewheel)
                 Centerline
                ◄────── W ──────►
                  (Track Width)
```

### 1.2 Incorporation of Kingpin Pivot Offset
In the physical and simulated Niti Vahan chassis:
- Wheelbase: $L = 0.120\text{ m}$
- Wheel Track Width (center-to-center): $W = 0.110\text{ m}$
- Kingpin-to-Wheel Center Offset: $O = 0.0275\text{ m}$

Because steering pivots physically exist at the kingpins rather than the wheel contact patch centers, the effective pivot separation distance between the steering axes is:

$$W_{piv} = W - 2 \cdot O = 0.110 - 2(0.0275) = 0.055\text{ m}$$

The half-pivot distance from the central axis is:

$$d_{piv} = \frac{W_{piv}}{2} = \frac{W}{2} - O = 0.0275\text{ m}$$

For a given virtual steering angle $\delta$, let $t = \tan \delta$. The turning radius of the virtual central front wheel relative to the rear axle is $R = L / t$.

Using trigonometric projection on the left and right steering triangles:
- For a leftward turn ($\delta > 0$):
  - Left wheel turning radius: $R_L = R - d_{piv} = \frac{L}{t} - d_{piv} = \frac{L - d_{piv} t}{t}$
  - Right wheel turning radius: $R_R = R + d_{piv} = \frac{L}{t} + d_{piv} = \frac{L + d_{piv} t}{t}$

Applying the relation $\tan \delta_L = \frac{L}{R_L}$ and $\tan \delta_R = \frac{L}{R_R}$:

$$\tan \delta_L = \frac{L}{\frac{L - d_{piv} t}{t}} = \frac{L \tan \delta}{L - d_{piv} \tan \delta}$$

$$\tan \delta_R = \frac{L}{\frac{L + d_{piv} t}{t}} = \frac{L \tan \delta}{L + d_{piv} \tan \delta}$$

### 1.3 Exact Closed-Form Implementation
To avoid numerical overflow or division-by-zero singularities as $\delta \to 0$, we formulate the angles using two-argument arctangent ($\text{atan2}$):

$$\delta_L = \text{atan2}\left(L \tan \delta,\; L - \left(\frac{W}{2} - O\right)\tan \delta\right)$$

$$\delta_R = \text{atan2}\left(L \tan \delta,\; L + \left(\frac{W}{2} - O\right)\tan \delta\right)$$

#### Properties of this formulation:
1. **$\delta = 0$ Smoothness**: As $\delta \to 0$, $\tan(0) = 0$, giving $\text{atan2}(0, L) = 0.0\text{ rad}$ for both wheels without conditional branch discontinuities.
2. **Asymmetric Steering Ackermann Condition**: For $\delta > 0$ (left turn), $\delta_L > \delta > \delta_R$, ensuring the inner wheel steers sharper than the outer wheel.
3. **Sign Invariance**: Evaluates symmetrically for negative steering inputs ($\delta < 0$).

---

## 2. Vehicle Kinematics & The Bicycle Model

Under the assumption of planar motion and negligible lateral tire slip (valid for sub-1 m/s operational regime in urban arena navigation), the vehicle state is defined in the inertial frame by:

$$\mathbf{x} = \begin{bmatrix} x \\ y \\ \psi \\ v \end{bmatrix}$$

where $(x, y)$ is the rear axle position, $\psi$ is the heading angle relative to the global X-axis, and $v$ is the longitudinal forward velocity.

```
       ▲ Global Y
       │         / Front Axle (δ)
       │        / 
       │       /
       │      /  L
       │     /
       │    /
       │   ● (x, y) Rear Axle
       │    \
       │     \ ψ (Heading)
       └──────┴──────────────────► Global X
```

The continuous-time kinematic equations of motion:

$$\dot{x} = v \cos \psi$$

$$\dot{y} = v \sin \psi$$

$$\dot{\psi} = \frac{v}{L} \tan \delta$$

$$\dot{v} = a$$

---

## 3. Closed-Loop Path Tracking: Stanley Controller

In Task 1B, the vehicle must execute high-precision dynamic lane switches along a straight corridor ($y_{target} \in \{0.0\text{ m}, 0.2\text{ m}\}$) at constant speed $v$.

### 3.1 Non-Linear Control Law
The Stanley controller computes a steering command $\delta(t)$ composed of two orthogonal error compensations:
1. **Heading Error** $\theta_e$: The difference between vehicle heading and path orientation.
2. **Cross-Track Error** $e(t)$: The perpendicular distance from the front axle center to the nearest path point.

$$\delta(t) = \theta_e(t) + \arctan\left(\frac{k \cdot e(t)}{v(t) + \epsilon}\right)$$

In our world frame, the vehicle drives along $-X$, where the left lane is $y = 0.0$ and the right lane is $y = 0.2\text{ m}$. Hence, a positive lateral error ($y > y_{target}$) mandates a leftward steering correction ($\delta > 0$).

Setting heading error as $-\psi$ and cross-track error as $(y - y_{target})$:

$$\delta(t) = K_{head} \cdot \text{wrap}(-\psi) + \arctan(K_{cross} \cdot (y - y_{target}))$$

where:
- $K_{head} = 2.0$
- $K_{cross} = 20.0$
- Steering is clamped to $[-\delta_{max}, +\delta_{max}] = [-30^\circ, +30^\circ]$.

### 3.2 Stability Proof via Lyapunov Analysis
Consider the cross-track error kinematics for a straight trajectory along $\psi_{path} = 0$:

$$\dot{e}(t) = -v \sin(\psi(t) - \delta(t))$$

When $\theta_e = -\psi \approx 0$, substituting $\delta = \arctan\left(\frac{k e}{v}\right)$:

$$\sin(\delta) = \frac{\frac{k e}{v}}{\sqrt{1 + \left(\frac{k e}{v}\right)^2}} = \frac{k e}{\sqrt{v^2 + k^2 e^2}}$$

$$\dot{e}(t) = -v \frac{k e}{\sqrt{v^2 + k^2 e^2}} = -\frac{k v}{\sqrt{v^2 + k^2 e^2}} e(t)$$

Define the scalar positive-definite Lyapunov function:

$$V(e) = \frac{1}{2} e^2$$

Taking its time derivative along system trajectories:

$$\dot{V}(e) = e \cdot \dot{e} = -\frac{k v}{\sqrt{v^2 + k^2 e^2}} e^2 < 0 \quad \forall e \neq 0, \; v > 0$$

Since $\dot{V}(e)$ is strictly negative-definite for all $e \neq 0$ and radially unbounded, the cross-track error $e(t) \to 0$ asymptotically with zero steady-state error.

---

## 4. Adaptive Pure Pursuit with Off-Tracking Compensation

For continuous closed-loop guidance along the curved arena in Task 2A, the vehicle utilizes an enhanced **Pure Pursuit** controller operating directly in the vehicle-centric coordinate system.

```
                    Target Point (x_L, y_L)
                           ●
                          /│
                         / │
                        /  │
                  L_d  /   │ y_L (Lateral target)
                      /    │
                     /     │
                    /  α   │
                   ●───────┘
            Rear Axle (0, 0)
                 x_L (Lookahead ahead)
```

### 4.1 Circular Arc Geometry
Pure Pursuit assumes the vehicle tracks a circular arc connecting the rear axle center $(0, 0)$ to a chosen lookahead point $(x_L, y_L)$ situated at distance $L_d = \sqrt{x_L^2 + y_L^2}$ ahead:

$$\sin \alpha = \frac{y_L}{L_d}$$

From the law of sines on the isosceles triangle formed by the circle radius $R_{arc}$:

$$\frac{L_d}{\sin(2\alpha)} = \frac{R_{arc}}{\cos \alpha} \implies R_{arc} = \frac{L_d}{2 \sin \alpha} = \frac{L_d^2}{2 y_L}$$

The path curvature $\kappa$ is the inverse of $R_{arc}$:

$$\kappa = \frac{2 y_L}{L_d^2} = \frac{2 y_L}{x_L^2 + y_L^2}$$

The required virtual steering angle $\delta_{pp}$ for a bicycle of wheelbase $L$:

$$\tan \delta_{pp} = L \cdot \kappa \implies \delta_{pp} = \arctan\left(L \cdot \frac{2 y_L}{x_L^2 + y_L^2}\right)$$

### 4.2 Rear-Wheel Cut-In & Off-Tracking Offset
In sharp turns, the rear axle tracks inside the circular radius of the steered front axle. The geometric off-tracking distance $\Delta r$ between front and rear axle paths on a turn of radius $R$ is:

$$\Delta r = R - \sqrt{R^2 - L^2} \approx \frac{L^2}{2R}$$

On the tight inner arena lane ($R \approx 0.4\text{ m}$), this inward cut-in causes the rear wheel to graze the inner yellow boundary line.

To eliminate this hazard without complicating the trajectory planner, our controller injects a dynamic outer-bias offset $\Delta y_{shift}$ proportional to instantaneous curvature:

$$\Delta y_{shift} = \text{sgn}(\kappa) \cdot B_{out} \cdot \min\left(1.0, \frac{|\kappa|}{2.0}\right)$$

where $B_{out} = 0.035\text{ m}$. The corrected target coordinate becomes:

$$y_L^* = y_L - \Delta y_{shift}$$

### 4.3 Composite Steering Control Law
To maintain strict lane centering while tracking curves, the total steering command combines Pure Pursuit geometric guidance with scale-free cross-track error feedback $e_n$:

$$\delta_{cmd} = K_{pp} \arctan(L \cdot \kappa^*) + K_e \left(e_n - \frac{\Delta y_{shift}}{W_{lane}}\right)$$

where:
- $K_{pp} = 0.9$
- $K_e = 1.2\text{ rad / lane-width}$
- Slew rate limiting: $|\Delta \delta| \le 6^\circ \text{/loop}$
- Exponential Moving Average: $\delta_k = \alpha_\delta \delta_{cmd} + (1 - \alpha_\delta) \delta_{k-1}$ ($\alpha_\delta = 0.5$)

---

## 5. Analytical Inverse Perspective Mapping (IPM) & Homography

The vehicle's forward camera captures images in perspective projection ($640 \times 480\text{ px}$). Curved lane marks become non-linear hyperbolas, and road width tapers towards the vanishing line. Inverse Perspective Mapping (IPM) rectifies the camera image onto an orthogonal top-down Bird's-Eye View (BEV).

```
  Camera View (Perspective)                Bird's-Eye View (Orthogonal Metric)
         Horizon y_0                             Y_bev ▲
  ┌────────────────────────┐                           │
  │     \            /     │                           │  ║       ║
  │      \          /      │      ──────► IPM ──────►  │  ║       ║  Parallel
  │       \        /       │                           │  ║   ●   ║  Lanes
  │        \      /        │                           │  ║ (Car) ║
  └────────────────────────┘                           └──────┴───────► X_bev
```

### 5.1 Optical & Spatial Camera Parameters
- Horizontal Field of View: $\theta_{hfov} = 95^\circ = 1.658\text{ rad}$
- Image Dimensions: $W_{img} = 640$, $H_{img} = 480$
- Focal Length in pixels:

$$f_x = \frac{W_{img} / 2}{\tan(\theta_{hfov} / 2)} = \frac{320}{\tan(47.5^\circ)} = 293.20\text{ px}$$

- Measured Horizon Row (road convergence): $y_0 = 67.0\text{ px}$
- Camera Tilt (Pitch angle $\theta_p$ down from horizon):

$$\theta_p = \arctan\left(\frac{H_{img} / 2 - y_0}{f_x}\right) = \arctan\left(\frac{240 - 67}{293.2}\right) = \arctan(0.590) \approx 30.54^\circ$$

### 5.2 Pinhole Ground-to-Pixel Projection
Let a ground point in normalized camera heights $(X_h, Z_h)$ be given where $Z_h$ is the forward distance and $X_h$ is the lateral distance. 

Rotating by pitch angle $\theta_p$ into camera optical frame coordinates $(X_c, Y_c, Z_c)$:

$$\begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos \theta_p & -\sin \theta_p \\ 0 & \sin \theta_p & \cos \theta_p \end{bmatrix} \begin{bmatrix} X_h \\ 1 \\ Z_h \end{bmatrix} = \begin{bmatrix} X_h \\ \cos \theta_p - Z_h \sin \theta_p \\ \sin \theta_p + Z_h \cos \theta_p \end{bmatrix}$$

Projecting onto camera sensor pixel coordinates $(u, v)$:

$$u = \frac{W_{img}}{2} + f_x \frac{X_c}{Z_c} = 320 + 293.2 \cdot \frac{X_h}{\sin \theta_p + Z_h \cos \theta_p}$$

$$v = \frac{H_{img}}{2} + f_x \frac{Y_c}{Z_c} = 240 + 293.2 \cdot \frac{\cos \theta_p - Z_h \sin \theta_p}{\sin \theta_p + Z_h \cos \theta_p}$$

### 5.3 Metric Homography Matrix $\mathbf{M}$
By evaluating ground bounding corners over range $Z_h \in [0.50, 4.20]$ and lateral reach $X_h \in [-1.64, +1.64]$, four ground points map to source image quad $\mathbf{S}$. The destination quad $\mathbf{D}$ is fixed to a metric grid of size $480 \times 320\text{ px}$.

The transformation matrix $\mathbf{M} \in \mathbb{R}^{3 \times 3}$ is computed via Direct Linear Transformation (DLT):

$$\begin{bmatrix} u_{bev} \\ v_{bev} \\ 1 \end{bmatrix} \sim \mathbf{M} \begin{bmatrix} u_{img} \\ v_{img} \\ 1 \end{bmatrix}$$

Applying $\mathbf{M}$ via `cv2.warpPerspective` generates a flat, metric plane where lane widths ($200\text{ px} = 0.19\text{ m}$) remain strictly constant regardless of distance.

---

## 6. State-Space Trajectory Generation for Precision Bay Parking

In Task 2C, the vehicle starts on the roadway and must park in Bay 0 ($0.22\text{ m}$ right) or Bay 1 ($0.44\text{ m}$ right) with clearance tolerances under $20\text{ mm}$ and heading error $< 2^\circ$.

```
               Y (Lateral offset from bay center)
               ▲
               │      Spawn State (y = 0.22m or 0.44m, ψ = 0)
               │      ●─────────────┐
               │                     \
               │                      \ Constant heading crossing (θ_c)
               │                       \
               │                        \
               │                         \  Tangential Arc (R = 0.14m)
               │                          \   ╭───
               │                           ╰──┤   Bay Centerline (y = 0, ψ = 0)
               └──────────────────────────────┴────────────────────────► X (Forward)
                                              Bay Mouth        Bay Middle
                                              (0.37m)          (0.19m)
```

### 6.1 State-Space Kinematic Planning
Let $y$ denote lateral offset from the target bay centerline, and $\psi$ denote heading relative to the bay axis. The goal is to reach $(y = 0, \psi = 0)$ before reaching the bay mouth ($X_{mouth} = 0.37\text{ m}$).

The trajectory is partitioned into two analytical stages:
1. **Constant-Heading Diagonal Intercept**: Vehicle steers to a fixed crossing angle $\theta_c$.
2. **Circular Tangential Straightening Arc**: Vehicle follows a circular arc of fixed radius $R_{arc} = 0.14\text{ m}$ that terminates precisely tangential to the bay centerline ($y = 0, \psi = 0$).

### 6.2 Analytical Tangential Arc Relationship
On the terminal arc of radius $R_{arc}$, the lateral displacement accumulated while turning through heading angle $\theta$ is:

$$\Delta y_{arc}(\theta) = R_{arc} (1 - \cos \theta)$$

The forward longitudinal distance traversed during this arc is:

$$\Delta x_{arc}(\theta) = R_{arc} \sin \theta$$

To smoothly transition from the diagonal straight path (at angle $\theta_c$) into the straightening arc, the required remaining lateral distance for the straight crossing is:

$$\Delta y_{diag} = |y| - \Delta y_{arc}(\theta_c) = |y| - R_{arc}(1 - \cos \theta_c)$$

The forward distance consumed by the diagonal crossing is:

$$\Delta x_{diag} = \frac{\Delta y_{diag}}{\tan \theta_c} = \frac{|y| - R_{arc}(1 - \cos \theta_c)}{\tan \theta_c}$$

The total forward distance $X_{req}(\theta_c)$ needed to complete both the crossing and the arc from initial lateral offset $y_0$ is:

$$X_{req}(\theta_c) = 2 R_{arc} \sin \theta_c + \frac{|y_0| - 2 R_{arc}(1 - \cos \theta_c)}{\tan \theta_c}$$

### 6.3 Optimal Crossing Angle Optimization
Given the available longitudinal distance $X_{avail} = X_{forward} - X_{end}$ (where $X_{end} = 0.48\text{ m}$):

$$\theta_c^* = \min \left\{ \theta \in [\theta_{min}, \theta_{max}] \;\middle|\; X_{req}(\theta) \le X_{avail} \right\}$$

where $\theta_{min} = 0.30\text{ rad} \approx 17.2^\circ$ and $\theta_{max} = 0.74\text{ rad} \approx 42.4^\circ$.

- For **Bay 0** ($y_0 = 0.22\text{ m}$): The solver yields a gentle angle $\theta_c^* \approx 25^\circ$.
- For **Bay 1** ($y_0 = 0.44\text{ m}$): Double the lateral crossing in the same distance yields $\theta_c^* \approx 40^\circ$.

### 6.4 Feedforward-Feedback Combined Steering
At any instantaneous offset $y$ along the trajectory:

$$\theta_{wanted}(y) = -\text{sgn}(y) \cdot \min\left(\theta_c^*,\; \arccos\left(1 - \frac{|y|}{R_{arc}}\right),\; K_{line} |y|\right)$$

The feedforward curvature is:

$$\kappa_{ff} = \begin{cases} \frac{\text{sgn}(y)}{R_{arc}} & \text{if in the terminal arc zone} \\ 0 & \text{otherwise} \end{cases}$$

The total commanded front steering curvature combines feedforward and proportional heading feedback:

$$\kappa_{cmd} = \eta_{ff} \kappa_{ff} + \frac{\theta_{wanted} - \psi_{curr}}{D_{head}}$$

$$\delta = \arctan(L \cdot \kappa_{cmd})$$

where $\eta_{ff} = 0.7$ and $D_{head} = 0.12\text{ m}$.

This feedforward-feedback coupling guarantees zero lag behind the planned geometry, achieving sub-centimeter alignment into both parking bays.
