# Control Systems, Robotics Simulation & Actuation

This document describes the robotics simulation architecture, remote inter-process communication, real-time control loops, and actuation mechanics implemented for **Niti Vahan (eYRC 2026-27)**.

---

## Table of Contents
1. [Simulation Environment & Architectural Hierarchy](#1-simulation-environment--architectural-hierarchy)
2. [ZeroMQ Remote API & Synchronous Stepping](#2-zeromq-remote-api--synchronous-stepping)
3. [Actuation & Vehicle Model Dynamics](#3-actuation--vehicle-model-dynamics)
4. [Autonomous Stop Discipline & Level-Triggered State Machine](#4-autonomous-stop-discipline--level-triggered-state-machine)
5. [Velocity Profiling & Curvature Throttling](#5-velocity-profiling--curvature-throttling)
6. [Fault Tolerance, Dead Reckoning & Occlusion Handling](#6-fault-tolerance-dead-reckoning--occlusion-handling)

---

## 1. Simulation Environment & Architectural Hierarchy

The Niti Vahan simulation runs inside **CoppeliaSim Edu 4.10**, a distributed, physics-enabled multi-robot simulation platform. 

```
┌────────────────────────────────────────────────────────────────────────┐
│                        CoppeliaSim 4.10 Scene                          │
│                                                                        │
│   ┌────────────────────────── /Niti_Vahan ──────────────────────────┐  │
│   │                                                                 │  │
│   │   /steeringLeft (Revolute)            /steeringRight (Revolute) │  │
│   │        │                                   │                    │  │
│   │   /motorLeft (Velocity)               /motorRight (Velocity)   │  │
│   │        │                                   │                    │  │
│   │   [Front Left Wheel]                  [Front Right Wheel]       │  │
│   │                                                                 │  │
│   │   /freeAxisLeft (Passive)             /freeAxisRight (Passive)  │  │
│   │        │                                   │                    │  │
│   │   [Rear Left Wheel]                   [Rear Right Wheel]        │  │
│   │                                                                 │  │
│   │   /Camera_joint/Cuboid/visionSensor (Explicit Handling)         │  │
│   └─────────────────────────────────────────────────────────────────┘  │
│                                                                        │
│   Signals: NV_stop (Int32)                                             │
└───────────────────────────────────▲────────────────────────────────────┘
                                    │
                         ZMQ Remote API (Port 23004)
                                    │
┌───────────────────────────────────▼────────────────────────────────────┐
│                    Python 3.10 Control Application                     │
│                                                                        │
│   1. Read Frame (flip + RGB2BGR)                                       │
│   2. Read State / Signal (NV_stop)                                     │
│   3. Compute Perception (BEV / ArUco)                                  │
│   4. Calculate Steering & Velocity (Pure Pursuit / Stanley)            │
│   5. Split Angles via Ackermann Kinematics                             │
│   6. Write Targets to Joint Actuators                                  │
└────────────────────────────────────────────────────────────────────────┘
```

### 1.1 Object Tree Breakdown
- `/Niti_Vahan`: Base rigid body representing the vehicle chassis.
- `/steeringLeft`, `/steeringRight`: Independent revolute joints controlling front wheel steer angles.
- `/motorLeft`, `/motorRight`: Driven front-wheel revolute joints operating under angular velocity control.
- `/freeAxisLeft`, `/freeAxisRight`: Passive revolute joints supporting undriven rear wheels.
- `/Camera_joint/.../visionSensor`: Monocular camera mounted at forward overhang ($d_{cam} = 0.047\text{ m}$ ahead of center, $h = 0.125\text{ m}$), configured for explicit rendering on request.

---

## 2. ZeroMQ Remote API & Synchronous Stepping

### 2.1 Communication Protocol
Control communication between the Python client and CoppeliaSim takes place via the **CoppeliaSim ZMQ Remote API Client** utilizing **CBOR (Concise Binary Object Representation)** over ZeroMQ sockets.

### 2.2 Synchronous Stepping Mode
To ensure deterministic execution across varying hardware capabilities, simulations in Tasks 1B and 2C employ synchronous stepping:

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(True)  # Simulation only advances when client explicitly calls sim.step()
```

- Each simulation step advances physics by $\Delta t = 50\text{ ms}$ (20 Hz simulation clock).
- This decouples simulation dynamics from Python rendering or computation overhead, ensuring identical, reproducible trajectories regardless of CPU speed.

### 2.3 Optimization of Scene Object Handles
Calling `sim.getObject(path)` requires a complete round-trip network transaction (6–10 ms). The Niti Vahan architecture resolves all handles **once** during scene initialization:

```python
def connect_and_locate():
    sim = RemoteAPIClient().getObject('sim')
    handles = {
        'vision_sensor': sim.getObject('/Niti_Vahan/Camera_joint/Cuboid/visionSensor'),
        'steer_left':    sim.getObject('/steeringLeft'),
        'steer_right':   sim.getObject('/steeringRight'),
        'motor_left':    sim.getObject('/motorLeft'),
        'motor_right':   sim.getObject('/motorRight')
    }
    return sim, handles
```

Inside the main execution loop, only numerical target values are dispatched, reducing per-step communication latency to under 0.8 ms.

---

## 3. Actuation & Vehicle Model Dynamics

### 3.1 Velocity & Steering Actuator Commands
The vehicle chassis is front-wheel drive, front-wheel steer:
- **Steering Control**: Commanded via angular position targets:
  - `sim.setJointTargetPosition(handle, target_angle)`
- **Longitudinal Drive Control**: Commanded via angular velocity targets:
  - `sim.setJointTargetVelocity(handle, target_omega)`

The physical forward linear velocity $v$ relates to wheel angular speed $\omega$ and effective wheel radius $r_{wheel} = 0.0225\text{ m}$ by:

$$v = \omega \cdot r_{wheel}$$

For standard cruising speed $\omega = 3.0\text{ rad/s}$:

$$v = 3.0 \times 0.0225 = 0.0675\text{ m/s} \approx 6.75\text{ cm/s}$$

### 3.2 Mechanical Joint Clamping
The vehicle steering geometry imposes physical hardware limits of $\pm 45.0^\circ$ ($\pm 0.785\text{ rad}$) at the steering knuckles. 

To ensure that the inner steered wheel never hits physical stops or induces mechanical binding, the virtual steering angle is clamped prior to Ackermann splitting:

$$\delta_{clamped} = \max(-\delta_{limit}, \; \min(\delta_{limit}, \; \delta))$$

where $\delta_{limit} = 39.0^\circ$ in Task 2A and $30.0^\circ$ in Task 1B.

---

## 4. Autonomous Stop Discipline & Level-Triggered State Machine

A primary civic requirement of Niti Vahan is **disciplined observance of traffic signals and stop zones**. In Task 2A, the arena evaluation raises a simulator signal `NV_stop` at predetermined waypoints.

```
                  ┌──────────────────────┐
                  │    CRUISING STATE    │
                  │ - Track Lane         │
                  │ - Adaptive Speed     │
                  └──────────┬───────────┘
                             │
                  NV_stop == 1 raised
                             │
                             ▼
                  ┌──────────────────────┐
                  │   STATIONARY STATE   │
                  │ - Speed = 0.0 rad/s  │
                  │ - Maintain Steering  │
                  │ - Hold for >= 5.0 s  │
                  └──────────┬───────────┘
                             │
                  NV_stop == 0 cleared
                             │
                             ▼
                  ┌──────────────────────┐
                  │   RESUMPTION STATE   │
                  │ - Smoothly resume    │
                  │ - Continue same lane │
                  └──────────────────────┘
```

### 4.1 Level-Triggered vs. Pulse-Triggered Logic
`NV_stop` is a **level-triggered integer signal**, not a transient pulse:
- Signal Value `1`: Evaluation requests immediate and sustained halt.
- Signal Value `0` (or `None`): Roadway is clear to navigate.

### 4.2 Grace-Period Deceleration & Stationary Invariance
The evaluation protocol permits a brief grace window for braking deceleration, after which the vehicle must remain strictly motionless ($\Delta x \le 1\text{ mm}$) for the entire duration (typically 5.0 seconds).

Crucially, **the Python loop must never pause or execute `time.sleep()` while stopped**. Blocking the script prevents checking when `NV_stop` returns to 0, which would cause the vehicle to stall indefinitely.

```python
def compute_control(detection, stop_requested):
    left, right = ackermann_wheel_angles(delta)
    if stop_requested:
        # Command zero motor velocity while preserving active steering geometry
        return left, right, 0.0
    return left, right, speed
```

---

## 5. Velocity Profiling & Curvature Throttling

Operating at constant maximum forward velocity causes lateral centrifugal acceleration $a_c = v^2 \kappa$ to spike during sharp turns, provoking wheel slip and rapid camera perspective distortion.

### 5.1 Curvature-Dependent Speed Scaling (Task 2A)
Forward motor speed is continuously modulated as an inverse linear function of instantaneous steering angle magnitude $|\delta|$:

$$v_{target} = v_{cruise} - (v_{cruise} - v_{slow}) \cdot \min\left(1.0, \; \frac{|\delta|}{\delta_{limit}}\right)$$

where:
- $v_{cruise} = 3.0\text{ rad/s}$ ($6.75\text{ cm/s}$ on straights)
- $v_{slow} = 1.5\text{ rad/s}$ ($3.38\text{ cm/s}$ on hairpin curves)

```
   Speed (rad/s)
    3.0 ├──────────╮
        │          │
    2.0 │           \
        │            ╰────────────
    1.5 │                        
        └─────┬────────────┬──────► Steering Angle |δ|
             0°           39°
```

### 5.2 Multi-Stage Deceleration Profile for Precision Parking (Task 2C)
As the vehicle approaches the head of the parking bay, stopping distance precision requires staged deceleration to eliminate inertial overrun:

```python
if forward <= STOP_FORWARD:      # 0.130 m: Terminal Bay Middle
    speed = 0.0
elif forward > SLOW_FORWARD:     # > 0.90 m: Long-range crossing
    speed = SPEED_CRUISE         # 2.4 rad/s
elif forward > CREEP_FORWARD:    # 0.50 m - 0.90 m: Straightening
    speed = SPEED_MID            # 1.2 rad/s
else:                            # 0.13 m - 0.50 m: Bay Entrance
    speed = SPEED_CREEP          # 1.2 rad/s
```

After the vehicle reaches `STOP_FORWARD` with zero commanded velocity, the controller waits `SETTLE_FRAMES = 12` simulation steps ($600\text{ ms}$) to ensure complete dynamic settling before returning `parked = True`.

---

## 6. Fault Tolerance, Dead Reckoning & Occlusion Handling

Real-world autonomous systems must fail gracefully when sensor inputs degrade.

### 6.1 Lane Occlusion & Detector Dropout
When track markings are temporarily corrupted by glare, sudden turns, or line gaps, `detect_lane()` returns `None`.

The controller applies **exponential angle unwinding**:
- Steering decays gradually towards zero: $\delta_k = 0.90 \cdot \delta_{k-1}$
- Drive speed throttles down to $v_{slow} = 1.5\text{ rad/s}$

This prevents erratic snaps in steering and smoothly coasts the vehicle forward until markings reappear.

### 6.2 ArUco Marker Occlusion Recovery
In Task 2C, if the ArUco marker exits the camera's $95^\circ$ FOV due to excessive yaw turn:
1. **Short Dropout ($\le 10$ frames)**: Hold the last known steering command at half speed:
   `return _ST['steer'], 0.5 * _ST['speed'], False`
2. **Prolonged Dropout ($> 10$ frames)**: Dead reckoning automatically applies counter-steering back towards the road centerline at creep speed:
   `return 0.5 * _ST['side'], SPEED_CREEP, False`

This counter-steer naturally rotates the camera optical axis back towards the parking bay, instantly restoring the fiducial marker in the camera view.
