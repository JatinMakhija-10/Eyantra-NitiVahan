'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 2A of Niti Vahan (NV) Theme.
*
*  Filename:         task2a.py
*  Created:          2026
*  Last Modified:
*  Author:           e-Yantra Team
*
*  You are ONLY allowed to write your code inside the block marked
*  "ADD YOUR IMPLEMENTATION HERE". Do not change anything outside it - the
*  evaluation script relies on the rest of this file staying as it is.
*
*****************************************************************************************
'''

# Team ID:          2199
# Author List:      Jatin Makhija
# Filename:         task2a.py
# Functions:        detect_lane, compute_control, _get_matrix, _clusters, _ground_to_pixel, ackermann_wheel_angles, _dump_debug
# Global variables: WHEELBASE, TRACK_WIDTH, WHEEL_OFFSET, WHEEL_RADIUS, LANE_WIDTH_M, STEER_LIMIT, IMG_W, IMG_H,
#                   F_PX, HORIZON_ROW, LANE_OVER_H, YELLOW_HALF_OVER_LANE, HOOD, CAM_H_M, PITCH_RAD, BEV_W,
#                   PX_PER_H, M_PER_PX, LANE_PX, Z_NEAR_H, Z_FAR_H, BEV_H, CAR_X, Y_HALF_PX, SRC, DST,
#                   WHITE_LO/HI, YELLOW_LO/HI, N_BANDS, MIN_PIX, FIT_BANDS, EMA_ALPHA, CAM_AHEAD_M, LOOKAHEAD_M,
#                   K_PP, K_E, BEND_OUT_M, CRUISE, SLOW, DELTA_ALPHA, MAX_DELTA_STEP, _M, _state, _last_delta,
#                   _dbg, DEBUG_LOG, _ring, _log


####################### IMPORT MODULES #######################
import sys
import time

import cv2
import numpy as np

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
##############################################################


##################### SCENE CONSTANTS ########################
# Paths into the CoppeliaSim scene hierarchy, read off Task_2A.ttt.
VISION_SENSOR = '/Niti_Vahan/Camera_joint/Cuboid/visionSensor'
STEER_LEFT    = '/steeringLeft'
STEER_RIGHT   = '/steeringRight'
MOTOR_LEFT    = '/motorLeft'
MOTOR_RIGHT   = '/motorRight'

# The integer signal the evaluation raises when the vehicle must stop.
STOP_SIGNAL = 'NV_stop'
##############################################################


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################

import math
import atexit
import collections
import os

# ---------------- constants: FILL IN / CALIBRATE (see notes) ----------------
WHEELBASE    = 0.120    # m   from your Task 1A / 1B
TRACK_WIDTH  = 0.110    # m   from your Task 1A / 1B
WHEEL_OFFSET = 0.0275   # m   from your Task 1A / 1B
WHEEL_RADIUS = 0.0225   # m   estimated from your 1B log (0.029 m/s at 1.3 rad/s) - verify in the scene
LANE_WIDTH_M = 0.19     # m   yellow line's inner edge -> dashed line. Measured from your recording (lane centre-to-centre 0.195 m, wheel track 0.110 m as ruler)
STEER_LIMIT  = math.radians(39)    # 39 deg keeps the inner front wheel just under the vehicle's 45 deg limit (Task 1A split)

# ---- camera model (measured from your saved frames; 640x480) ----
IMG_W, IMG_H = 640, 480
F_PX        = 293.2     # focal length, px: 95 deg perspective angle across 640 px  (320 / tan(47.5 deg))
HORIZON_ROW = 67.0      # row where the road lines meet (measured on straight frames)
LANE_OVER_H = 1.52      # (yellow inner edge -> dash line) / camera height, measured on your straight frames at this field of view
YELLOW_HALF_OVER_LANE = 0.055  # half the yellow line width / lane width (22 px of 200 in the BEV)
HOOD = (415, 480, 215, 425)    # rows r0:r1, cols c0:c1 of the vehicle's own hood in the image -> masked out

CAM_H_M   = LANE_WIDTH_M / LANE_OVER_H           # camera height implied by the lane width
PITCH_RAD = math.atan2(IMG_H / 2.0 - HORIZON_ROW, F_PX)

# Bird's-eye view: 480 px wide (= +-1.64 camera heights), lane (inner edge -> dash) ~ 200 px
BEV_W   = 480
PX_PER_H = 200.0 / LANE_OVER_H
M_PER_PX = CAM_H_M / PX_PER_H
LANE_PX  = LANE_WIDTH_M / M_PER_PX               # = 200 px, inner edge -> dash
Z_NEAR_H, Z_FAR_H = 0.50, 4.20                   # ground range ahead of the camera, in camera heights
BEV_H   = int(round((Z_FAR_H - Z_NEAR_H) * PX_PER_H))
CAR_X   = BEV_W / 2.0                            # camera/vehicle axis in the BEV
Y_HALF_PX = YELLOW_HALF_OVER_LANE * LANE_PX


def _ground_to_pixel(X_h, Z_h):
    '''Pinhole model: ground point (right X, ahead Z; in camera heights) -> image pixel.'''
    s, c = math.sin(PITCH_RAD), math.cos(PITCH_RAD)
    xc, yc, zc = X_h, c - Z_h * s, s + Z_h * c
    return IMG_W / 2.0 + F_PX * xc / zc, IMG_H / 2.0 + F_PX * yc / zc


_xl, _xr = -(BEV_W / 2.0) / PX_PER_H, (BEV_W / 2.0) / PX_PER_H
SRC = np.float32([_ground_to_pixel(_xl, Z_FAR_H), _ground_to_pixel(_xr, Z_FAR_H),
                  _ground_to_pixel(_xr, Z_NEAR_H), _ground_to_pixel(_xl, Z_NEAR_H)])
DST = np.float32([[0, 0], [BEV_W, 0], [BEV_W, BEV_H], [0, BEV_H]])

WHITE_LO, WHITE_HI   = (0, 0, 180),  (180, 60, 255)
YELLOW_LO, YELLOW_HI = (15, 80, 80), (40, 255, 255)

N_BANDS   = 10
MIN_PIX   = 12
FIT_BANDS = 7            # nearest bands used for the lane fit
EMA_ALPHA = 0.6          # smoothing of the target (1.0 = none)

CAM_AHEAD_M = 0.107      # m   camera ahead of the rear axle: measured in the scene (0.107 m)
LOOKAHEAD_M = 0.32       # m   how far ahead (from the rear axle) the vehicle aims. Shorter = tighter, may wobble
K_PP        = 0.9        # steering gain on the pure-pursuit angle
CRUISE      = 3.0        # rad/s (~0.07 m/s) - start slow, raise once it laps reliably
SLOW        = 1.5        # rad/s while steering hard
BEND_OUT_M  = 0.035       # m   aim this far toward the OUTSIDE of a tight bend (keeps the rear wheels off the inside line)
K_E         = 1.2        # extra steering (rad) per lane-width of cross-track error; independent of any metric scale guess
DELTA_ALPHA    = 0.5     # steering smoothing (1.0 = none)
MAX_DELTA_STEP = math.radians(6)   # largest steering change per loop

_M = None
_state = {"y": 0.0, "en": 0.0, "have": False, "vote": 0}

# ---- optional flight recorder: set DEBUG_LOG = True to write debug_out/ (log.csv + last frames) when the run ends.
# Leave it False for submission.
DEBUG_LOG = False
_ring = collections.deque(maxlen=90)
_log = []


def _dump_debug():
    if not DEBUG_LOG or not _log:
        return
    os.makedirs('debug_out', exist_ok=True)
    with open('debug_out/log.csv', 'w') as f:
        f.write('t,n_bands,y_target_m,cross_track_lanes,steer_deg,speed,stop\n')
        for r in _log:
            f.write('%.3f,%d,%.4f,%.4f,%.2f,%.2f,%d\n' % r)
    for i, fr in enumerate(_ring):
        cv2.imwrite('debug_out/last_%02d.png' % i, fr)


atexit.register(_dump_debug)
_last_delta = 0.0
_dbg = {}               # filled every frame; dev_tools.py reads it, nothing is drawn here


def _get_matrix(h, w):
    global _M
    if _M is None:
        _M = cv2.getPerspectiveTransform(SRC * np.float32([w / IMG_W, h / IMG_H]), DST)
    return _M


def _clusters(mask_band, min_pix):
    '''x-centroids of separate blobs in a band mask (split on column gaps).'''
    colsum = mask_band.sum(axis=0)
    cols = np.nonzero(colsum)[0]
    if cols.size == 0:
        return []
    out = []
    start = prev = cols[0]
    for c in list(cols[1:]) + [None]:
        if c is None or c - prev > 12:
            xs = np.arange(start, prev + 1)
            wts = colsum[start:prev + 1].astype(np.float64)
            if wts.sum() / 255 >= min_pix:
                out.append((float((xs * wts).sum() / wts.sum()), int(start), int(prev)))
            if c is not None:
                start = c
        if c is not None:
            prev = c
    return out


def ackermann_wheel_angles(delta):
    '''Your Task 1A split, unchanged.'''
    half_pivot = TRACK_WIDTH / 2.0 - WHEEL_OFFSET
    t = math.tan(delta)
    left_angle = math.atan2(WHEELBASE * t, WHEELBASE - half_pivot * t)
    right_angle = math.atan2(WHEELBASE * t, WHEELBASE + half_pivot * t)
    return float(left_angle), float(right_angle)


def detect_lane(frame):
    '''
    Purpose:
    ---
    Work out, from a single camera frame, how wrong the vehicle currently is -
    and hand back whatever `compute_control()` needs in order to correct it.

    This is your Task 1C detector, extended. Task 1C answered *where the lane
    centre is* and *which lane the vehicle is in*; a controller cannot use
    either of those directly. It needs to know how far the vehicle is from
    where it should be, and which way it is pointing.

    Input Arguments:
    ---
    `frame` :   [ numpy.ndarray ]
        One BGR frame from the vehicle's camera, of shape (height, width, 3),
        the right way up and in the right colour order.

    Returns:
    ---
    Anything you like - a tuple, a dictionary, your own class. Whatever you
    return here is passed straight to compute_control() and nowhere else, so
    the two functions only have to agree with each other.

    Example call:
    ---
    detection = detect_lane(frame)

    REMEMBER:
    ---
    Both lanes are driven during evaluation, and the arena looks mirrored from
    the other one. Do not hard-code which side the boundary is on.

    Do NOT call cv2.imshow() or cv2.imwrite() from inside this function.

    You are marked on how closely you hold the MIDDLE of your lane, not merely
    on staying the correct side of the dashed line.
    '''

    if DEBUG_LOG:
        _ring.append(frame.copy())
    h, w = frame.shape[:2]
    M = _get_matrix(h, w)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    k = np.ones((3, 3), np.uint8)
    white = cv2.inRange(hsv, WHITE_LO, WHITE_HI)
    yellow = cv2.inRange(hsv, YELLOW_LO, YELLOW_HI)
    sy, sx = h / float(IMG_H), w / float(IMG_W)
    r0h, r1h, c0h, c1h = HOOD
    white[int(r0h * sy):int(r1h * sy), int(c0h * sx):int(c1h * sx)] = 0   # own hood is white
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, k)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)

    wb = (cv2.warpPerspective(white, M, (BEV_W, BEV_H)) > 127).astype(np.uint8) * 255
    yb = (cv2.warpPerspective(yellow, M, (BEV_W, BEV_H)) > 127).astype(np.uint8) * 255

    band_h = BEV_H // N_BANDS
    rows = []                 # per band: (distance ahead in px, left boundary x or None, right boundary x or None)
    white_right = white_left = 0
    ev = 0                    # evidence for "dashed line is on my right (+) / left (-)" from bands that show both lines
    ref = CAR_X               # where to look for the lane: starts at the vehicle axis, then follows the lane
    # which side the DASHED line is on is remembered (+1 right = outer lane, -1 left = inner lane), because in a
    # tight bend the dashes curve across the vehicle's axis and a position-only rule would call them the wrong edge
    ws = 1 if _state["vote"] >= 5 else (-1 if _state["vote"] <= -5 else 0)
    for i in range(N_BANDS):
        r1 = BEV_H - i * band_h
        r0 = r1 - band_h
        wcl = _clusters(wb[r0:r1], MIN_PIX)
        ycl = _clusters(yb[r0:r1], MIN_PIX)
        # --- position-only reading (used until the side is known, and as evidence for it)
        pm = [(c_, 'w') for c_, _, _ in wcl]
        for c_, x0_, x1_ in ycl:
            pm.append(((x1_ - Y_HALF_PX) if c_ < ref else (x0_ + Y_HALF_PX), 'y'))
        pl = [m for m in pm if 0.15 * LANE_PX < ref - m[0] < 1.3 * LANE_PX]
        pr = [m for m in pm if 0.15 * LANE_PX < m[0] - ref < 1.3 * LANE_PX]
        PL = max(pl, key=lambda m: m[0]) if pl else None
        PR = min(pr, key=lambda m: m[0]) if pr else None
        if PL and PR and PL[1] != PR[1]:
            ev += 1 if PR[1] == 'w' else -1
        # --- the reading actually used
        if ws == 0:
            L, R = PL, PR
        else:
            ym = []
            for c_, x0_, x1_ in ycl:                       # yellow is on the side OPPOSITE the dashed line
                ym.append(((x1_ - Y_HALF_PX) if ws > 0 else (x0_ + Y_HALF_PX), 'y'))
            wm = [(c_, 'w') for c_, _, _ in wcl]
            if ws > 0:    # dashes on the right, yellow on the left
                l_ = [m for m in ym if 0.15 * LANE_PX < ref - m[0] < 1.3 * LANE_PX]
                r_ = [m for m in wm if -0.3 * LANE_PX < m[0] - ref < 1.3 * LANE_PX]
            else:         # dashes on the left, yellow on the right
                l_ = [m for m in wm if -0.3 * LANE_PX < ref - m[0] < 1.3 * LANE_PX]
                r_ = [m for m in ym if 0.15 * LANE_PX < m[0] - ref < 1.3 * LANE_PX]
            L = max(l_, key=lambda m: m[0]) if l_ else None
            R = min(r_, key=lambda m: m[0]) if r_ else None
        if L is None and R is None:
            continue
        s_mid = BEV_H - 0.5 * (r0 + r1)
        rows.append((s_mid, L, R))
        if L and R:
            ref = 0.5 * (L[0] + R[0])
        elif L:
            ref = L[0] + LANE_PX / 2.0
        else:
            ref = R[0] - LANE_PX / 2.0
        white_right += bool(R and R[1] == 'w')
        white_left += bool(L and L[1] == 'w')

    # second pass: in a bend the lane looks wider SIDEWAYS (width / cos of its direction), so the
    # half-width used when only one line is visible has to follow the lane's local direction
    pts_s, pts_x = [], []
    if len(rows) >= 3:
        s0 = np.array([r[0] for r in rows])
        x0 = np.array([0.5 * (r[1][0] + r[2][0]) if (r[1] and r[2]) else
                       (r[1][0] + LANE_PX / 2.0 if r[1] else r[2][0] - LANE_PX / 2.0) for r in rows])
        rough = np.polyfit(s0, x0, 2 if len(rows) >= 5 else 1)
        dr = np.polyder(rough)
        for (s_, L, R) in rows:
            hw = 0.5 * LANE_PX * math.sqrt(1.0 + np.polyval(dr, s_) ** 2)
            if L and R:
                if abs((R[0] - L[0]) - 2 * hw) > 0.5 * 2 * hw:
                    continue
                cx = 0.5 * (L[0] + R[0])
            elif L:
                cx = L[0] + hw
            else:
                cx = R[0] - hw
            pts_s.append(s_)
            pts_x.append(cx)

    _dbg.update(bev=cv2.max(wb, yb), pts_s=pts_s, pts_x=pts_x)
    if len(pts_s) < 3:
        _state["have"] = False
        return None

    # update the remembered side from straight-ish views that show both lines
    if len(pts_x) >= 6 and abs(pts_x[-1] - pts_x[0]) < 0.25 * LANE_PX and ev != 0:
        _state["vote"] = max(-20, min(20, _state["vote"] + (1 if ev > 0 else -1)))
    # PURE PURSUIT target: a point on the lane centre LOOKAHEAD_M ahead of the rear axle, read by
    # INTERPOLATING a fit of the visible bands (never extrapolating behind the view, which breaks
    # wherever a straight turns into a bend). Steering on a circular arc to that point is exact on a
    # steady bend, so the vehicle does not cut it.
    ps, px_ = np.array(pts_s), np.array(pts_x)
    kf = max(3, min(len(ps), FIT_BANDS))
    coef = np.polyfit(ps[:kf], px_[:kf], 2 if kf >= 5 else 1)
    s_L = (LOOKAHEAD_M - CAM_AHEAD_M) / M_PER_PX - Z_NEAR_H * PX_PER_H     # px above the bottom of the view
    s_L = min(max(s_L, ps[0]), ps[kf - 1])
    y_L = (CAR_X - np.polyval(coef, s_L)) * M_PER_PX          # + : the target is to the LEFT (metres)
    ahead = CAM_AHEAD_M + Z_NEAR_H * CAM_H_M + s_L * M_PER_PX  # metres ahead of the rear axle

    # scale-free cross-track error: lane-centre offset at the nearest band, in lane widths (+ : centre is LEFT)
    en = (CAR_X - float(np.polyval(coef, ps[0]))) / LANE_PX

    if _state["have"]:
        y_L = EMA_ALPHA * y_L + (1 - EMA_ALPHA) * _state["y"]
        en = EMA_ALPHA * en + (1 - EMA_ALPHA) * _state["en"]
    _state.update(y=y_L, en=en, have=True)

    lane = "unknown"
    if _state["vote"] >= 5 or (_state["vote"] > -5 and white_right > white_left):
        lane = "left"
    elif _state["vote"] <= -5 or white_left > white_right:
        lane = "right"
    return {"y": float(y_L), "ahead": float(ahead), "en": float(en), "n": len(pts_s), "lane": lane}


def compute_control(detection, stop_requested):
    '''
    Purpose:
    ---
    Turn the errors from detect_lane() into the three numbers the vehicle is
    driven with, and obey the stop flag.

    This is where your Task 1B controller and your Task 1A steering geometry
    meet: work out one steering angle from the error, then split it into the
    two front-wheel angles the joints actually take.

    Input Arguments:
    ---
    `detection` :   [ any ]
        Whatever detect_lane() returned for this frame.

    `stop_requested` :  [ bool ]
        True while the evaluation is holding the vehicle at a stop point.
        This is a LEVEL, not a pulse: it stays True for the whole duration of
        the stop and goes False when the wait is over. While it is True the
        vehicle must be STATIONARY, so return a speed of 0.0. There is no
        stopwatch for you to write and no duration to hard-code.

    Returns:
    ---
    `left_angle`, `right_angle` :   [ float ]
        The two front-wheel steering angles, in RADIANS, written straight to
        the steering joints. The scene's steering joints are POSITIVE TO THE
        LEFT. Nothing is clamped for you - the vehicle's own limit is 45
        degrees, and commanding past it will not steer any harder.

    `speed` :   [ float ]
        Target velocity for both drive motors, in rad/s. 0.0 holds the
        vehicle still.

    Example call:
    ---
    left_angle, right_angle, speed = compute_control(detection, stop_requested)

    REMEMBER:
    ---
    Keep the loop running while stopped - set the SPEED to zero, not your
    program. If you stop reading frames you will not see the flag drop.

    Sooner or later a frame comes back with nothing usable in it. Holding the
    last steering angle, easing off the throttle and stopping dead are all
    defensible - doing nothing is not.
    '''

    global _last_delta
    if detection is None:
        delta = 0.9 * _last_delta            # lost: unwind slowly, creep on
        speed = SLOW
    else:
        # pure pursuit: curvature of the circular arc through the target point, then the steering
        # angle that produces it (Ackermann, rear axle) - Task 1A turns it into the two wheel angles
        y, x = detection["y"], detection["ahead"]
        curvature = 2.0 * y / (x * x + y * y)
        # aim a little to the OUTSIDE of a bend: the rear wheels cut inside the front wheels' path, and in
        # the inner lane the yellow line is right there on the inside
        shift = math.copysign(BEND_OUT_M * min(1.0, abs(curvature) / 2.0), curvature)
        y -= shift
        curvature = 2.0 * y / (x * x + y * y)
        delta = K_PP * math.atan(WHEELBASE * curvature) + K_E * (detection["en"] - shift / LANE_WIDTH_M)   # + cross-track feedback (scale-free)
        delta = max(-STEER_LIMIT, min(STEER_LIMIT, delta))
        delta = DELTA_ALPHA * delta + (1 - DELTA_ALPHA) * _last_delta
        step = max(-MAX_DELTA_STEP, min(MAX_DELTA_STEP, delta - _last_delta))
        delta = _last_delta + step
        speed = CRUISE - (CRUISE - SLOW) * min(1.0, abs(delta) / STEER_LIMIT)

    _last_delta = delta
    left, right = ackermann_wheel_angles(delta)
    if DEBUG_LOG:
        _log.append((time.time(), -1 if detection is None else detection['n'], 0 if detection is None else detection['y'],
                     0 if detection is None else detection['en'], math.degrees(delta), speed, int(bool(stop_requested))))
    if stop_requested:
        return left, right, 0.0              # keep steering, hold still
    return left, right, speed


##############################################################
############## END OF YOUR IMPLEMENTATION ####################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################
#
# Connecting to the simulator, reading the camera and writing to the joints.
# None of it involves a decision you are being marked on - it is here so that
# every team reads the same picture and drives the same vehicle.
#
#####################################################################

def connect_and_locate():
    '''Open the remote API connection and look up every scene handle once.

    Each getObject() is a network round trip, which is why they all happen
    here rather than inside the loop.
    '''
    try:
        sim = RemoteAPIClient().getObject('sim')
        sim.getSimulationTime()
    except Exception as err:
        raise ConnectionError(
            f'Could not reach CoppeliaSim on the ZMQ Remote API port.\n'
            f'  Is CoppeliaSim running with Task_2A.ttt open?\n'
            f'  Simulator said: {err}')

    handles = {}
    for name, path in (('vision_sensor', VISION_SENSOR),
                       ('steer_left', STEER_LEFT),
                       ('steer_right', STEER_RIGHT),
                       ('motor_left', MOTOR_LEFT),
                       ('motor_right', MOTOR_RIGHT)):
        try:
            handles[name] = sim.getObject(path)
        except Exception as err:
            raise RuntimeError(
                f'No object at "{path}" in the open scene.\n'
                f'  Open Task_2A.ttt from the repository.\n'
                f'  Simulator said: {err}')
    return sim, handles


def read_frame(sim, handles):
    '''Grab one frame from the vehicle's camera as a BGR image.

    The vision sensor hands back a raw byte buffer and a resolution, not an
    image, and three corrections are needed to turn one into the other. None
    of these problems announces itself - each produces a detector that looks
    like it is working on a frame that is wrong:

      * handleVisionSensor() - this scene's sensor renders only when asked.
        Without it every read returns the SAME frame for the whole lap.
      * cv2.flip(frame, 0) - the buffer's first row is the BOTTOM of the
        image, so the road comes out vertically mirrored.
      * RGB2BGR - the buffer is RGB and OpenCV assumes BGR. Without the swap
        every HSV threshold you tuned in Task 1C selects the wrong colour.

    Note `res[1], res[0]` in the reshape too: the resolution comes back as
    (width, height) while NumPy wants rows first.

    Returns None if no frame is ready yet, which happens on the first few
    iterations after the simulation starts.
    '''
    sim.handleVisionSensor(handles['vision_sensor'])
    img, res = sim.getVisionSensorImg(handles['vision_sensor'], 0)
    if len(img) == 0:
        return None
    frame = np.frombuffer(img, dtype=np.uint8).reshape((res[1], res[0], 3))
    frame = cv2.flip(frame, 0)
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def drive(sim, handles, left_angle, right_angle, speed):
    '''Write the two steering angles and the drive speed to the joints.'''
    sim.setJointTargetPosition(handles['steer_left'], float(left_angle))
    sim.setJointTargetPosition(handles['steer_right'], float(right_angle))
    sim.setJointTargetVelocity(handles['motor_left'], float(speed))
    sim.setJointTargetVelocity(handles['motor_right'], float(speed))


def main():
    sim, handles = connect_and_locate()

    # While you are developing you press Play yourself. Under evaluation the
    # harness resets the scene, places the vehicle in the lane being tested
    # and starts the simulation - so wait for that rather than starting your
    # own, which would race the harness into position.
    #
    # Your script must never call startSimulation(), stopSimulation() or move
    # the vehicle. Reading the camera and driving the joints is all it does.
    deadline = time.monotonic() + 120.0
    while sim.getSimulationState() != sim.simulation_advancing_running:
        if time.monotonic() > deadline:
            print('The simulation never started. Press Play in CoppeliaSim, '
                  'then run this script again.')
            return 1
        time.sleep(0.05)
    print('Simulation running - driving.')

    frames = 0
    try:
        # The harness stops the simulation when the lap is over, which is the
        # cue to exit - hence testing the state rather than looping forever.
        while sim.getSimulationState() == sim.simulation_advancing_running:
            frame = read_frame(sim, handles)
            if frame is None:
                continue
            frames += 1

            # getInt32Signal() returns None until the signal is first set, so
            # "not set" reads as "not stopping" and the first frames behave.
            stop_requested = bool(sim.getInt32Signal(STOP_SIGNAL))

            detection = detect_lane(frame)
            left_angle, right_angle, speed = compute_control(detection,
                                                             stop_requested)
            drive(sim, handles, left_angle, right_angle, speed)
    except KeyboardInterrupt:
        print('\nInterrupted.')
    except Exception as err:
        # A call raising because the simulation stopped mid-frame is normal at
        # the end of a run; anything else is a bug worth seeing.
        print(f'Loop ended: {type(err).__name__}: {err}')
    finally:
        # Without this, whatever speed was last commanded is still being
        # applied and the vehicle drives on unsteered.
        try:
            sim.setJointTargetVelocity(handles['motor_left'], 0.0)
            sim.setJointTargetVelocity(handles['motor_right'], 0.0)
        except Exception:
            pass

    print(f'{frames} frames processed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
