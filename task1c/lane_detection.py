'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 1C of Niti Vahan (NV) Theme.
*
*  Filename:         lane_detection.py
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
# Filename:         lane_detection.py
# Functions:        detect_lane, _marking_masks, _dash_points, _fit_dash_curve,
#                   _estimate_from_dashes, _runs, _yellow_boundary, _yellow_only
# Global variables: FRAME_W, FRAME_H, REF_ROW, VEHICLE_X, IMAGE_CENTRE_X, FIT_MIN_ROW,
#                   SLOPE_ROWS, WHITE_V_MIN, WHITE_S_MAX, DARK_V_MAX, LEFT_COEF,
#                   RIGHT_COEF_YELLOW, RIGHT_COEF_DASH, YELLOW_ONLY_LEFT,
#                   YELLOW_ONLY_RIGHT, _RING_KERNEL, _RING_RADIUS


####################### IMPORT MODULES #######################
import argparse
import json
import os

import cv2
import numpy as np
##############################################################

# The only three values "lane" is allowed to take.
LANE_LEFT = "left"
LANE_RIGHT = "right"
LANE_UNKNOWN = "unknown"
VALID_LANES = (LANE_LEFT, LANE_RIGHT, LANE_UNKNOWN)


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################

def detect_lane(frame):
    '''
    Purpose:
    ---
    Detect the lane in a single frame and report where the centre of the lane
    is, and which of the two lanes the vehicle is currently in.

    Input Arguments:
    ---
    `frame` :   [ numpy.ndarray ]
        A single BGR frame read from the video, of shape (height, width, 3).

    Returns:
    ---
    `result` :  [ dict ]
        {
            "center_x" : int,   x-pixel of the lane centre in this frame,
                                or -1 if the lane could not be found
            "lane"     : str,   "left", "right" or "unknown"
        }

    Example call:
    ---
    result = detect_lane(frame)

    COORDINATE SYSTEM:
    ---
    `center_x` is an absolute pixel column in the frame AS RECEIVED - the
    dataset's own resolution, 640x480. It is compared against a ground truth
    measured in those pixels, so it only means anything in them.

    You may resize, crop or warp all you like inside this function, but scale
    the answer back before returning it. A centre found in a 320x240 copy is
    half the value it should be, and a centre read off a bird's-eye view is in
    warped coordinates, not frame ones - map the point back through the inverse
    of your transform. Do not re-encode or resize the clip files themselves.

    NOTE:
    ---
    This function must ONLY compute and return the result.
    Do not call cv2.imshow(), cv2.waitKey(), cv2.imwrite() or print() from
    inside it. All visualisation and debugging output belongs outside this
    function - see draw_overlay() and process_video() below.
    '''

    center_x = -1
    lane = LANE_UNKNOWN

    height, width = frame.shape[:2]
    scale_x = width / float(FRAME_W)        # the answer is reported in the caller's pixels
    work = frame if (width, height) == (FRAME_W, FRAME_H) else cv2.resize(frame, (FRAME_W, FRAME_H))

    hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
    yellow_mask, white_mask = _marking_masks(hsv)
    dash_fit = _fit_dash_curve(_dash_points(white_mask, hsv))

    if dash_fit is not None:
        lane, cx = _estimate_from_dashes(dash_fit, yellow_mask)
        center_x = int(round(cx * scale_x))
    else:
        # no usable dashes: fall back on the solid yellow boundary alone
        side, x_yellow = _yellow_only(yellow_mask)
        if x_yellow is not None:
            lane = LANE_LEFT if side == "left" else LANE_RIGHT
            a, b = YELLOW_ONLY_LEFT if side == "left" else YELLOW_ONLY_RIGHT
            center_x = int(round((a * x_yellow + b) * scale_x))

    # keep a detected lane's centre inside the frame; "unknown" stays -1
    if center_x < 0 or center_x >= width:
        center_x = int(min(max(center_x, 0), width - 1)) if lane != LANE_UNKNOWN else -1

    return {"center_x": center_x, "lane": lane}


# ------------------------------------------------------------------
# Helpers and globals. Everything below works in 640 x 480 coordinates.
#
# How it works, in short:
#   1. Threshold white (dashes) and yellow (road edges) in HSV.
#   2. Keep white blobs that sit on dark tarmac and fit x = f(y) through
#      their centroids - this is the dashed divider, curved or straight.
#   3. Lane = which side of the vehicle's reference column the divider is
#      on, read at REF_ROW (the row the centre is also reported on).
#   4. Lane centre = a small linear model of the divider position, the
#      yellow edge on the same row and the divider's slope, fitted on the
#      public clips. If no dashes are visible, use the yellow edge alone.
# ------------------------------------------------------------------

FRAME_W, FRAME_H = 640, 480

REF_ROW = 305            # image row at which the lane centre is reported
VEHICLE_X = 278.0        # column at REF_ROW where the divider flips the lane label
IMAGE_CENTRE_X = 320.0   # middle of the frame, used only by the yellow-only fallback
FIT_MIN_ROW = 215        # dashes above this row are ignored by the curve fit
SLOPE_ROWS = (215, 430)  # rows used to measure the divider's local direction

WHITE_V_MIN = 160        # brightness a pixel needs to count as white paint
WHITE_S_MAX = 50         # saturation limit for white paint
DARK_V_MAX = 70          # brightness limit for "dark tarmac"
_RING_KERNEL = np.ones((9, 9), np.uint8)
_RING_RADIUS = _RING_KERNEL.shape[0] // 2

# Lane-centre models, x at REF_ROW in 640x480 px, least-squares fitted on the
# public clips and validated leave-one-clip-out.
LEFT_COEF = (0.9256, -132.21)                        # a*x_dash + b
RIGHT_COEF_YELLOW = (0.972, 0.2042, 45.87, 38.06)    # a*x_dash + b*x_yellow + c*slope + d
RIGHT_COEF_DASH = (0.6648, 0.5265, 102.62)           # a*x_dash + b*x_dash(row 215) + c
YELLOW_ONLY_LEFT = (0.7098, 208.35)                  # fallback: a*x_yellow + b
YELLOW_ONLY_RIGHT = (0.5836, 17.06)


def _marking_masks(hsv):
    """Binary masks of the yellow edge lines and the white paint."""
    yellow = cv2.inRange(hsv, (18, 100, 100), (38, 255, 255))
    white = cv2.inRange(hsv, (0, 0, WHITE_V_MIN), (180, WHITE_S_MAX, 255))
    white[410:, 215:420] = 0        # the vehicle's own bonnet
    white[:150, :] = 0              # too far away to be useful
    return yellow, white


def _dash_points(white, hsv):
    """Centroids of the white blobs that sit on dark tarmac (the dashes)."""
    dark = hsv[:, :, 2] <= DARK_V_MAX
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(white, connectivity=8)
    points = []
    for i in range(1, count):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < 15 or area > 4000:
            continue
        # work on the blob's bounding box (plus the ring's reach), not the whole frame
        x0 = max(stats[i, cv2.CC_STAT_LEFT] - _RING_RADIUS, 0)
        y0 = max(stats[i, cv2.CC_STAT_TOP] - _RING_RADIUS, 0)
        x1 = min(stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] + _RING_RADIUS, FRAME_W)
        y1 = min(stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] + _RING_RADIUS, FRAME_H)
        blob = (labels[y0:y1, x0:x1] == i).astype(np.uint8)
        ring = (cv2.dilate(blob, _RING_KERNEL) > 0) & (blob == 0)
        if dark[y0:y1, x0:x1][ring].mean() < 0.6:     # buildings, bonnet, signs ...
            continue
        points.append((centroids[i][0], centroids[i][1]))
    return points


def _fit_dash_curve(points):
    """x = f(y) through the nearer dash centroids, rejecting outliers.

    Only dashes in the lower part of the frame are used: far away the road
    bends sideways and x is no longer a smooth function of y.
    """
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
    if keep.sum() < degree + 1:
        return None
    return np.polyfit(ys[keep], xs[keep], degree)


def _estimate_from_dashes(dash_fit, yellow_mask):
    """Lane label and lane-centre x (640x480 px) from the fitted divider."""
    near_row, far_row = SLOPE_ROWS[1], SLOPE_ROWS[0]
    x_dash = float(np.polyval(dash_fit, REF_ROW))
    x_far = float(np.polyval(dash_fit, far_row))
    slope = (x_far - float(np.polyval(dash_fit, near_row))) / (near_row - far_row)

    # divider to the right of the vehicle's reference column -> vehicle is in the left lane
    lane = LANE_LEFT if x_dash > VEHICLE_X else LANE_RIGHT
    if lane == LANE_LEFT:
        a, b = LEFT_COEF
        return lane, a * x_dash + b

    x_yellow = _yellow_boundary(yellow_mask, REF_ROW, x_dash, "right")
    if x_yellow is not None:
        a, b, c, d = RIGHT_COEF_YELLOW
        return lane, a * x_dash + b * x_yellow + c * slope + d
    a, b, c = RIGHT_COEF_DASH
    return lane, a * x_dash + b * x_far + c


def _runs(mask_row):
    """(first, last) column of each horizontal run of set pixels in one mask row."""
    xs = np.where(mask_row > 0)[0]
    if len(xs) == 0:
        return []
    return [(r[0], r[-1]) for r in np.split(xs, np.where(np.diff(xs) > 3)[0] + 1)]


def _yellow_boundary(yellow, row, x_dash, side):
    """Centre of the yellow run nearest the dashes, on the given side of them."""
    best = None
    for lo, hi in _runs(yellow[row]):
        if hi - lo < 2:
            continue
        cx = 0.5 * (lo + hi)
        if side == "right" and cx > x_dash and (best is None or cx < best):
            best = cx
        if side == "left" and cx < x_dash and (best is None or cx > best):
            best = cx
    return best


def _yellow_only(yellow):
    """Fallback: the thickest yellow run in the lower half decides side and position."""
    best = None
    for row in (400, 380, 360, 340, 320, REF_ROW):
        for lo, hi in _runs(yellow[row]):
            if best is None or (hi - lo) > best[2]:
                best = (0.5 * (lo + hi), row, hi - lo)
        if best is not None and best[2] > 12:
            break
    if best is None:
        return None, None
    side = "left" if best[0] < IMAGE_CENTRE_X else "right"
    # refine the position on REF_ROW, using the run closest to the one found
    wide = [r for r in _runs(yellow[REF_ROW]) if r[1] - r[0] >= 2]
    if wide:
        cand = min(wide, key=lambda r: abs(0.5 * (r[0] + r[1]) - best[0]))
        return side, 0.5 * (cand[0] + cand[1])
    return side, best[0]


##############################################################
################ END OF YOUR IMPLEMENTATION ##################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################

def validate_result(result, frame_index):
    '''
    Purpose:
    ---
    Check that detect_lane() returned the expected structure and normalise it,
    so that a malformed return is reported here instead of silently scoring
    zero during evaluation.

    Input Arguments:
    ---
    `result` :          [ object ]      whatever detect_lane() returned
    `frame_index` :     [ int ]         index of the frame, used in error messages

    Returns:
    ---
    `clean` :           [ dict ]        {"center_x": int, "lane": str}
    '''
    where = "detect_lane() on frame {}".format(frame_index)

    if not isinstance(result, dict):
        raise TypeError("{} must return a dict, got {}".format(where, type(result).__name__))

    missing = {"center_x", "lane"} - set(result.keys())
    if missing:
        raise ValueError("{} is missing the key(s): {}".format(where, ", ".join(sorted(missing))))

    center_x = result["center_x"]
    if isinstance(center_x, bool) or not isinstance(center_x, (int, float, np.integer, np.floating)):
        raise TypeError("{} returned center_x of type {}, expected a number".format(
            where, type(center_x).__name__))
    center_x = int(round(float(center_x)))

    lane = result["lane"]
    if not isinstance(lane, str):
        raise TypeError("{} returned lane of type {}, expected a string".format(
            where, type(lane).__name__))
    lane = lane.strip().lower()
    if lane not in VALID_LANES:
        raise ValueError("{} returned lane = '{}', expected one of {}".format(
            where, result["lane"], ", ".join(VALID_LANES)))

    return {"center_x": center_x, "lane": lane}


def draw_overlay(frame, result):
    '''
    Purpose:
    ---
    Draw the detected lane centre and lane label on a copy of the frame.
    This is where display code belongs - never inside detect_lane().

    Input Arguments:
    ---
    `frame` :   [ numpy.ndarray ]   the frame that was passed to detect_lane()
    `result` :  [ dict ]            the validated result for that frame

    Returns:
    ---
    `canvas` :  [ numpy.ndarray ]   a copy of the frame with the overlay drawn
    '''
    canvas = frame.copy()
    height, width = canvas.shape[:2]

    # frame centre, for reference - roughly where the vehicle is pointing
    cv2.line(canvas, (width // 2, height), (width // 2, height - 40), (128, 128, 128), 1)

    center_x = result["center_x"]
    if 0 <= center_x < width:
        cv2.line(canvas, (center_x, height), (center_x, height // 2), (0, 0, 255), 2)
        cv2.circle(canvas, (center_x, height - 10), 5, (0, 0, 255), -1)

    label = "lane: {}   center_x: {}".format(result["lane"], center_x)
    cv2.putText(canvas, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return canvas


def process_video(video_path, show=False):
    '''
    Purpose:
    ---
    Read a video frame by frame, hand each frame to detect_lane() and collect
    the results.

    Input Arguments:
    ---
    `video_path` :  [ str ]     path to the video file
    `show` :        [ bool ]    if True, display the overlay while processing

    Returns:
    ---
    `results` :     [ list ]    one dict per frame:
                                {"frame": int, "center_x": int, "lane": str}
    '''
    if not os.path.isfile(video_path):
        raise FileNotFoundError("no such video file: {}".format(video_path))

    capture = cv2.VideoCapture(video_path)
    if not capture.isOpened():
        raise IOError("OpenCV could not open the video: {}".format(video_path))

    window = "Task 1C - {}".format(os.path.basename(video_path))
    results = []
    frame_index = 0

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break

            # A copy is passed in, so anything drawn inside detect_lane() cannot
            # corrupt the frame used for display.
            result = validate_result(detect_lane(frame.copy()), frame_index)
            results.append({"frame": frame_index, **result})

            if show:
                cv2.imshow(window, draw_overlay(frame, result))
                if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                    break

            frame_index += 1
    finally:
        capture.release()
        if show:
            cv2.destroyAllWindows()

    if not results:
        raise IOError("no frames could be read from: {}".format(video_path))

    return results


def summarise(video_path, results):
    '''
    Purpose:
    ---
    Print a one-line-per-video summary, so you can see at a glance whether the
    detector is returning anything sensible.

    Input Arguments:
    ---
    `video_path` :  [ str ]     path to the video that was processed
    `results` :     [ list ]    output of process_video()

    Returns:
    ---
    None
    '''
    total = len(results)
    counts = {lane: 0 for lane in VALID_LANES}
    for entry in results:
        counts[entry["lane"]] += 1
    not_found = sum(1 for entry in results if entry["center_x"] < 0)

    print("{:<28} {:>5} frames | left {:>5} | right {:>5} | unknown {:>5} | no centre {:>5}".format(
        os.path.basename(video_path), total,
        counts[LANE_LEFT], counts[LANE_RIGHT], counts[LANE_UNKNOWN], not_found))


def expand_videos(paths):
    '''
    Purpose:
    ---
    Turn the command-line arguments into a list of video files, accepting a
    FOLDER as well as individual files.

    A folder is the portable way to say "all the clips": Windows shells do not
    expand `public/*.mp4` the way bash does - cmd and PowerShell hand the
    pattern through verbatim and the script would look for a file literally
    named "*.mp4". `python lane_detection.py public` behaves the same on every
    platform.

    Input Arguments:
    ---
    `paths` :   [ list ]    the raw command-line arguments

    Returns:
    ---
    `videos` :  [ list ]    paths to individual video files, folders expanded
    '''
    videos = []
    for raw in paths:
        if os.path.isdir(raw):
            found = sorted(f for f in os.listdir(raw) if f.lower().endswith(".mp4"))
            if not found:
                raise FileNotFoundError("no .mp4 files in the folder: {}".format(raw))
            videos.extend(os.path.join(raw, f) for f in found)
        else:
            videos.append(raw)
    return videos


def main():
    parser = argparse.ArgumentParser(
        description="Task 1C - run your lane detector over one or more videos.")
    parser.add_argument("videos", nargs="+",
                        help="video file(s), or a folder holding them "
                             "(e.g. 'public')")
    parser.add_argument("--show", action="store_true",
                        help="display the detection overlay while processing (press q to stop)")
    parser.add_argument("--out", metavar="FILE",
                        help="write the per-frame results to this JSON file")
    args = parser.parse_args()

    all_results = {}
    for video_path in expand_videos(args.videos):
        results = process_video(video_path, show=args.show)
        summarise(video_path, results)
        all_results[os.path.basename(video_path)] = results

    if args.out:
        with open(args.out, "w", encoding="utf-8") as handle:
            json.dump(all_results, handle, indent=2)
        print("\nresults written to {}".format(args.out))


if __name__ == "__main__":
    main()