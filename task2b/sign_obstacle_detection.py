'''
*****************************************************************************************
*
*  ===============================================
*     Niti Vahan (NV) Theme of eYRC 2026-27
*  ===============================================
*
*  This script is intended for implementation of Task 2B of Niti Vahan (NV) Theme.
*
*  Filename:         sign_obstacle_detection.py
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
# Filename:         sign_obstacle_detection.py
# Functions:        detect_signs, detect_obstacle, _red_mask, _blue_mask, _touches_border, _red_signs_raw,
#                   _red_signs, _find_obstacle, _order_quad, _face_quad, _probe, _classify_letter,
#                   _blue_signs, _letter_label
# Global variables: MIN_SIGN_AREA, MIN_OBS_AREA, OBS_MIN_FRAME_FRAC, RING_HOLE_FRAC, CIRCLE_FILL_MIN, OBS_MIN_SOLIDITY, OBS_MIN_EXTENT, SQ


####################### IMPORT MODULES #######################
import cv2
import numpy as np
##############################################################

# The only sign labels detect_signs() is allowed to return.
SIGN_LABELS = ("School", "Speed40", "Parking", "Hospital")


##############################################################
############### ADD YOUR IMPLEMENTATION HERE #################
##############################################################

def detect_signs(image):
    '''
    Purpose:
    ---
    Detect every road sign visible in a single image, and report what each
    one is and how large it appears. An image can hold zero, one, or more
    than one sign at once.

    Input Arguments:
    ---
    `image` :   [ numpy.ndarray ]
        A single BGR image, of shape (height, width, 3).

    Returns:
    ---
    `detections` :  [ list of dict ]
        Zero, one, or many entries - one per sign found in the image:
        [
            {
                "label" : str,   one of SIGN_LABELS
                "area"  : int,   that sign's area in pixels
            },
            ...
        ]

    Example call:
    ---
    detections = detect_signs(image)

    NOTE:
    ---
    This function must ONLY compute and return the result. Do not call
    cv2.imshow(), cv2.waitKey(), cv2.imwrite() or print() from inside it -
    reading images and reporting results is the evaluator's job, not this
    function's.
    '''

    detections = []

    #################### ADD YOUR CODE HERE ####################
    # 1-2. red signs (School / Speed40): red ring with a white interior
    for sign in _red_signs(image):
        detections.append({"label": sign[0], "area": sign[1]})
    # 1-4. blue signs (Parking / Hospital): blue face, letter told apart
    #      after rectifying the face to a square
    for sign in _blue_signs(image):
        detections.append({"label": sign[0], "area": sign[1]})
    ############################################################

    return detections


def detect_obstacle(image):
    '''
    Purpose:
    ---
    Detect whether the obstacle is visible in a single image, and how large
    it appears.

    Input Arguments:
    ---
    `image` :   [ numpy.ndarray ]
        A single BGR image, of shape (height, width, 3).

    Returns:
    ---
    `result` :  [ dict ]
        {
            "present" : bool,   True if the obstacle is visible
            "area"    : int,    its area in pixels, 0 if not present
        }

    Example call:
    ---
    result = detect_obstacle(image)

    NOTE:
    ---
    This function must ONLY compute and return the result. Do not call
    cv2.imshow(), cv2.waitKey(), cv2.imwrite() or print() from inside it -
    reading images and reporting results is the evaluator's job, not this
    function's.
    '''

    present = False
    area = 0

    #################### ADD YOUR CODE HERE ####################
    # 1. red mask minus the red signs (solid red, no white interior)
    # 2. merge split shading bands, keep the biggest solid, compact blob
    # 3-4. present + area
    best = _find_obstacle(image)
    if best is not None:
        present = True
        area = int(best)
    ############################################################

    return {"present": present, "area": area}


# ------------------------------------------------------------------
# Add any helper functions and global variables you need below this
# comment, and keep them ABOVE the "END OF YOUR IMPLEMENTATION" line.
# They must be called from detect_signs() or detect_obstacle() - the
# evaluation script only ever calls those two functions. List them in
# the file header too.
# ------------------------------------------------------------------

# ---- tunables (globals) ----
MIN_SIGN_AREA = 300          # px; lane-overlay dots are < 200 px
MIN_OBS_AREA = 300
OBS_MIN_FRAME_FRAC = 0.0006   # also scales with frame size (overlay dots ~0.03%)
RING_HOLE_FRAC = 0.15        # sign interior (white) / outer area
CIRCLE_FILL_MIN = 0.65       # blob area / min-enclosing-circle area: circle ~1, triangle ~0.41
OBS_MIN_SOLIDITY = 0.85
OBS_MIN_EXTENT = 0.65


def _red_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    m = (((h <= 10) | (h >= 170)) & (s >= 100) & (v >= 60)).astype(np.uint8) * 255
    return m


def _blue_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    return ((h >= 100) & (h <= 130) & (s >= 120) & (v >= 60)).astype(np.uint8) * 255


def _touches_border(x, y, w, h, shape):
    return x <= 0 or y <= 0 or x + w >= shape[1] or y + h >= shape[0]


def _red_signs_raw(image):
    '''Returns list of (label, area, filled_contour) for red signs.'''
    out = []
    m = cv2.morphologyEx(_red_mask(image), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    cnts, hier = cv2.findContours(m, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return out
    hier = hier[0]
    for i, c in enumerate(cnts):
        if hier[i][3] != -1:
            continue
        area = cv2.contourArea(c)
        if area < MIN_SIGN_AREA:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if _touches_border(x, y, w, h, image.shape):
            continue
        hole = sum(cv2.contourArea(cc) for j, cc in enumerate(cnts) if hier[j][3] == i)
        if hole / area < RING_HOLE_FRAC:
            continue                                   # solid red -> not a sign
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0 or area / hull_area < 0.85:
            continue
        (_, _), r = cv2.minEnclosingCircle(c)
        fill = area / (np.pi * r * r + 1e-6)
        label = "Speed40" if fill >= CIRCLE_FILL_MIN else "School"
        out.append((label, int(round(area)), c))
    return out


def _red_signs(image):
    return [(l, a) for (l, a, _) in _red_signs_raw(image)]


def _find_obstacle(image):
    m = _red_mask(image)
    # remove red-sign regions first so an adjacent sign cannot be merged in
    for (_, _, c) in _red_signs_raw(image):
        cv2.drawContours(m, [c], -1, 0, -1)
        cv2.drawContours(m, [c], -1, 0, 5)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = max(MIN_OBS_AREA, OBS_MIN_FRAME_FRAC * image.shape[0] * image.shape[1])
    best = None
    for c in cnts:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0:
            continue
        if area / hull_area < OBS_MIN_SOLIDITY or area / float(w * h) < OBS_MIN_EXTENT:
            continue
        if not (0.4 <= w / float(h) <= 3.0):
            continue
        if best is None or area > best:
            best = area
    return best


def _order_quad(pts):
    pts = np.array(pts, dtype=np.float32).reshape(-1, 2)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()
    return np.array([pts[np.argmin(s)], pts[np.argmin(d)], pts[np.argmax(s)], pts[np.argmax(d)]],
                    dtype=np.float32)          # tl, tr, br, bl


def _face_quad(hull):
    peri = cv2.arcLength(hull, True)
    for eps in (0.02, 0.03, 0.04, 0.06, 0.08, 0.1, 0.13):
        ap = cv2.approxPolyDP(hull, eps * peri, True)
        if len(ap) == 4:
            return _order_quad(ap)
    return _order_quad(cv2.boxPoints(cv2.minAreaRect(hull)))


SQ = 48


def _probe(nb, r0, r1, c0, c1):
    reg = nb[int(r0 * SQ):int(r1 * SQ), int(c0 * SQ):int(c1 * SQ)]
    return float(reg.mean()) / 255.0 if reg.size else 0.0


def _classify_letter(image, quad):
    '''Rectify the blue face to SQ x SQ; returns ("Parking"/"Hospital"/None, debug).'''
    dst = np.array([[0, 0], [SQ - 1, 0], [SQ - 1, SQ - 1], [0, SQ - 1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(quad, dst)
    face = cv2.warpPerspective(image, M, (SQ, SQ), flags=cv2.INTER_LINEAR)
    blue = _blue_mask(face)
    nb = 255 - blue                                 # letter / frame band / anti-aliasing
    return face, nb


def _blue_signs(image):
    out = []
    bm = _blue_mask(image)
    grp = cv2.dilate(bm, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(grp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        if w * h < MIN_SIGN_AREA or _touches_border(x, y, w, h, image.shape):
            continue
        comp = np.zeros(bm.shape, np.uint8)
        cv2.drawContours(comp, [c], -1, 255, -1)
        pts = cv2.findNonZero(cv2.bitwise_and(bm, comp))
        if pts is None:
            continue
        hull = cv2.convexHull(pts)
        area = cv2.contourArea(hull)
        if area < MIN_SIGN_AREA:
            continue
        bx, by, bw, bh = cv2.boundingRect(hull)
        if not (0.7 <= bw / float(bh) <= 1.4) or area / float(bw * bh) < 0.6:
            continue
        # face must be solid blue + letter, not a grid of window panes
        quad = _face_quad(hull)
        face, nb = _classify_letter(image, quad)
        label = _letter_label(nb)
        if label is not None:
            out.append((label, int(round(area))))
    return out


def _letter_label(nb):
    # non-blue pixel fractions at fixed spots of the rectified face
    left_stem = _probe(nb, 0.30, 0.72, 0.26, 0.33)
    top_mid = _probe(nb, 0.21, 0.27, 0.45, 0.55)
    bot_right = _probe(nb, 0.60, 0.74, 0.67, 0.75)
    bot_mid = _probe(nb, 0.60, 0.74, 0.45, 0.55)
    if left_stem < 0.6:
        return None
    if bot_right >= 0.6 and top_mid <= 0.35 and bot_mid <= 0.35:
        return "Hospital"
    if bot_right <= 0.35 and top_mid >= 0.6:
        return "Parking"
    return None


##############################################################
################ END OF YOUR IMPLEMENTATION ##################
##############################################################


#################### DO NOT EDIT BELOW THIS LINE ####################

def validate_signs(detections, image_name):
    '''
    Purpose:
    ---
    Check that detect_signs() returned the expected structure and normalise
    it, so a malformed return is reported here instead of silently scoring
    zero during evaluation.

    Input Arguments:
    ---
    `detections` :  [ object ]     whatever detect_signs() returned
    `image_name` :  [ str ]        name of the image, used in error messages

    Returns:
    ---
    `clean` :       [ list of dict ]   [{"label": str, "area": int}, ...]
    '''
    where = "detect_signs() on {}".format(image_name)

    if not isinstance(detections, list):
        raise TypeError("{} must return a list, got {}".format(where, type(detections).__name__))

    clean = []
    for index, entry in enumerate(detections):
        entry_where = "{}, entry {}".format(where, index)

        if not isinstance(entry, dict):
            raise TypeError("{} must be a dict, got {}".format(entry_where, type(entry).__name__))

        missing = {"label", "area"} - set(entry.keys())
        if missing:
            raise ValueError("{} is missing the key(s): {}".format(entry_where, ", ".join(sorted(missing))))

        label = entry["label"]
        if label not in SIGN_LABELS:
            raise ValueError("{} returned label = '{}', expected one of {}".format(
                entry_where, label, ", ".join(SIGN_LABELS)))

        area = _validate_area(entry["area"], entry_where)
        clean.append({"label": label, "area": area})

    return clean


def validate_obstacle(result, image_name):
    '''
    Purpose:
    ---
    Check that detect_obstacle() returned the expected structure and
    normalise it.

    Input Arguments:
    ---
    `result` :      [ object ]     whatever detect_obstacle() returned
    `image_name` :  [ str ]        name of the image, used in error messages

    Returns:
    ---
    `clean` :       [ dict ]       {"present": bool, "area": int}
    '''
    where = "detect_obstacle() on {}".format(image_name)

    if not isinstance(result, dict):
        raise TypeError("{} must return a dict, got {}".format(where, type(result).__name__))

    missing = {"present", "area"} - set(result.keys())
    if missing:
        raise ValueError("{} is missing the key(s): {}".format(where, ", ".join(sorted(missing))))

    present = result["present"]
    if not isinstance(present, (bool, np.bool_)):
        raise TypeError("{} returned present of type {}, expected a bool".format(
            where, type(present).__name__))
    present = bool(present)

    area = _validate_area(result["area"], where)
    if not present and area != 0:
        raise ValueError("{} returned present=False but area={}, expected 0".format(where, area))

    return {"present": present, "area": area}


def _validate_area(area, where):
    if isinstance(area, bool) or not isinstance(area, (int, float, np.integer, np.floating)):
        raise TypeError("{} returned area of type {}, expected a number".format(
            where, type(area).__name__))
    area = int(round(float(area)))
    if area < 0:
        raise ValueError("{} returned a negative area: {}".format(where, area))
    return area
