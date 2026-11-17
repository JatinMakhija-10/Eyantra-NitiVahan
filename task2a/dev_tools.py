"""Development helpers for Task 2A. NOT part of the submission.
Keep it in the same folder as task2a.py (it imports your functions from it).

  python dev_tools.py grab [every_n]    drive with YOUR controller and save every n-th frame to frames/
  python dev_tools.py bev frame.png     write bev.png: the bird's-eye view of one frame (lines should be vertical + parallel)
  python dev_tools.py test frames/      run detect_lane on saved frames, write overlays to overlays/
  python dev_tools.py stop [seconds]    raise NV_stop for N seconds (default 5), then lower it
"""
import glob, math, os, sys, time
import cv2
import numpy as np
import task2a as T


def grab(every=15):
    os.makedirs('frames', exist_ok=True)
    sim, h = T.connect_and_locate()
    while sim.getSimulationState() != sim.simulation_advancing_running:
        time.sleep(0.05)
    n = saved = 0
    try:
        while sim.getSimulationState() == sim.simulation_advancing_running:
            f = T.read_frame(sim, h)
            if f is None:
                continue
            n += 1
            stop = bool(sim.getInt32Signal(T.STOP_SIGNAL))
            det = T.detect_lane(f)
            L, R, sp = T.compute_control(det, stop)
            T.drive(sim, h, L, R, sp)
            if n % every == 0:
                cv2.imwrite(f'frames/f_{n:05d}.png', f)
                saved += 1
    finally:
        try:
            sim.setJointTargetVelocity(h['motor_left'], 0.0)
            sim.setJointTargetVelocity(h['motor_right'], 0.0)
        except Exception:
            pass
    print(f'{n} frames, {saved} saved to frames/')


def bev(path):
    img = cv2.imread(path)
    M = T._get_matrix(img.shape[0], img.shape[1])
    out = cv2.warpPerspective(img, M, (T.BEV_W, T.BEV_H))
    cv2.line(out, (int(T.CAR_X), 0), (int(T.CAR_X), T.BEV_H), (0, 255, 0), 1)
    cv2.imwrite('bev.png', out)
    print('wrote bev.png - yellow and dashed lines should be vertical and parallel, dashes equal length')


def test(folder):
    os.makedirs('overlays', exist_ok=True)
    for p in sorted(glob.glob(os.path.join(folder, '*.png'))):
        f = cv2.imread(p)
        T._state['have'] = False
        d = T.detect_lane(f)
        bev = cv2.cvtColor(T._dbg['bev'], cv2.COLOR_GRAY2BGR)
        for s, x in zip(T._dbg['pts_s'], T._dbg['pts_x']):
            cv2.circle(bev, (int(x), int(T.BEV_H - s)), 4, (0, 0, 255), -1)
        cv2.line(bev, (int(T.CAR_X), 0), (int(T.CAR_X), T.BEV_H), (0, 255, 0), 1)
        txt = 'None' if d is None else 'y=%+.3f ahead=%.2f n=%d %s' % (d['y'], d['ahead'], d['n'], d['lane'])
        print(os.path.basename(p), txt)
        side = cv2.resize(f, (bev.shape[1] * f.shape[0] // bev.shape[0], bev.shape[0]))
        out = np.hstack([side, bev]); cv2.putText(out, txt, (5, 18), 0, 0.5, (0, 255, 255), 1)
        cv2.imwrite(os.path.join('overlays', os.path.basename(p)), out)


def stop(secs=5.0):
    sim, _ = T.connect_and_locate()
    sim.setInt32Signal(T.STOP_SIGNAL, 1); print('NV_stop = 1')
    time.sleep(secs)
    sim.setInt32Signal(T.STOP_SIGNAL, 0); print('NV_stop = 0')


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a: print(__doc__)
    elif a[0] == 'grab': grab(int(a[1]) if len(a) > 1 else 15)
    elif a[0] == 'bev': bev(a[1])
    elif a[0] == 'test': test(a[1])
    elif a[0] == 'stop': stop(float(a[1]) if len(a) > 1 else 5.0)
    else: print(__doc__)
