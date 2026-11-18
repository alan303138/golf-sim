"""Find the nominal camera toe-in (and, optionally, pitch / lens height) that keeps the
ball at rest AND the longest stretch of flight inside BOTH camera fields of view.

    python3 optimize_fov.py            # search the two toe-in angles for the current parameters
    python3 optimize_fov.py --all      # also scan pitch (15-25 deg) and lens height (200-300 mm)

Pure geometry (pinhole model, 8 mm lens on the sensor given in params.py).  Copy the printed
TOE_IN_R / TOE_IN_L (and PITCH_NOM / LENS_HEIGHT) into params.py and run build.py again.
"""
import math
import sys
import numpy as np

import params as P
from lib import rot_x, rot_z

HU = math.atan(P.SENSOR_W / 2 / P.FOCAL)
HV = math.atan(P.SENSOR_H / 2 / P.FOCAL)
MARGIN = math.radians(P.FOV_MARGIN_DEG)


def cams_world(yaw_r, yaw_l, theta, height, dist=None, aim_u=None):
    """World pose (pupil, rotation) of both cameras; the mean pupil sits at
    (aim_u, -dist, height) and the head is pitched down by theta about it."""
    dist = P.BALL_DIST if dist is None else dist
    aim_u = P.AIM_U if aim_u is None else aim_u
    pb = []
    for s, yaw in ((1, yaw_r), (-1, -yaw_l)):
        Rz = rot_z(yaw)
        # pupil exactly at x = s * baseline / 2 (see pose.cam_origin)
        p = np.array([s * P.BASELINE / 2, P.CAM_Y0 + P.PUPIL_K * math.cos(math.radians(yaw)), P.CAM_AXIS_Z])
        pb.append((p, Rz))
    mean = (pb[0][0] + pb[1][0]) / 2
    Rp = rot_x(-theta)
    return [(np.array([aim_u, -dist, height]) + Rp @ (p - mean), Rp @ Rz) for p, Rz in pb]


def coverage(cams, launch_deg, step=2.0):
    """Down-range distance (mm) up to which the ball stays fully inside both fields of view."""
    best = -1.0
    for dx in np.arange(0, 500, step):
        pt = np.array([dx, 0, P.BALL_R + dx * math.tan(math.radians(launch_deg))])
        for pw, R in cams:
            v = R.T @ (pt - pw)
            r = math.atan(P.BALL_R / np.linalg.norm(v))
            if abs(math.atan2(v[0], v[1])) > HU - MARGIN - r or abs(math.atan2(v[2], v[1])) > HV - MARGIN - r:
                return best
        best = dx
    return best


def best_yaws(theta, height, yaws=np.arange(0.0, 20.1, 0.5)):
    res = []
    for yr in yaws:
        for yl in yaws:
            c = cams_world(yr, yl, theta, height)
            c0, c30, c45 = coverage(c, 0), coverage(c, 30), coverage(c, 45)
            res.append((min(c0, c30), c45, c0, c30, float(yr), float(yl)))
    res.sort(reverse=True)
    return res[0]


if __name__ == "__main__":
    if "--json" in sys.argv:
        import json
        b = best_yaws(P.PITCH_NOM, P.LENS_HEIGHT, np.arange(0.0, 20.1, 1.0))
        print(json.dumps({"TOE_IN_R": b[4], "TOE_IN_L": b[5], "cover_0_30_45": [b[2], b[3], b[1]]}))
        sys.exit(0)
    if "--all" in sys.argv:
        rows = []
        for th in (15, 17, 20, 25):
            for h in (200, 225, 250, 255, 275, 300):
                b = best_yaws(th, h, np.arange(0.0, 20.1, 1.0))
                rows.append((th, h) + b)
        for r in rows:
            print("pitch %2d  height %3d : covered at launch 0 deg %5.0f mm, 30 deg %5.0f mm, 45 deg %5.0f mm  (yaw R %.1f / L %.1f)"
                  % (r[0], r[1], r[4], r[5], r[3], r[6], r[7]))
    else:
        b = best_yaws(P.PITCH_NOM, P.LENS_HEIGHT)
        print("pitch %.1f deg, lens height %.0f mm, baseline %.0f mm" % (P.PITCH_NOM, P.LENS_HEIGHT, P.BASELINE))
        print("best toe-in:  TOE_IN_R = %.1f   TOE_IN_L = %.1f" % (b[4], b[5]))
        print("down-range coverage with both cameras: launch 0 deg %.0f mm, 30 deg %.0f mm, 45 deg %.0f mm" % (b[2], b[3], b[1]))
        c = cams_world(P.TOE_IN_R, P.TOE_IN_L, P.PITCH_NOM, P.LENS_HEIGHT)
        print("current params.py values: 0 deg %.0f mm, 30 deg %.0f mm, 45 deg %.0f mm"
              % (coverage(c, 0), coverage(c, 30), coverage(c, 45)))
