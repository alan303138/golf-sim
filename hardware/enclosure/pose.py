"""Frames and the nominal pose: BF (beam), K (camera), WF (world)."""
import math
import numpy as np
import params as P
from lib import rot_x, rot_z, T4


def yaw(side):
    """Built-in camera yaw (deg, CCW about +Z): +X camera turns towards -X, -X camera towards +X."""
    return P.TOE_IN_R if side > 0 else -P.TOE_IN_L


def cam_origin(side):
    """BF position of the camera frame origin so that the lens pupil sits exactly at x = side * baseline / 2."""
    psi = math.radians(yaw(side))
    x0 = side * P.BASELINE / 2.0 + P.PUPIL_K * math.sin(psi)
    return np.array([x0, P.CAM_Y0, P.CAM_AXIS_Z])


def k2bf(side):
    return T4(rot_z(yaw(side)), cam_origin(side))


def pupil_bf(side):
    return (k2bf(side) @ np.array([0, P.PUPIL_K, 0, 1.0]))[:3]


PUPIL_MEAN = (pupil_bf(1) + pupil_bf(-1)) / 2.0
X0 = P.AIM_U - PUPIL_MEAN[0]          # world x of the BF origin (PUPIL_MEAN[0] is 0)
M0 = np.array([P.AIM_U, -P.BALL_DIST, P.LENS_HEIGHT])   # world position of the mean pupil


def bf2w_matrix(theta=None):
    """BF -> WF: pitch nose-down by theta about the mean pupil (the tripod head tilts the whole beam)."""
    th = P.PITCH_NOM if theta is None else theta
    R = rot_x(-th)
    return T4(R, M0 - R @ PUPIL_MEAN)


def ball_w():
    return np.array([0.0, 0.0, P.BALL_R])


def hood_front_y():
    """Largest BF y of any camera hood point (front face corners)."""
    ymax = 0.0
    for side in (1, -1):
        M = k2bf(side)
        for sx in (-1, 1):
            p = M @ np.array([sx * P.HOOD_HALF, P.HOOD_FRONT_K, 0, 1.0])
            ymax = max(ymax, p[1])
    return ymax


def shield_y0():
    """BF y of the shield sheet back face."""
    return math.ceil(hood_front_y() + P.SHIELD_CLEAR)


# ---- LED board stack along +Y (BF) -------------------------------------------------
def led_tip_y():
    """BF y of the top of the TIR lenses: just behind the hood front plane."""
    return math.floor(hood_front_y() - 3.0)


def led_geom():
    """Key BF y values of one LED tray."""
    tip = led_tip_y()
    stars_y0 = tip - P.LED_LENS_H - P.LED_STAR_T         # front face of the plate
    plate_y1 = stars_y0
    plate_y0 = plate_y1 - P.LED_PLATE[2]                 # plate rear face
    lip0 = plate_y0 - 8.0
    parts_front = plate_y0 - 1.0
    pcb_front = parts_front - P.MOD_PARTS_H
    pcb_rear = pcb_front - P.MOD[2]
    return dict(tip=tip, stars_y0=stars_y0, plate_y0=plate_y0, plate_y1=plate_y1, lip0=lip0,
                parts_front=parts_front, pcb_front=pcb_front, pcb_rear=pcb_rear,
                y_end=tip + 8.0, slit_y=tip + 2.0)


def led_star_xz(i, j, side):
    gx, gz = P.LED_GRID
    return (side * P.LED_X + (i - (gx - 1) / 2.0) * P.LED_STAR_D, P.LED_ZC + (j - (gz - 1) / 2.0) * P.LED_STAR_D)


def tripod_point_w(theta=None):
    """World position of the top of the tripod stud (adapter underside) for the given pitch."""
    S = np.array([P.TA_X, P.TA_Y, P.Z_BEAM_BOT - P.TA_T, 1.0])
    return (bf2w_matrix(theta) @ S)[:3]
