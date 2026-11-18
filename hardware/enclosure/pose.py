"""Nominal-pose mathematics: frames BF / PF / HF / WF (see params.py)."""
import math
import numpy as np
import params as P
from lib import rot_x, rot_z, T4


def toed_in_yaw(side):
    """KP rotation (deg, CCW about +Z) of the camera stage on side +1 (x>0) / -1."""
    return P.TOE_IN_R if side > 0 else -P.TOE_IN_L


def pupil_bf(side):
    """Entrance pupil of the camera on `side`, in BF (pitch 0)."""
    kp = np.array([0.0, P.PUPIL_Y, P.Z_AX])
    return rot_z(toed_in_yaw(side)) @ kp + np.array([side * P.BASELINE / 2.0, 0.0, P.Z_CB_TOP])


PUPIL_MEAN = (pupil_bf(1) + pupil_bf(-1)) / 2.0
# BF origin sits at world x = X0 so that the mean pupil is AIM_U down-range of the ball
X0 = P.AIM_U - PUPIL_MEAN[0]
PIVOT_X = 0.0                                     # pitch axis (and mast) at the beam centre: head is balanced over the hub
PIVOT_BF = np.array([PIVOT_X, 0.0, P.Z_PIVOT])


def pitch_pupil_offset(theta_deg):
    """(dy, dz) of the mean pupil relative to the pitch axis once pitched down by theta."""
    d = PUPIL_MEAN - PIVOT_BF
    t = math.radians(theta_deg)
    return (d[1] * math.cos(t) + d[2] * math.sin(t), -d[1] * math.sin(t) + d[2] * math.cos(t))


_off = pitch_pupil_offset(P.PITCH_NOM)
PIVOT_W = np.array([X0 + PIVOT_X, -P.BALL_DIST - _off[0], P.LENS_HEIGHT - _off[1]])   # pitch axis in WF

HUB_Z0 = P.Z_PAD_TOP + P.FOOT_GAP_NOM            # hub / bracket bottom (WF z), nominal
MAST_TOP_W = PIVOT_W[2] + P.MC_FLANGE_BOTTOM_PF
MAST_BOTTOM_W = HUB_Z0 + 8.0
MAST_LEN = MAST_TOP_W - MAST_BOTTOM_W


def lens_height_for(theta_deg, foot_gap):
    """Lens height reached with the *cut* mast at another pitch / foot setting."""
    hub_z0 = P.Z_PAD_TOP + foot_gap
    pivot_z = hub_z0 + 8.0 + MAST_LEN - P.MC_FLANGE_BOTTOM_PF
    return pivot_z + pitch_pupil_offset(theta_deg)[1]


def bf2w_matrix(theta_deg=None):
    """4x4 transform BF -> WF for a pitch angle (deg, nose down)."""
    th = P.PITCH_NOM if theta_deg is None else theta_deg
    R = rot_x(-th)
    return T4(R, PIVOT_W - R @ PIVOT_BF)


def pf2w_matrix():
    """PF (pivot frame, origin on the pitch axis, never pitches) -> WF."""
    return T4(np.eye(3), PIVOT_W)


def hf2w_matrix(foot_gap=None):
    gap = P.FOOT_GAP_NOM if foot_gap is None else foot_gap
    return T4(np.eye(3), (PIVOT_W[0], PIVOT_W[1], P.Z_PAD_TOP + gap))


def ball_w():
    return np.array([0.0, 0.0, P.BALL_R])


def ball_bf(theta_deg=None):
    M = bf2w_matrix(theta_deg)
    return (np.linalg.inv(M) @ np.append(ball_w(), 1.0))[:3]


def shield_y0():
    """BF y of the shield sheet back face (clear of the retaining rings at any yaw setting)."""
    ymax = 0.0
    for side in (1, -1):
        for dyaw in (P.YAW_RANGE, -P.YAW_RANGE, 0.0):
            psi = math.radians(toed_in_yaw(side) + dyaw)
            for sx in (-1, 1):
                x, y = sx * P.FH_DISC_R, P.FR_FRONT_Y
                ymax = max(ymax, x * math.sin(psi) + y * math.cos(psi))
    return math.ceil(ymax + P.SHIELD_CLEAR)


def template_geometry():
    """Lug pin position, unit vector pin -> ball, distance, and the rotation (deg, CCW about +Z) of the
    template parts (their local +Y = direction of the stick, pin at their origin)."""
    lug = np.array([PIVOT_W[0], PIVOT_W[1] + P.LUG_Y])
    v = -lug
    D = float(np.linalg.norm(v))
    u = v / D
    rot = -math.degrees(math.atan2(u[0], u[1]))
    return lug, u, D, rot


def stick_len():
    return template_geometry()[2] - 66.0


# ---- LED side boards: stack along +Y in BF ------------------------------------
LED_FRAME_REAR_OFF = 11.0
LED_FRAME_T = 20.0
LED_PLATE_OFF = 5.0
POST_H = LED_FRAME_REAR_OFF
FLANGE_Y = 6.0                                    # printed backplate thickness


def led_y0():
    """BF y of the front face of the printed LED backplate (frame front stays behind the shield posts)."""
    return shield_y0() - 5.0 - P.LED_FRAME_FRONT_CLEAR - (LED_FRAME_REAR_OFF + LED_FRAME_T)


def led_tip_y():
    """BF y of the top of the TIR lenses."""
    return led_y0() + LED_PLATE_OFF + P.LED_PLATE[2] + P.LED_STAR_T + P.LED_LENS_H


def led_star_xz(i, j, side):
    """Star centre (x, z) in BF of grid position (i, j) of the board on `side`."""
    gx, gz = P.LED_GRID
    return (side * P.LED_X + (i - (gx - 1) / 2.0) * P.LED_STAR_D, P.LED_ZC + (j - (gz - 1) / 2.0) * P.LED_STAR_D)
