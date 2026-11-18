"""LED side boards (one per side, mirrored): arm bracket, backplate, module plate, diffuser frame
with baffle wing, ARMED LED holder.  Everything is modelled directly in BF (beam frame)."""
import math
import numpy as np
import params as P
import pose
from lib import *

ARM_Z0, ARM_Z1 = -31.0, -5.0           # arm bar height range (BF z)
ARM_W = 14.0
BP_X, BP_Z = 67.0, 47.0                # backplate / frame half sizes
POST_XZ = (61.0, 41.0)
PLATE_BOSS = (50.0, 40.0)
MODBOLT = (25.0, 25.0)
FL_HOLES_Z = (-26.0, -10.0)            # arm flange screws (BF z)


def _tag(side):
    return "R" if side > 0 else "L"


def led_arm(side):
    """LE-1R/L: clamp on the beam front slot (2 x M5) + bar to the backplate, flange with 2 M3."""
    set_up((0, 1, 0))
    xa = side * (P.LED_X + P.LE_ARM_X)
    y0 = P.Y_BEAM_FRONT
    yb = pose.led_y0() - pose.FLANGE_Y          # rear face of the backplate
    wall = box(xa - 14.5, xa + 14.5, y0, y0 + 6.0, ARM_Z0, 9.0)
    bar = box(xa - ARM_W / 2, xa + ARM_W / 2, y0 + 6.0, yb - 6.0, ARM_Z0, ARM_Z1)
    flange = box(xa - 10.0, xa + 10.0, yb - 6.0, yb, ARM_Z0, ARM_Z1)
    part = union(wall, bar, flange)
    cut = []
    for dx in (-7.5, 7.5):
        cut.append(cbore((xa + dx, y0 + 6.0, 0), (0, -1, 0), P.M5_CLEAR, 7.0, P.M5_HEAD_D, P.M5_HEAD_H))
    for z in FL_HOLES_Z:
        cut.append(cbore((xa, yb - 6.0, z), (0, 1, 0), P.M3_CLEAR, 7.0, P.M3_BTN_D, P.M3_HEAD_H, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("LE-1" + _tag(side), (xa, y0 + 6.0, 5.0), (0, 1, 0), (1, 0, 0), size=4.0))
    return Part("LE-1" + _tag(side), part, (0, -1, 0), "BF", 1, "LED board arm (beam front slot)")


def led_backplate():
    """LP-2: printed backplate for the aluminium LED plate (front bosses), frame posts, arm flange
    nuts, module plate nuts, wire windows.  Local frame: BF axes, centred on x = 0, z = LED_ZC, y = front face."""
    set_up((0, 1, 0))
    y1 = pose.led_y0()
    zc = P.LED_ZC
    plate = box(-BP_X, BP_X, y1 - pose.FLANGE_Y, y1, zc - BP_Z, zc + BP_Z)
    hx = P.LED_PLATE[0] / 2 - P.LED_PLATE_HOLE_INSET
    hz = P.LED_PLATE[1] / 2 - P.LED_PLATE_HOLE_INSET
    bosses = [cyl((sx, y1 - 0.01, zc + sz), (0, 1, 0), 4.5, pose.LED_PLATE_OFF) for sx in (-hx, hx) for sz in (-hz, hz)]
    part = union(plate, *bosses)
    cut = []
    for sx in (-hx, hx):
        for sz in (-hz, hz):
            cut.append(hole((sx, y1 + pose.LED_PLATE_OFF, zc + sz), (0, -1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    px, pz = POST_XZ
    for sx in (-px, px):
        for sz in (-pz, pz):
            cut.append(cbore((sx, y1 - pose.FLANGE_Y, zc + sz), (0, 1, 0), P.M3_CLEAR, pose.FLANGE_Y + 1, P.M3_BTN_D, P.M3_HEAD_H, tear=False))
    for sx in (-MODBOLT[0], MODBOLT[0]):
        for sz in (-MODBOLT[1], MODBOLT[1]):
            cut.append(hole((sx, y1 - pose.FLANGE_Y, zc + sz), (0, 1, 0), P.M3_CLEAR, pose.FLANGE_Y + 1, tear=False))
            cut.append(hexpocket((sx, y1 + 0.01, zc + sz), (0, -1, 0), 5.5 + 0.4, 2.7, rot=30.0))
    for sx in (-P.LE_ARM_X, P.LE_ARM_X):
        for z in FL_HOLES_Z:
            cut.append(hole((sx, y1 - pose.FLANGE_Y, z), (0, 1, 0), P.M3_CLEAR, pose.FLANGE_Y + 1, tear=False))
            cut.append(hexpocket((sx, y1 + 0.01, z), (0, -1, 0), 5.5 + 0.4, 2.7, rot=30.0))
    for sx in (-12.0, 12.0):                         # wire windows
        cut.append(box(sx - 8, sx + 8, y1 - pose.FLANGE_Y - 1, y1 + 1, zc - 3, zc + 3))
    part = cut_all(part, cut)
    part = part.union(emboss("LP-2", (0, y1, zc + 43.0), (0, 1, 0), (1, 0, 0), size=5.0))
    return Part("LP-2", part, (0, 1, 0), "BF", 2, "LED backplate (both sides, same part)")


def module_plate():
    """SM-1: carrier for one 90 x 70 strobe module (bosses face the beam)."""
    set_up((0, -1, 0))
    y1 = pose.led_y0() - pose.FLANGE_Y
    zc = P.LED_ZC
    plate = box(-48.0, 48.0, y1 - 4.0, y1, zc - 38.0, zc + 38.0)
    hx, hz = P.MOD[0] / 2 - P.MOD_HOLE_INSET, P.MOD[1] / 2 - P.MOD_HOLE_INSET
    bosses = [cyl((sx, y1 - 4.0 + 0.01, zc + sz), (0, -1, 0), 4.8, P.MOD_STANDOFF) for sx in (-hx, hx) for sz in (-hz, hz)]
    part = union(plate, *bosses)
    cut = []
    for sx in (-hx, hx):
        for sz in (-hz, hz):
            cut.append(hole((sx, y1 - 4.0 - P.MOD_STANDOFF, zc + sz), (0, 1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    for sx in (-MODBOLT[0], MODBOLT[0]):
        for sz in (-MODBOLT[1], MODBOLT[1]):
            cut.append(cbore((sx, y1 - 4.0, zc + sz), (0, 1, 0), P.M3_CLEAR, 5.0, P.M3_BTN_D, 1.6, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("SM-1", (0, y1 - 4.0, zc - 34.0), (0, -1, 0), (-1, 0, 0), size=4.0))
    return Part("SM-1", part, (0, -1, 0), "BF", 2, "strobe-module carrier plate")


def led_frame(side):
    """LD-2R/L: diffuser frame (IR-passing sheet slides in from the outer side), 4 posts to the backplate,
    baffle wing over the top that reaches under the camera hood."""
    set_up((0, -1, 0))
    xl = side * P.LED_X
    y1 = pose.led_y0()
    zc = P.LED_ZC
    yr = y1 + pose.LED_FRAME_REAR_OFF
    yf = yr + pose.LED_FRAME_T
    frame = box(xl - BP_X, xl + BP_X, yr, yf, zc - BP_Z, zc + BP_Z)
    ox = P.LED_GRID[0] * P.LED_STAR_D / 2 + 1.0
    oz = P.LED_GRID[1] * P.LED_STAR_D / 2 + 1.0
    frame = frame.cut(box(xl - ox, xl + ox, yr - 1, yf + 1, zc - oz, zc + oz))
    px, pz = POST_XZ
    posts = [cyl((xl + sx, y1, zc + sz), (0, 1, 0), 4.5, pose.LED_FRAME_REAR_OFF + 0.01) for sx in (-px, px) for sz in (-pz, pz)]
    xin = side * (P.BASELINE / 2.0 - 22.0)                      # wing reaches in to the camera axis - 22
    zt = zc + BP_Z
    wing = box(min(xin, xl + side * BP_X), max(xin, xl + side * BP_X), yr, yf, zt, zt + P.BAFFLE_WING_T)
    part = union(frame, wing, *posts)
    cut = []
    for sx in (-px, px):
        for sz in (-pz, pz):
            cut.append(hole((xl + sx, y1, zc + sz), (0, 1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    # diffuser slot: from the outer side, at tip + 2
    yg = pose.led_tip_y() + 2.0
    xo = xl + side * (BP_X + 1)
    xi = xl - side * (ox + 2.0)
    cut.append(box(min(xo, xi), max(xo, xi), yg, yg + P.DIFFUSER_T + 0.4, zc - oz - 2.0, zc + oz + 2.0))
    part = cut_all(part, cut)
    part = part.union(emboss("LD-2" + _tag(side), (xl, yf, zc - BP_Z + 3.0), (0, 1, 0), (1, 0, 0), size=4.0))
    return Part("LD-2" + _tag(side), part, (0, -1, 0), "BF", 1, "diffuser frame + baffle wing")


def armed_holder():
    """AL-1: front-visible ARMED LED holder on the beam front slot (centre)."""
    set_up((0, 1, 0))
    x0 = P.AL_X
    y0 = P.Y_BEAM_FRONT
    part = box(x0 - 17, x0 + 17, y0, y0 + 22.0, -14.0, 9.0)
    cut = [hole((x0, y0 - 1, -3.0), (0, 1, 0), P.ARMED_LED_D, 24.0)]
    for dx in (-11.0, 11.0):
        cut.append(cbore((x0 + dx, y0 + 22.0, 0), (0, -1, 0), P.M5_CLEAR, 23.0, P.M5_HEAD_D, P.M5_HEAD_H))
    part = cut_all(part, cut)
    part = part.union(emboss("AL-1", (x0, y0 + 22.0, -9.0), (0, 1, 0), (1, 0, 0), size=4.0))
    return Part("AL-1", part, (0, 1, 0), "BF", 1, "ARMED LED holder (front)")
