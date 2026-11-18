"""Camera stage parts: clamp base (BF), yaw stage / camera plate / filter housing /
retaining ring (KP frame).  All holes that are horizontal in the print orientation
are teardrops (set_up gives the print-up direction in the part's own frame)."""
import math
import numpy as np
import params as P
import pose
from lib import *


def side_tag(side):
    return "R" if side > 0 else "L"


def kp2bf(side, dyaw=0.0):
    """Transform camera-stage frame (KP) -> beam frame (BF)."""
    psi = pose.toed_in_yaw(side) + dyaw
    return T4(rot_z(psi), (side * P.BASELINE / 2.0, 0.0, P.Z_CB_TOP))


# ---------------------------------------------------------------------------
def clamp_base(side):
    """CB-1L / CB-1R: clamp base on the beam top face; tongue = datum in the T-slot."""
    set_up((0, 0, -1))
    xc = side * P.BASELINE / 2.0
    zb, zt = P.Z_BEAM_TOP, P.Z_CB_TOP
    body = box(xc - P.CB_HALF_L, xc + P.CB_HALF_L, -P.CB_HALF_W, P.CB_HALF_W, zb, zt)
    tongue = box(xc - 48, xc + 48, -P.TONGUE_W / 2, P.TONGUE_W / 2, zb - P.TONGUE_D, zb)
    part = body.union(tongue)
    cut = []
    for dx in (-P.CB_SCREW_X, P.CB_SCREW_X):
        cut.append(cbore((xc + dx, 0, zt), (0, 0, -1), P.M5_CLEAR, P.CB_T + P.TONGUE_D + 1,
                         P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    cut.append(hole((xc, 0, zt), (0, 0, -1), P.INSERT_M4_D, P.INSERT_M4_L, tear=False))
    psi = math.radians(pose.toed_in_yaw(side))
    for k in (-1, 1):
        px = xc + k * P.YS_LOCK_R * math.cos(psi)
        py = k * P.YS_LOCK_R * math.sin(psi)
        cut.append(hole((px, py, zt), (0, 0, -1), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("CB-1" + side_tag(side), (xc, P.CB_HALF_W, (zb + zt) / 2), (0, 1, 0), (1, 0, 0)))
    return Part("CB-1" + side_tag(side), part, (0, 0, -1), "BF", 1,
                "camera clamp base (key in T-slot = datum)")


# ---------------------------------------------------------------------------
def yaw_stage():
    """YS-1: yaw plate with arc slots (+-YAW_RANGE) and tilt cheeks (KP frame)."""
    set_up((0, 0, 1))
    cx0 = P.CP_W / 2 + P.CHEEK_GAP
    cx1 = cx0 + P.CHEEK_T
    ytop = P.CP_FRONT_Y + 4.0
    ztop = P.Z_AX + P.CP_H / 2 + 4.0
    pts = [(-26, -24), (26, -24), (26, 10), (cx1, 10 + (cx1 - 26)), (cx1, ytop),
           (-cx1, ytop), (-cx1, 10 + (cx1 - 26)), (-26, 10)]
    plate = cq.Workplane("XY").polyline(pts).close().extrude(P.YS_T)
    part = plate
    for s in (-1, 1):
        x0, x1 = (cx0, cx1) if s > 0 else (-cx1, -cx0)
        part = part.union(box(x0, x1, 10, ytop, 0, ztop))
    zb0 = ztop - 6.0
    part = part.union(box(-cx1, cx1, 10, 20, zb0, ztop))
    # 45 degree corbels under the bridge (no support needed)
    for s in (-1, 1):
        pl = cq.Workplane("XZ", origin=(0, 20, 0))
        tri = pl.polyline([(s * cx0, zb0 - 12), (s * cx0, zb0), (s * (cx0 - 12), zb0)]).close().extrude(10)
        part = part.union(tri)
    cut = []
    # yaw pivot and lock slots
    cut.append(hole((0, 0, P.YS_T + 1), (0, 0, -1), P.M4_CLEAR, P.YS_T + 2, tear=False))
    sl = P.M3_CLEAR + 2 * P.YS_LOCK_R * math.sin(math.radians(P.YAW_RANGE))
    for k in (-1, 1):
        cut.append(slot((k * P.YS_LOCK_R, 0, P.YS_T + 1), (0, 0, -1), (0, 1, 0), sl, P.M3_CLEAR, P.YS_T + 2))
    # tilt hinge hole and lock slots through both cheeks
    yh = (P.CP_BACK_Y + P.CP_FRONT_Y) / 2
    cut.append(hole((-cx1 - 1, yh, P.Z_AX), (1, 0, 0), P.M3_CLEAR, 2 * cx1 + 2))
    sl2 = P.M3_CLEAR + 2 * P.TILT_LOCK_Z * math.sin(math.radians(P.TILT_RANGE))
    for dz in (-P.TILT_LOCK_Z, P.TILT_LOCK_Z):
        cut.append(slot((-cx1 - 1, yh, P.Z_AX + dz), (1, 0, 0), (0, 1, 0), sl2, P.M3_CLEAR, 2 * cx1 + 2))
    # cable-tie slots in the strain-relief bridge
    for x in (-9.5, 9.5):
        cut.append(box(x - (P.ZIP_T + 0.8) / 2, x + (P.ZIP_T + 0.8) / 2, 15 - (P.ZIP_W + 0.6) / 2,
                       15 + (P.ZIP_W + 0.6) / 2, zb0 - 1, ztop + 1))
    part = cut_all(part, cut)
    part = part.union(emboss("YS-1", (0, -14, P.YS_T), (0, 0, 1), (1, 0, 0)))
    return Part("YS-1", part, (0, 0, 1), "KP", 2, "yaw stage with tilt cheeks and FPC strain relief")


# ---------------------------------------------------------------------------
def cam_plate():
    """CP-1: camera plate. Board seats on 4 bosses behind the plate with two datum
    edges; lens holder passes through the window; filter housing bolts to the front."""
    set_up((0, -1, 0))
    zc = P.Z_AX
    hw = P.CP_W / 2
    plate = box(-hw, hw, P.CP_BACK_Y, P.CP_FRONT_Y, zc - P.CP_H / 2, zc + P.CP_H / 2)
    win = (P.CAM_HOLDER_W + 1.0) / 2
    plate = plate.cut(box(-win, win, P.CP_BACK_Y - 1, P.CP_FRONT_Y + 1, zc - win, zc + win))
    h = P.CAM_HOLE_SQ / 2
    parts = [plate]
    for sx in (-h, h):
        for sz in (-h, h):
            parts.append(cyl((sx, P.CP_BACK_Y, zc + sz), (0, -1, 0), 4.2, P.BOSS_H))
    # datum blocks: bottom edge (2) and left edge (2)
    pb = P.CAM_PCB / 2
    t = P.CLR_TIGHT
    for x in (-10, 10):
        parts.append(box(x - 3, x + 3, P.PCB_BACK_Y, P.CP_BACK_Y, zc - pb - t - 3, zc - pb - t))
    for z in (-10, 10):
        parts.append(box(-pb - t - 3, -pb - t, P.PCB_BACK_Y, P.CP_BACK_Y, zc + z - 3, zc + z + 3))
    part = union(*parts)
    cut = []
    for sx in (-h, h):
        for sz in (-h, h):
            cut.append(hole((sx, P.PCB_FRONT_Y, zc + sz), (0, 1, 0), P.INSERT_M25_D, P.INSERT_M25_L, tear=False))
    f = P.FH_SCREW
    for sx in (-f, f):
        for sz in (-f, f):
            cut.append(hole((sx, P.CP_FRONT_Y, zc + sz), (0, -1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    yh = (P.CP_BACK_Y + P.CP_FRONT_Y) / 2
    for xs, d in ((-hw, 1), (hw, -1)):
        for dz in (-P.TILT_LOCK_Z, 0.0, P.TILT_LOCK_Z):
            cut.append(hole((xs, yh, zc + dz), (d, 0, 0), P.INSERT_M3_D, P.INSERT_M3_L))
    part = cut_all(part, cut)
    part = part.union(emboss("CP-1", (0, yh, zc + P.CP_H / 2), (0, 0, 1), (1, 0, 0)))
    return Part("CP-1", part, (0, -1, 0), "KP", 2, "camera plate (board datum, window, hinge inserts)")


# ---------------------------------------------------------------------------
def filter_housing():
    """FH-1: light-tight sleeve over the lens with a seat for the 850 nm filter."""
    set_up((0, 1, 0))
    zc = P.Z_AX
    y0 = P.CP_FRONT_Y
    f2 = P.FH_FLANGE / 2
    flange = box(-f2, f2, y0, y0 + P.FH_FLANGE_T, zc - f2, zc + f2)
    Rt, Rd = P.FH_TUBE_R, P.FH_DISC_R
    yc1 = P.FH_ROOF_Y
    yc0 = yc1 - (Rd - Rt)           # 45 degree cone, prints without support
    prof = [(0, y0 + 3), (Rt, y0 + 3), (Rt, yc0), (Rd, yc1), (Rd, P.FH_FRONT_Y), (0, P.FH_FRONT_Y)]
    rev = cq.Workplane("XY").polyline(prof).close().revolve(360, (0, 0, 0), (0, 1, 0)).translate((0, 0, zc))
    part = flange.union(rev)
    cut = []
    cut.append(cyl((0, y0 - 1, zc), (0, 1, 0), P.FH_BORE_R, P.FH_ROOF_Y - y0 + 1))
    cut.append(cyl((0, P.FH_ROOF_Y - 0.1, zc), (0, 1, 0), P.FILTER_APERTURE / 2, P.FH_FRONT_Y - P.FH_ROOF_Y + 1))
    cut.append(cyl((0, P.FH_FRONT_Y - P.SEAT_DEPTH, zc), (0, 1, 0), P.FILTER_D / 2 + P.FILTER_SEAT_CLR, P.SEAT_DEPTH + 1))
    f = P.FH_SCREW
    for sx in (-f, f):
        for sz in (-f, f):
            cut.append(cbore((sx, y0 + P.FH_FLANGE_T, zc + sz), (0, -1, 0), P.M3_CLEAR, P.FH_FLANGE_T + 1,
                             P.M3_BTN_D, P.M3_HEAD_H, tear=False))
    r = P.FH_RING_SCREW_R * math.sqrt(0.5)
    for sx in (-r, r):
        for sz in (-r, r):
            cut.append(hole((sx, P.FH_FRONT_Y, zc + sz), (0, -1, 0), P.INSERT_M3_D, P.INSERT_M3_SHORT_L, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("FH-1", (f2, y0 + P.FH_FLANGE_T / 2, zc), (1, 0, 0), (0, 0, 1), size=4.0))
    return Part("FH-1", part, (0, 1, 0), "KP", 2, "filter housing (light-tight sleeve + filter seat)")


def filter_ring():
    """FR-1: retaining ring that clamps the filter into the seat."""
    set_up((0, 1, 0))
    zc = P.Z_AX
    part = cyl((0, P.FH_FRONT_Y, zc), (0, 1, 0), P.FH_DISC_R, P.FR_T)
    cut = [cyl((0, P.FH_FRONT_Y - 1, zc), (0, 1, 0), P.FILTER_APERTURE / 2, P.FR_T + 2)]
    r = P.FH_RING_SCREW_R * math.sqrt(0.5)
    for sx in (-r, r):
        for sz in (-r, r):
            # screws enter from the front (print top): head pocket opens upwards
            cut.append(cbore((sx, P.FH_FRONT_Y + P.FR_T, zc + sz), (0, -1, 0), P.M3_CLEAR, P.FR_T + 1,
                             P.M3_BTN_D, 1.6, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("FR-1", (0, P.FR_FRONT_Y if False else P.FH_FRONT_Y + P.FR_T, zc - 20), (0, 1, 0), (1, 0, 0), size=4.0))
    return Part("FR-1", part, (0, 1, 0), "KP", 2, "filter retaining ring")
