"""The seven printed designs (10 pieces).  Parts are modelled directly in BF; the camera body
is modelled in K and transformed.  `set_up` gives the print-up direction for teardrop holes."""
import math
import numpy as np
import params as P
import pose
from lib import *


def tag(side):
    return "R" if side > 0 else "L"


# ---------------------------------------------------------------------------
# 1. camera mount  CM-1R / CM-1L
# ---------------------------------------------------------------------------
def camera_mount(side):
    """Clamps on the beam top slot (2 x M5), holds the IMX296 board, filter pocket + hood, built-in toe-in."""
    set_up((0, 0, 1))
    H = P.HOOD_HALF
    wb, wf = P.WALL_BACK_K, P.WALL_FRONT_K
    inner = -side                       # K-frame x direction of the inner side (towards the beam centre)
    parts = [box(-H, H, wb, P.HOOD_FRONT_K, -H, H)]
    # floor slab behind the wall (carries datum blocks, ear)
    xe0, xe1 = (-H, H + 6) if inner > 0 else (-H - 6, H)
    parts.append(box(-H, H, P.ZIP_Y[0], wb, -H, -H + 3.0))
    parts.append(box(xe0, xe1, P.ZIP_Y[0], P.ZIP_Y[1], -H, -H + 3.0))
    # ear for the FPC tie, vertical sheet on the inner side
    xa, xb = (H + 3.0, H + 6.0) if inner > 0 else (-H - 6.0, -H - 3.0)
    parts.append(box(xa, xb, P.ZIP_Y[0], P.ZIP_Y[1], -H, 24.0))
    # board seating bosses and datum blocks
    h = P.CAM_HOLE_SQ / 2
    for sx in (-h, h):
        for sz in (-h, h):
            parts.append(cyl((sx, wb + 0.01, sz), (0, -1, 0), 4.2, P.BOSS_H + 0.01))
    pb = P.CAM_PCB / 2
    t = P.CLR_TIGHT
    for x in (-10, 10):
        parts.append(box(x - 3, x + 3, -P.CAM_PCB_T, wb, -pb - t - 3, -pb - t))
    for z in (-10, 10):
        parts.append(box(-pb - t - 3, -pb - t, -P.CAM_PCB_T, wb, z - 3, z + 3))
    body = union(*parts)
    cut = []
    win = (P.CAM_HOLDER_W + 1.0) / 2
    cut.append(box(-win, win, wb - 1, wf + 0.01, -win, win))
    cut.append(tear_prism(wf, P.BORE_END_K, P.BORE_R, trunc=21.0))
    ap = P.FILTER_APERTURE / 2
    cut.append(tear_prism(P.APW1_K[0] - 0.1, P.APW1_K[1] + 0.1, ap))
    fp = P.FILTER_D / 2 + 0.3
    cut.append(box(-fp, fp, P.POCKET_K[0], P.POCKET_K[1], -fp, H + 1))
    cut.append(tear_prism(P.APW2_K[0] - 0.1, P.APW2_K[1] + 1, ap))
    for sx in (-h, h):
        for sz in (-h, h):
            cut.append(hole((sx, 0.0, sz), (0, 1, 0), P.INSERT_M25_D, P.INSERT_M25_L, tear=False))
    for y in (-15.0, -8.0):             # cable-tie slots through the ear
        cut.append(box(xa - 1, xb + 1, y - 1.2, y + 1.2, 8.0, 12.2))
    body = cut_all(body, cut)
    M = T4(rot_z(pose.yaw(side)), pose.cam_origin(side))
    body = apply(body, M)
    # clamp slab on the beam top face (axis-aligned), 2 x M5
    xc = side * P.BASELINE / 2
    zt = P.Z_BEAM_TOP
    slab = box(xc - P.CBASE_HALF_X, xc + P.CBASE_HALF_X, -10.0, 14.0, zt, zt + P.BASE_T)
    part = body.union(slab)
    cuts = [cbore((xc + dx, 0, zt + P.BASE_T), (0, 0, -1), P.M5_CLEAR, P.BASE_T + 1, P.M5_HEAD_D, P.M5_HEAD_H, tear=False)
            for dx in (-28.0, 28.0)]
    part = cut_all(part, cuts)
    part = part.union(emboss("CM-1" + tag(side), (xc, -3.0, zt + P.BASE_T), (0, 0, 1), (1, 0, 0), size=5.0))
    return Part("CM-1" + tag(side), part, (0, 0, 1), "BF", 1, "camera mount")


# ---------------------------------------------------------------------------
# 2. LED board mount  LT-1R / LT-1L
# ---------------------------------------------------------------------------
def led_tray(side, wing=True):
    """Clamp on the beam front slot (2 x M5) + tunnel: aluminium plate seat, diffuser slit, baffle wing,
    posts for the strobe module on the back of the plate."""
    set_up((0, 1, 0))
    G = pose.led_geom()
    s = side
    xl = s * P.LED_X
    zc = P.LED_ZC
    Wo, Zo = 59.5, 49.5
    Wi, Zi = 55.5, 45.5
    y0 = P.Y_BEAM_FRONT
    back = box(xl - Wo, xl + Wo, y0, y0 + 6.0, zc - Zo, zc + Zo)
    tunnel = box(xl - Wo, xl + Wo, y0 + 6.0, G["y_end"], zc - Zo, zc + Zo).cut(
        box(xl - Wi, xl + Wi, y0 + 5.0, G["y_end"] + 1, zc - Zi, zc + Zi))
    lip = box(xl - Wi - 0.5, xl + Wi + 0.5, G["lip0"], G["plate_y0"], zc - Zi - 0.5, zc + Zi + 0.5).cut(
        box(xl - 44, xl + 44, G["lip0"] - 1, G["plate_y0"] + 1, zc - 34, zc + 34))
    posts = [cyl((xl + sx, y0 + 5.9, zc + sz), (0, 1, 0), 4.8, G["pcb_rear"] - y0 - 5.9)
             for sx in (-41.0, 41.0) for sz in (-31.0, 31.0)]
    xin = s * (P.BASELINE / 2 - 22.0)                     # wing reaches in under the camera hood
    wing_b = box(xin, xl - s * Wo, y0, G["y_end"], zc + Zi, zc + Zo)
    part = union(back, tunnel, lip, *( [wing_b] if wing else []), *posts)
    cut = []
    for dx in (-25.0, 25.0):
        cut.append(cbore((xl + dx, 0, y0 + 6.0), (0, -1, 0), P.M5_CLEAR, 7.0, P.M5_HEAD_D, P.M5_HEAD_H))
    for sx in (-50.0, 50.0):
        for sz in (-40.0, 40.0):
            cut.append(hole((xl + sx, G["plate_y0"], zc + sz), (0, -1, 0), P.INSERT_M3_D, P.INSERT_M3_SHORT_L, tear=False))
    for sx in (-41.0, 41.0):
        for sz in (-31.0, 31.0):
            cut.append(hole((xl + sx, G["pcb_rear"], zc + sz), (0, -1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    # diffuser slit through the outer wall, shallow grooves in the inner wall and the bottom wall
    ys = G["slit_y"]
    sh = P.DIFFUSER_T + 0.4
    cut.append(box(xl + s * (Wi - 1), xl + s * (Wo + 1), ys, ys + sh, zc - 43.5, zc + 43.5))
    cut.append(box(xl - s * Wi, xl - s * (Wi + 1.6), ys, ys + sh, zc - 43.5, zc + 43.5))
    cut.append(box(xl - Wi, xl + Wi, ys, ys + sh, zc - Zi, zc - Zi - 1.6))
    # wire window in the bottom wall
    cut.append(box(xl + 12, xl + 36, 30, 44, zc - Zo - 1, zc - Zi + 1))
    part = cut_all(part, cut)
    part = part.union(emboss("LT-1" + tag(side), (xl + s * Wo, 40.0, zc + 25.0), (s, 0, 0), (0, 1, 0), size=5.0))
    return Part("LT-1" + tag(side), part, (0, 1, 0), "BF", 1, "LED board mount")


# ---------------------------------------------------------------------------
# 4. shield clamp  SC-1R / SC-1L
# ---------------------------------------------------------------------------
def shield_clamp(side):
    xs = side * P.SHIELD_X_ARM
    set_up((-side, 0, 0))
    ysh = pose.shield_y0()
    zb = P.Z_BEAM_TOP
    xo = xs + side * 17.0
    xi = xo - side * 14.0
    block = box(xs - 17, xs + 17, -14, 14, zb, zb + 12)
    bar = box(xi, xo, 10, ysh - 5, zb, zb + 12)
    post = box(xi, xo, ysh - 5, ysh + P.SHIELD_T + 5, P.SHIELD_Z[0] - 3, P.SHIELD_Z[1] + 2)
    part = union(block, bar, post)
    ch = box(xi - side * 1, xi + side * 8, ysh - P.CLR, ysh + P.SHIELD_T + P.CLR, P.SHIELD_Z[0], P.SHIELD_Z[1] + 5)
    cut = [ch] + [cbore((xs + dx, 0, zb + 12), (0, 0, -1), P.M5_CLEAR, 13, P.M5_HEAD_D, P.M5_HEAD_H) for dx in (-9, 9)]
    part = cut_all(part, cut)
    part = part.union(emboss("SC-1" + tag(side), (xs, -9.0, zb + 12), (0, 0, 1), (1, 0, 0), size=4.0))
    return Part("SC-1" + tag(side), part, (-side, 0, 0), "BF", 1, "shield clamp")


# ---------------------------------------------------------------------------
# 3. electronics box + lid.  Pi 5 near the beam centre (short ribbons), rotated 180 deg so its USB/ETH
#    edge faces the left wall (window); control board to the right, Pico USB facing the Pi.
# ---------------------------------------------------------------------------
def box_geom():
    pi_x0, pi_x1 = P.PI_X - P.PI_PCB[0] / 2, P.PI_X + P.PI_PCB[0] / 2
    c_x0 = pi_x1 + P.CTRL_GAP
    c_x1 = c_x0 + P.CTRL[0]
    inner_x0, inner_x1 = pi_x0 - 2.0, c_x1 + 2.0
    z_top = P.CTRL_Z0 + P.CTRL[1] + 2.0
    return dict(c_x0=c_x0, c_x1=c_x1, c_cx=(c_x0 + c_x1) / 2, c_cz=P.CTRL_Z0 + P.CTRL[1] / 2,
                x0=inner_x0 - P.BOX_WALL, x1=inner_x1 + P.BOX_WALL, z0=-12.0, z1=z_top + P.BOX_WALL,
                yf=P.Y_BEAM_REAR, yb=P.Y_BEAM_REAR - 6.0, yr=P.Y_BEAM_REAR - 6.0 - P.BOX_DEPTH_IN,
                zi0=-12.0 + P.BOX_WALL, zi1=z_top, xi0=inner_x0, xi1=inner_x1)


LID_POSTS = tuple((P.PI_X + dx, z) for dx, z in ((-38.0, 4.0), (150.0, 4.0), (-38.0, 172.0), (48.0, 172.0), (48.0, 100.0)))
M5_BOX_X = (P.PI_X - 20.0, P.PI_X + 125.0)
JACK_X, SWITCH_X, PANEL_Z = P.PI_X + 70.0, P.PI_X + 110.0, 2.0
ARMED_XZ = (P.PI_X + 10.0, 100.0)


def pi_xz(u, v):
    """Pi 5 board coordinates (u from left edge, v from bottom edge, ports at u = 85) -> BF (x, z); board rotated 180 deg."""
    return (P.PI_X - (u - P.PI_PCB[0] / 2), P.PI_Z - (v - P.PI_PCB[1] / 2))


def pi_hole_xz():
    return [pi_xz(u, v) for u in (P.PI_HOLE_EDGE, P.PI_HOLE_EDGE + P.PI_HOLES[0])
            for v in (P.PI_HOLE_EDGE, P.PI_HOLE_EDGE + P.PI_HOLES[1])]


def ctrl_hole_xz():
    g = box_geom()
    hx, hz = P.CTRL[0] / 2 - P.CTRL_HOLE_INSET, P.CTRL[1] / 2 - P.CTRL_HOLE_INSET
    return [(g["c_cx"] + sx, g["c_cz"] + sz) for sx in (-hx, hx) for sz in (-hz, hz)]


def elec_box():
    set_up((0, -1, 0))
    g = box_geom()
    shell = box(g["x0"], g["x1"], g["yr"], P.Y_BEAM_REAR, g["z0"], g["z1"]).cut(
        box(g["xi0"], g["xi1"], g["yr"] - 1, g["yb"], g["zi0"], g["zi1"]))
    posts = [cyl((x, g["yb"] + 0.01, z), (0, -1, 0), 4.5, P.BOX_DEPTH_IN + 0.01) for x, z in LID_POSTS]
    for x, z in pi_hole_xz():
        posts.append(cyl((x, g["yb"] + 0.01, z), (0, -1, 0), 4.2, P.BOARD_STANDOFF))
    for x, z in ctrl_hole_xz():
        posts.append(cyl((x, g["yb"] + 0.01, z), (0, -1, 0), 4.8, P.BOARD_STANDOFF))
    part = union(shell, *posts)
    cut = []
    for x, z in LID_POSTS:
        cut.append(hole((x, g["yr"], z), (0, 1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    for x, z in pi_hole_xz():
        cut.append(hole((x, g["yb"] - P.BOARD_STANDOFF, z), (0, 1, 0), P.INSERT_M25_D, P.INSERT_M25_L, tear=False))
    for x, z in ctrl_hole_xz():
        cut.append(hole((x, g["yb"] - P.BOARD_STANDOFF, z), (0, 1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    for x in M5_BOX_X:                                           # clamp to the beam back slot
        cut.append(cbore((x, g["yb"], 0), (0, 1, 0), P.M5_CLEAR, 8.0, P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    # ribbon slots in the front wall just above the Pi's CSI edge
    zs = P.PI_Z + P.PI_PCB[1] / 2 + 4.0
    for u, v in P.PI_CSI:
        x, _ = pi_xz(u, v)
        cut.append(box(x - 9, x + 9, g["yb"] - 1, P.Y_BEAM_REAR + 1, zs, zs + 3.5))
    # ARMED LED hole in the front wall (visible from the ball side, between the camera hoods)
    cut.append(hole((ARMED_XZ[0], g["yb"] - 1, ARMED_XZ[1]), (0, 1, 0), P.ARMED_LED_D, 9.0, tear=False))
    # Pi USB/ETH window in the left wall
    zlo, zhi = P.PI_Z - 26.0, P.PI_Z + 26.0
    cut.append(box(g["x0"] - 1, g["xi0"] + 1, g["yb"] - 8.0, g["yb"] - 8.0 - 22.0, zlo, zhi))
    # cable exit for the LED module wires in the right wall, low
    cut.append(box(g["xi1"] - 1, g["x1"] + 1, g["yb"] - 14, g["yb"] - 30, 2.0, 14.0))
    # vents: slots in the top and bottom walls
    for k in range(6):
        xv = g["c_x0"] + 8 + k * 16.0
        cut.append(box(xv, xv + 3.0, g["yb"] - 8, g["yb"] - 34, g["z1"] + 1, g["z1"] - P.BOX_WALL - 1))
    for k in range(5):
        xv = P.PI_X - 35 + k * 16.0
        cut.append(box(xv, xv + 3.0, g["yb"] - 8, g["yb"] - 34, g["z1"] + 1, g["z1"] - P.BOX_WALL - 1))
        cut.append(box(xv, xv + 3.0, g["yb"] - 8, g["yb"] - 34, g["z0"] - 1, g["z0"] + P.BOX_WALL + 1))
    part = cut_all(part, cut)
    part = part.union(emboss("EB-1", (P.PI_X - 10.0, g["yb"] + 0.01, 100.0), (0, -1, 0), (-1, 0, 0), size=6.0))
    return Part("EB-1", part, (0, -1, 0), "BF", 1, "electronics box")


def elec_lid():
    set_up((0, -1, 0))
    g = box_geom()
    y1 = g["yr"]
    lid = box(g["x0"], g["x1"], y1 - 3.0, y1, g["z0"], g["z1"])
    cut = [cbore((x, y1, z), (0, -1, 0), P.M3_CLEAR, 4.0, P.M3_BTN_D, 1.6, tear=False) for x, z in LID_POSTS]
    cut = [hole((x, y1 + 0.01, z), (0, -1, 0), P.M3_CLEAR, 4.0, tear=False) for x, z in LID_POSTS]
    cut.append(hole((JACK_X, y1 + 1, PANEL_Z), (0, -1, 0), P.JACK_D, 5.0, tear=False))
    cut.append(hole((SWITCH_X, y1 + 1, PANEL_Z), (0, -1, 0), P.SWITCH_D, 5.0, tear=False))
    part = cut_all(lid, cut)
    part = part.union(emboss("EL-1", (P.PI_X, y1 + 0.01, 100.0), (0, 1, 0), (1, 0, 0), size=8.0))
    return Part("EL-1", part, (0, 1, 0), "BF", 1, "electronics box lid")


# ---------------------------------------------------------------------------
# 5. tripod adapter
# ---------------------------------------------------------------------------
def tripod_adapter():
    set_up((0, 0, 1))
    zt = P.Z_BEAM_BOT
    zb = zt - P.TA_T
    y_lo = -10.0
    y_hi = max(10.0, P.TA_Y + 40.0)
    plate = box(P.TA_X - P.TA_W / 2, P.TA_X + P.TA_W / 2, y_lo, y_hi, zb, zt)
    cut = []
    for dx in (-22.0, 22.0):
        cut.append(cbore((P.TA_X + dx, 0, zb), (0, 0, 1), P.M5_CLEAR, P.TA_T + 1, P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    cut.append(hole((P.TA_X, P.TA_Y, zb - 1), (0, 0, 1), P.UNC14_CLEAR, P.TA_T + 2, tear=False))
    cut.append(hexpocket((P.TA_X, P.TA_Y, zt - P.NUT14_H), (0, 0, 1), P.NUT14_AF, P.NUT14_H + 0.01))
    ya = P.TA_Y + 28.0                                              # optional 3/8"-16 position
    cut.append(hole((P.TA_X, ya, zb - 1), (0, 0, 1), P.UNC38_CLEAR, P.TA_T + 2, tear=False))
    cut.append(hexpocket((P.TA_X, ya, zt - P.NUT38_H), (0, 0, 1), P.NUT38_AF, P.NUT38_H + 0.01))
    part = cut_all(plate, cut)
    part = part.union(emboss("TA-1", (P.TA_X + P.TA_W / 2, (y_lo + y_hi) / 2, (zt + zb) / 2), (1, 0, 0), (0, 1, 0), size=5.0))
    return Part("TA-1", part, (0, 0, 1), "BF", 1, "tripod adapter")


# ---------------------------------------------------------------------------
# 6. alignment card
# ---------------------------------------------------------------------------
def card_info():
    """Numbers printed on the card (world frame)."""
    M = pose.bf2w_matrix()
    S = pose.tripod_point_w()
    return dict(stud_x=float(S[0]), stud_y=float(S[1]), stud_z=float(S[2]),
                pupil_dist=float(P.BALL_DIST), mean_pupil_x=float(P.AIM_U))


def alignment_card():
    set_up((0, 0, 1))
    W, D, T = P.CARD
    plate = box(-W / 2, W / 2, -D / 2, D / 2, 0, T)
    cupr = 26.0
    boss = cyl((0, 0, 0), (0, 0, 1), cupr, 9.0)
    part = plate.union(boss)
    zc = P.BALL_R
    rc = P.BALL_R * math.sin(math.radians(45))
    zcon = zc - P.BALL_R * math.cos(math.radians(45))
    z_apex = zcon - rc - 0.1 * math.sqrt(2.0)
    part = part.cut(cone((0, 0, z_apex), (0, 0, 1), 0.0, 9.0 - z_apex + 2.0, 9.0 - z_apex + 2.0))
    info = card_info()
    # target-line arrow (+X)
    arrow = box(cupr + 6, cupr + 70, -3, 3, T - 0.1, T + P.EMBOSS_H)
    head = cq.Workplane("XY", origin=(0, 0, T - 0.1)).polyline([(cupr + 70, -9), (cupr + 90, 0), (cupr + 70, 9)]).close().extrude(P.EMBOSS_H + 0.1)
    part = part.union(arrow).union(head)
    # ruler along -Y (rig side), ticks every 10 mm, long every 50
    for k in range(3, 10):
        y = -k * 10.0
        w = 16.0 if k % 5 == 0 else 8.0
        part = part.union(box(-w / 2, w / 2, y - 0.5, y + 0.5, T - 0.1, T + P.EMBOSS_H))
    # V notches on the rig-side edge: ball line and tripod-stud line
    for x in (0.0, max(-W / 2 + 12, min(W / 2 - 12, info["stud_x"]))):
        part = part.cut(cq.Workplane("XY", origin=(0, 0, -1)).polyline([(x - 8, -D / 2 - 0.1), (x + 8, -D / 2 - 0.1), (x, -D / 2 + 14)]).close().extrude(T + 2))
    lines = ["TARGET LINE +X", "BALL CENTRE = CUP", "RIG EDGE: PUPIL %d MM FROM BALL" % P.BALL_DIST,
             "TRIPOD STUD X %+.0f Y %.0f" % (info["stud_x"], info["stud_y"]), "HEIGHT %.0f MM PITCH %.0f DEG" % (info["stud_z"], P.PITCH_NOM)]
    for i, tx in enumerate(lines):
        part = part.union(emboss(tx, (0, 60.0 - i * 10.0, T), (0, 0, 1), (1, 0, 0), size=5.0))
    part = part.union(emboss("AC-1", (60.0, -80.0, T), (0, 0, 1), (1, 0, 0), size=6.0))
    return Part("AC-1", part, (0, 0, 1), "WF", 1, "alignment card")


def build_parts():
    out = {}
    for s in (1, -1):
        out["CM-1" + tag(s)] = camera_mount(s)
        out["LT-1" + tag(s)] = led_tray(s)
        out["SC-1" + tag(s)] = shield_clamp(s)
    out["EB-1"] = elec_box()
    out["EL-1"] = elec_lid()
    out["TA-1"] = tripod_adapter()
    out["AC-1"] = alignment_card()
    return out
