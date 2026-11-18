"""Pitch pivot (drum, ears, mast collar), hub, feet, rear board plates, shield arms,
sight blades, cable clips and the ball-position template."""
import math
import numpy as np
import params as P
import pose
from lib import *

# ---------------------------------------------------------------------------
# pitch pivot
# ---------------------------------------------------------------------------
EAR_X0 = P.DRUM_L / 2 + P.EAR_GAP          # inner face of the ears
EAR_X1 = EAR_X0 + P.EAR_T
WALL_IN = P.EXT / 2 + P.CLR                 # saddle wall inner face (|y|)
WALL_T = 6.0


def pitch_drum():
    """DR-1: drum between the ears; flat top + saddle walls clamp the beam (4 x M5, side slots).
    Frame: BF translated so the origin sits on the pitch axis (PF axes)."""
    set_up((1, 0, 0))
    R, L2 = P.DRUM_R, P.DRUM_L / 2
    flat = -P.Z_PIVOT + P.Z_BEAM_BOT            # PF z of the flat (beam bottom)
    ztop = -P.Z_PIVOT + P.Z_BEAM_TOP            # PF z of the beam top
    zc0 = -P.Z_PIVOT                            # PF z of the beam centre line
    drum = cyl((-L2, 0, 0), (1, 0, 0), R, P.DRUM_L)
    drum = drum.cut(box(-L2 - 1, L2 + 1, -R - 1, R + 1, flat, R + 1))
    parts = [drum]
    for s in (-1, 1):
        y0 = s * WALL_IN
        y1 = s * (WALL_IN + WALL_T)
        parts.append(box(-18, 18, min(y0, y1), max(y0, y1), 20.0, ztop))
    part = union(*parts)
    cut = []
    for x in (-10, 10):
        cut.append(hole((x, -WALL_IN - WALL_T - 1, zc0), (0, 1, 0), P.M5_CLEAR, 2 * (WALL_IN + WALL_T) + 2))
        cut.append(cyl((x, -WALL_IN - WALL_T - 1, zc0), (0, 1, 0), P.M5_HEAD_D / 2, 1 + P.M5_HEAD_H))
        cut.append(cyl((x, WALL_IN + WALL_T + 1, zc0), (0, -1, 0), P.M5_HEAD_D / 2, 1 + P.M5_HEAD_H))
    # pitch bolts / pivot screws enter from both ends
    pocket = 1.2
    for s in (-1, 1):
        xe = s * L2
        xin = xe - s * pocket if s > 0 else xe
        cut.append(hole((xe, 0, 0), (-s, 0, 0), P.INSERT_M6_D, P.INSERT_M6_L + (pocket if s > 0 else 0), tear=False))
        for k in (-1, 1):
            cut.append(hole((xe, k * P.PIV_LOCK_R, 0), (-s, 0, 0), P.INSERT_M4_D, P.INSERT_M4_L + (pocket if s > 0 else 0), tear=False))
        for th, ang in P.PIV_PINS:                  # detent-pin inserts (drum angle = ear angle + pitch)
            a = math.radians(ang + th)
            cut.append(hole((xe, P.PIV_PIN_R * math.cos(a), P.PIV_PIN_R * math.sin(a)), (-s, 0, 0),
                            P.INSERT_M4_D, P.INSERT_M4_L + (pocket if s > 0 else 0), tear=False))
        # engraved zero line (r = 27.6 .. 31.6) on both end faces
        cut.append(box(min(xe, xe - s * 0.5) - 0.01, max(xe, xe - s * 0.5) + 0.01, 27.6, 31.6, -0.4, 0.4))
    # recessed text pocket on the +X end (open at the top while printing)
    cut.append(cyl((L2 - pocket, 0, 0), (1, 0, 0), 22.0, pocket + 1))
    part = cut_all(part, cut)
    part = part.union(emboss("DR-1", (L2 - pocket, 0, -10.0), (1, 0, 0), (0, 1, 0)))
    return Part("DR-1", part, (1, 0, 0), "PF", 1, "pitch drum with beam saddle")


def pivot_ears():
    """PV-1: base plate with two ears: pivot hole, 2 x arc slots per ear, angle scale. PF frame."""
    set_up((0, 0, 1))
    zt = P.PV_PLATE_TOP_PF
    plate = box(-EAR_X1, EAR_X1, -30.0, 30.0, zt - P.PV_T, zt)
    ears = []
    for s in (-1, 1):
        x0, x1 = (EAR_X0, EAR_X1) if s > 0 else (-EAR_X1, -EAR_X0)
        pl = cq.Workplane("YZ", origin=(x0, 0, 0))
        e = pl.circle(P.EAR_R).extrude(P.EAR_T)
        e = e.union(box(x0, x1, -P.EAR_STEM, P.EAR_STEM, zt, 0.0))
        e = e.union(box(x0, x1, P.EAR_R - 3.0, P.EAR_R + 4.5, -1.0, 1.0))   # pointer fin
        ears.append(e)
    part = union(plate, *ears)
    cut = []
    cut.append(hole((-EAR_X1 - 1, 0, 0), (1, 0, 0), P.M6_CLEAR, 2 * EAR_X1 + 2))
    for s in (-1, 1):
        xo = s * EAR_X1
        cut.append(cyl((xo, 0, 0), (-s, 0, 0), 5.6, 3.5))      # head pocket for the M6 screw
    pr = P.PIV_RANGE
    for s in (-1, 1):
        x0 = EAR_X0 if s > 0 else -EAR_X1
        cut.append(arc_slot((x0 - 0.1, 0, 0), (1, 0, 0), (0, 1, 0), P.PIV_LOCK_R, -pr[1], -pr[0], 4.6, P.EAR_T + 0.2))
        cut.append(arc_slot((x0 - 0.1, 0, 0), (1, 0, 0), (0, 1, 0), P.PIV_LOCK_R, 180 - pr[1], 180 - pr[0], 4.6, P.EAR_T + 0.2))
    for th, ang in P.PIV_PINS:                      # detent-pin holes (positive pitch stops)
        a = math.radians(ang)
        yy, zz = P.PIV_PIN_R * math.cos(a), P.PIV_PIN_R * math.sin(a)
        cut.append(hole((-EAR_X1 - 1, yy, zz), (1, 0, 0), P.M4_CLEAR, 2 * EAR_X1 + 2))
    for x in (-14.0, 14.0):
        for y in (-22.0, 22.0):
            cut.append(cbore((x, y, zt), (0, 0, -1), P.M4_CLEAR, P.PV_T + 1, 7.6, 3.0, tear=False))
    part = cut_all(part, cut)
    # angle scale: raised ticks every degree, long every 5, labels every 10 degrees
    ticks = []
    for s in (-1, 1):
        xf = EAR_X1 if s > 0 else -EAR_X1
        for th in range(0, 31):
            a = -th
            major = th % 5 == 0
            r0, r1 = (23.6, P.EAR_R) if major else (25.4, P.EAR_R)
            w = 0.8 if major else 0.6
            u = np.array([0.0, math.cos(math.radians(a)), math.sin(math.radians(a))])
            pl = cq.Plane(origin=(xf - s * 0.1, 0, 0), xDir=tuple(u), normal=(s, 0, 0))
            tk = cq.Workplane(pl).center((r0 + r1) / 2, 0).rect(r1 - r0, w).extrude(P.EMBOSS_H + 0.1)
            ticks.append(tk)
        for th, ang in P.PIV_PINS:
            a = math.radians(ang)
            tx = P.PIV_PIN_R * math.cos(a)
            tz = P.PIV_PIN_R * math.sin(a) + 5.0
            ticks.append(cq.Workplane(cq.Plane(origin=(xf - s * 0.1, tx, tz), xDir=(0, 1, 0), normal=(s, 0, 0)))
                         .text("%d" % th, 3.0, P.EMBOSS_H + 0.1, combine=False, halign="center", valign="center"))
        for th in (0, 10, 20, 30):
            a = -th
            u = (0.0, math.cos(math.radians(a)), math.sin(math.radians(a)))
            lab = cq.Workplane(cq.Plane(origin=(xf - s * 0.1, 0, 0), xDir=u, normal=(s, 0, 0))).center(21.2, 0) \
                .text(str(th), 3.2, P.EMBOSS_H + 0.1, combine=False, halign="center", valign="center")
            ticks.append(lab)
    part = part.union(cq.Workplane(obj=cq.Compound.makeCompound([t.val() for t in ticks])), clean=False)
    part = part.union(emboss("PV-1", (0, 30.0, zt - P.PV_T / 2), (0, 1, 0), (1, 0, 0), size=4.5))
    return Part("PV-1", part, (0, 0, 1), "PF", 1, "pitch ears with angle scale and arc lock slots")


def mast_collar():
    """MC-1: collar clamping the mast top; flange bolts to PV-1 with 4 x M4. PF frame."""
    set_up((0, 0, -1))
    zf1 = P.PV_BOTTOM_PF
    zf0 = zf1 - P.MC_FLANGE_T
    flange = box(-24.0, 24.0, -30.0, 30.0, zf0, zf1)
    m = P.EXT / 2 + P.CLR
    zc0 = zf0 - P.MC_COLLAR_H
    xo1 = m + 8.0                                   # +X face thickened for the M5 counterbores
    collar = box(-(m + P.MC_WALL), xo1, -(m + P.MC_WALL), m + P.MC_WALL, zc0, zf0)
    part = flange.union(collar)
    cut = [box(-m, m, -m, m, zc0 - 1, zf0)]
    for dz in (7.0, 19.0):
        z = zc0 + dz
        cut.append(hole((xo1, 0, z), (-1, 0, 0), P.M5_CLEAR, 8.0 + 1))
        cut.append(cyl((xo1 + 0.5, 0, z), (-1, 0, 0), P.M5_HEAD_D / 2, 0.5 + P.M5_HEAD_H))
    for x in (-14.0, 14.0):
        for y in (-22.0, 22.0):
            cut.append(hole((x, y, zf1), (0, 0, -1), P.INSERT_M4_D, P.INSERT_M4_L, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("MC-1", (0, 30.0, (zf0 + zf1) / 2), (0, 1, 0), (1, 0, 0)))
    return Part("MC-1", part, (0, 0, -1), "PF", 1, "mast collar + flange")


# ---------------------------------------------------------------------------
# hub, feet
# ---------------------------------------------------------------------------
MAST_POCKET = P.EXT + 0.4
TOWER_HALF = MAST_POCKET / 2 + P.TOWER_WALL


def hub():
    """HB-1: hub with the mast tower and three leg sockets (leg angles in HF)."""
    set_up((0, 0, 1))
    H = P.HUB_ARM_H
    tower = box(-TOWER_HALF, TOWER_HALF, -TOWER_HALF, TOWER_HALF, 0, P.TOWER_H)
    parts = [tower]
    cut = [box(-MAST_POCKET / 2, MAST_POCKET / 2, -MAST_POCKET / 2, MAST_POCKET / 2, -1, P.TOWER_H + 1)]
    for phi in P.LEG_ANGLES:
        arm = box(0, P.HUB_ARM_L, -P.HUB_ARM_W / 2, P.HUB_ARM_W / 2, 0, H)
        pocket = box(P.HUB_ARM_L - P.LEG_SOCKET_DEPTH, P.HUB_ARM_L + 1, -MAST_POCKET / 2, MAST_POCKET / 2,
                     (H - MAST_POCKET) / 2, (H + MAST_POCKET) / 2)
        r = 50.0
        scr = union(hole((r, 0, H + 1), (0, 0, -1), P.M5_CLEAR, 8.0, tear=False),
                    cyl((r, 0, H + 1), (0, 0, -1), P.M5_HEAD_D / 2, 1 + 2.2))
        parts.append(arm.rotate((0, 0, 0), (0, 0, 1), phi))
        cut.append(pocket.rotate((0, 0, 0), (0, 0, 1), phi))
        cut.append(scr.rotate((0, 0, 0), (0, 0, 1), phi))
    part = union(*parts)
    lug = box(-12, 12, 10, P.LUG_Y + 12, 0, 8)
    part = part.union(lug)
    cut.append(hole((0, P.LUG_Y, -1), (0, 0, 1), 6.8, 12, tear=False))
    cut.append(cyl((0, 40.0, 8.0 - 2.0), (0, 0, 1), P.BUBBLE_D / 2 + 0.3, 3.0))   # bubble level pocket
    for s in (-1, 1):                               # mast clamp screws above the arms
        z = H + 6.5
        cut.append(hole((s * (TOWER_HALF + 1), 0, z), (-s, 0, 0), P.M5_CLEAR, P.TOWER_WALL + 2))
        cut.append(cyl((s * (TOWER_HALF + 1), 0, z), (-s, 0, 0), P.M5_HEAD_D / 2, 1 + P.M5_HEAD_H))
    part = cut_all(part, cut)
    part = part.union(emboss("HB-1", (0, -42.0, H), (0, 0, 1), (1, 0, 0)))
    return Part("HB-1", part, (0, 0, 1), "HF", 1, "hub: mast tower, 3 leg sockets, template lug")


def foot_bracket():
    """FB-1: sleeve on the leg end with a trapped M6 nut under the foot bolt (local x = leg axis)."""
    set_up((0, 0, 1))
    H = P.HUB_ARM_H
    L = P.FB_LEN
    part = box(0, L, -P.HUB_ARM_W / 2, P.HUB_ARM_W / 2, 0, H)
    cut = [box(-1, P.FB_SOCKET_DEPTH, -MAST_POCKET / 2, MAST_POCKET / 2, (H - MAST_POCKET) / 2, (H + MAST_POCKET) / 2)]
    cut.append(union(hole((13.0, 0, H + 1), (0, 0, -1), P.M5_CLEAR, 8.0, tear=False),
                     cyl((13.0, 0, H + 1), (0, 0, -1), P.M5_HEAD_D / 2, 1 + 2.2)))
    xb = L - P.FB_BOLT_FROM_BRACKET_END
    cut.append(hole((xb, 0, -1), (0, 0, 1), P.M6_CLEAR, H + 2, tear=False))
    cut.append(hexpocket((xb, 0, -0.01), (0, 0, 1), P.NUT_M6_AF, P.NUT_M6_H))
    part = cut_all(part, cut)
    part = part.union(emboss("FB-1", (xb - 10.0, 0, H), (0, 0, 1), (0, 1, 0), size=4.0))
    return Part("FB-1", part, (0, 0, 1), "FBL", 3, "foot bracket with trapped M6 nut")


def foot_pad():
    """FP-1: foot pad (glued to the bolt end) with a rubber-foot recess underneath."""
    set_up((0, 0, 1))
    part = cq.Workplane("XY").circle(P.FOOT_PAD_D / 2).extrude(P.FOOT_PAD_T).faces("<Z").edges().chamfer(1.5)
    cut = [hole((0, 0, -1), (0, 0, 1), P.M6_CLEAR, P.FOOT_PAD_T + 2, tear=False),
           hexpocket((0, 0, P.FOOT_PAD_T - P.NUT_M6_H), (0, 0, 1), P.NUT_M6_AF, P.NUT_M6_H + 0.01),
           cyl((0, 0, -0.01), (0, 0, 1), P.RUBBER_D / 2 + 0.2, P.RUBBER_RECESS + 0.1)]
    part = cut_all(part, cut)
    part = part.union(emboss("FP-1", (0, 12.5, P.FOOT_PAD_T), (0, 0, 1), (1, 0, 0), size=3.5))
    return Part("FP-1", part, (0, 0, 1), "FPL", 3, "foot pad, glue on the bolt end")


def foot_knob():
    """KN-1: knob that grips the M6 hex head."""
    set_up((0, 0, 1))
    R = 17.0
    part = cq.Workplane("XY").circle(R).extrude(14.0)
    cut = [hexpocket((0, 0, -0.01), (0, 0, 1), 10.0 + 0.5, 5.2)]
    for k in range(8):
        a = math.radians(45 * k)
        cut.append(cyl((R * math.cos(a), R * math.sin(a), -1), (0, 0, 1), 4.2, 16.0))
    part = cut_all(part, cut)
    part = part.union(emboss("KN-1", (0, 0, 14.0), (0, 0, 1), (1, 0, 0), size=5.0))
    return Part("KN-1", part, (0, 0, 1), "KNL", 3, "foot adjusting knob")


# ---------------------------------------------------------------------------
# rear plates (BF)
# ---------------------------------------------------------------------------
BA_HALF_X = 15.0


def rear_arm(xa):
    """BA-1: block between the beam rear T-slot and a rear plate; tunnel gives access
    to the M5 screw from behind."""
    set_up((0, 1, 0))
    y0, y1 = P.REAR_PLATE_Y, P.Y_BEAM_REAR
    part = box(xa - BA_HALF_X, xa + BA_HALF_X, y0, y1, -10.0, 10.0)
    cut = [cyl((xa, y0 - 1, 0), (0, 1, 0), P.M5_TOOL_D / 2, (y1 - y0) - 4.0 + 1),
           hole((xa, y1 - 4.5, 0), (0, 1, 0), P.M5_CLEAR, 6.0)]
    for dx in (-9.5, 9.5):
        cut.append(hole((xa + dx, y0, 0), (0, 1, 0), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("BA-1", (xa, (y0 + y1) / 2, 10.0), (0, 0, 1), (1, 0, 0)))
    return Part("BA-1", part, (0, 1, 0), "BF", 6, "rear arm block (3 plates x 2)")


def _plate_base(xc, zc, hw, hh, arm_xs):
    y1 = P.REAR_PLATE_Y
    y0 = y1 - P.PLATE_T
    plate = box(xc - hw, xc + hw, y0, y1, zc - hh, zc + hh)
    cut = []
    for xa in arm_xs:
        cut.append(cyl((xa, y1 + 1, 0), (0, -1, 0), P.M5_TOOL_D / 2, P.PLATE_T + 2))
        for dx in (-9.5, 9.5):
            cut.append(hole((xa + dx, y1 + 1, 0), (0, -1, 0), P.M3_CLEAR, P.PLATE_T + 2, tear=False))
    return plate, cut, y0


def _boss(x, z, y0, d_out, insert_d, insert_l, od=9.0):
    b = cyl((x, y0 + 0.01, z), (0, -1, 0), od / 2, P.BOARD_STANDOFF)
    ins = hole((x, y0 - P.BOARD_STANDOFF, z), (0, 1, 0), insert_d, insert_l, tear=False)
    return b, ins


def pi_arm_xs():
    return (P.PI_PLATE_X - 55.0, P.PI_PLATE_X + 55.0)


def ctrl_arm_xs():
    return (-213.0, -75.0)


def pi_plate(xc=0.0, zc=14.0):
    """PI-1: Raspberry Pi 5 plate (58 x 49 M2.5 pattern, CSI edge up, all ports open)."""
    set_up((0, -1, 0))
    plate, cut, y0 = _plate_base(xc, zc, 70.0, 34.0, pi_arm_xs())
    bosses = []
    pw, ph = P.PI_PCB[0], P.PI_PCB[1]
    for u in (P.PI_HOLE_EDGE, P.PI_HOLE_EDGE + P.PI_HOLES[0]):
        for v in (P.PI_HOLE_EDGE, P.PI_HOLE_EDGE + P.PI_HOLES[1]):
            x = xc - (u - pw / 2)
            z = zc - (v - ph / 2)
            b, ins = _boss(x, z, y0, 0, P.INSERT_M25_D, P.INSERT_M25_L)
            bosses.append(b)
            cut.append(ins)
    part = union(plate, *bosses)
    part = cut_all(part, cut)
    part = part.union(emboss("PI-1", (xc, y0, zc - 31.0), (0, -1, 0), (-1, 0, 0), size=4.5))
    return Part("PI-1", part, (0, -1, 0), "BF", 1, "Raspberry Pi 5 plate")


def ctrl_plate(xc=-144.5, zc=0.0):
    """PB-2: plate for the 100 x 160 mm control board (Pico socket at its +X end, USB facing the Pi)."""
    set_up((0, -1, 0))
    plate, cut, y0 = _plate_base(xc, zc, 85.0, 54.0, ctrl_arm_xs())
    bosses = []
    hx, hz = P.CTRL[0] / 2 - P.CTRL_HOLE_INSET, P.CTRL[1] / 2 - P.CTRL_HOLE_INSET
    for sx in (-hx, hx):
        for sz in (-hz, hz):
            b, ins = _boss(xc + sx, zc + sz, y0, 0, P.INSERT_M3_D, P.INSERT_M3_L, od=9.6)
            bosses.append(b)
            cut.append(ins)
    part = union(plate, *bosses)
    part = cut_all(part, cut)
    part = part.union(emboss("PB-2", (xc, y0, zc), (0, -1, 0), (-1, 0, 0), size=6.0))
    return Part("PB-2", part, (0, -1, 0), "BF", 1, "100 x 160 control board plate")


def rear_panel_arm():
    """RJ-1: long arm on the rear slot carrying a rear panel with the 12 V jack and the interlock switch."""
    set_up((0, 1, 0))
    xa = P.RJ_X
    yp = P.REAR_PLATE_Y - 44.0
    y1 = P.Y_BEAM_REAR
    block = box(xa - 15, xa + 15, yp, y1, -10.0, 10.0)
    panel = box(xa - 40, xa + 36, yp - 4.0, yp, -22.0, 22.0)
    part = block.union(panel)
    cut = [cyl((xa, yp - 5, 0), (0, 1, 0), P.M5_TOOL_D / 2, (y1 - yp) - 4.0 + 5),
           hole((xa, y1 - 4.5, 0), (0, 1, 0), P.M5_CLEAR, 6.0),
           hole((xa - 24.0, yp - 5, 0), (0, 1, 0), P.JACK_D, 6.0),
           hole((xa + 25.0, yp - 5, 0), (0, 1, 0), P.SWITCH_D, 6.0)]
    part = cut_all(part, cut)
    part = part.union(emboss("RJ-1", (xa + 25.0, yp, -16.0), (0, 1, 0), (1, 0, 0), size=4.0))
    return Part("RJ-1", part, (0, 1, 0), "BF", 1, "rear panel arm: 12 V jack + interlock switch")


# ---------------------------------------------------------------------------
# shield arms, sights, cable clip
# ---------------------------------------------------------------------------
def shield_arm(side):
    """SA-1L / SA-1R: arm from the beam top slot, rising over the LED frames, to the shield side channel.
    Everything except the clamp block is flush with the outer face so the part prints lying on it."""
    xs = side * P.SHIELD_X_ARM
    set_up((-side, 0, 0))
    ysh = pose.shield_y0()
    zb = P.Z_BEAM_TOP
    xo = xs + side * 17.0                         # outer face
    xi = xo - side * 14.0                         # inner face of the 14 mm wide profile
    lo, hi = (min(xo, xi), max(xo, xi))
    block = box(xs - 17, xs + 17, -14, 14, zb, zb + 12)
    riser = box(lo, hi, 6, 20, zb + 12, zb + 34)
    bar = box(lo, hi, 20, ysh - 5, zb + 22, zb + 34)
    post = box(lo, hi, ysh - 5, ysh + P.SHIELD_T + 5, zb + 3, zb + 3 + P.SHIELD_H + 3)
    part = union(block, riser, bar, post)
    if side > 0:
        ch = box(xi - 1, xi + 8, ysh - P.CLR, ysh + P.SHIELD_T + P.CLR, zb + 3 + 3, zb + 3 + P.SHIELD_H + 4)
    else:
        ch = box(xi - 8, xi + 1, ysh - P.CLR, ysh + P.SHIELD_T + P.CLR, zb + 3 + 3, zb + 3 + P.SHIELD_H + 4)
    cut = [ch]
    for dx in (-9, 9):
        cut.append(cbore((xs + dx, 0, zb + 12), (0, 0, -1), P.M5_CLEAR, 13, P.M5_HEAD_D, P.M5_HEAD_H))
    part = cut_all(part, cut)
    tag = "R" if side > 0 else "L"
    part = part.union(emboss("SA-1" + tag, (xs, -9.0, zb + 12), (0, 0, 1), (1, 0, 0), size=4.0))
    return Part("SA-1" + tag, part, (-side, 0, 0), "BF", 1, "shield arm with sheet channel")


def sight_blade(kind):
    """SG-1F (slit, front) / SG-1R (peep hole, rear): sighting blades along the beam axis."""
    set_up((0, 0, 1))
    xs = P.SIGHT_X[1] if kind == "F" else P.SIGHT_X[0]
    zb = P.Z_BEAM_TOP
    s = 1 if kind == "F" else -1
    block = box(xs - 17, xs + 17, -12, 12, zb, zb + 8)
    xb = xs + s * 17
    blade = box(min(xb, xb - s * 3.0), max(xb, xb - s * 3.0), -14, 14, zb, zb + P.SIGHT_H)
    part = block.union(blade)
    zs = zb + 24.0                                    # sight line height above the beam centre = 34
    cut = []
    if kind == "F":
        cut.append(box(xb - 4, xb + 4, -0.6, 0.6, zs - 10, zs + 10))
    else:
        cut.append(cyl((xb - 4, 0, zs), (1, 0, 0), 1.0, 8.0))
    for dx in (-9, 9):
        cut.append(cbore((xs + dx, 0, zb + 8), (0, 0, -1), P.M5_CLEAR, 9, P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("SG-1" + kind, (xs - s * 2.0, 8.5, zb + 8), (0, 0, 1), (1, 0, 0), size=4.0))
    return Part("SG-1" + kind, part, (0, 0, 1), "BF", 1, "sight blade (slit / peep hole)")


def cable_clip(xc):
    """CC-1: cable clip on the beam bottom slot (2 x M5)."""
    set_up((0, 0, -1))
    zb = P.Z_BEAM_BOT
    base = box(xc - 16, xc + 16, -10, 10, zb - 6, zb)
    parts = [base]
    for s in (-1, 1):
        y0, y1 = (5.0, 8.0) if s > 0 else (-8.0, -5.0)
        parts.append(box(xc - 14, xc + 14, y0, y1, zb - 6 - 14, zb - 6))
        lip = cq.Workplane("YZ", origin=(xc - 14, 0, 0)).polyline(
            [(s * 5.0, zb - 20), (s * 5.0, zb - 20 + 1.5), (s * 3.2, zb - 20 + 1.5 + 0.0)]).close().extrude(28)
        parts.append(lip)
    part = union(*parts)
    cut = []
    for dx in (-9, 9):
        cut.append(cbore((xc + dx, 0, zb - 6), (0, 0, 1), P.M5_CLEAR, 7, P.M5_HEAD_D, 3.2, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("CC-1", (xc, 9.9, zb - 3), (0, 1, 0), (1, 0, 0), size=4.0))
    return Part("CC-1", part, (0, 0, -1), "BF", 4, "cable clip")


# ---------------------------------------------------------------------------
# template parts (own frame: origin at the ball / card centre / lug pin, mat at z = 0, stick along -Y;
# the assembly rotates them about Z so that the stick points from the hub lug to the ball)
# ---------------------------------------------------------------------------
SOCK_Y0, SOCK_END = -70.0, -33.0
def _socket():
    m = P.EXT / 2 + P.CLR
    blk = box(-m - 4, m + 4, SOCK_Y0, -P.BALL_R - 2.0, 0, P.EXT + 0.4 + 7.0)   # ends clear of the ball
    return blk, box(-m, m, SOCK_Y0 - 1, SOCK_END, -0.01, P.EXT + 0.4), m


def template_cup():
    """BT-1: ball cup (90 degree countersink) puts the ball centre at z = ball radius."""
    set_up((0, 0, 1))
    blk, pocket, m = _socket()
    plate = box(-62, 35, -35, 35, 0, 9.0)
    part = plate.union(blk)
    zc = P.BALL_R
    rc = P.BALL_R * math.sin(math.radians(45))
    zc_contact = zc - P.BALL_R * math.cos(math.radians(45))
    z_apex = zc_contact - rc - P.CUP_CLR * math.sqrt(2.0)
    cone_cut = cone((0, 0, z_apex), (0, 0, 1), 0.0, 9.0 - z_apex + 2.0, 9.0 - z_apex + 2.0)
    cut = [pocket, cone_cut]
    for y in (-9.5, 9.5):
        cut.append(hole((-45.0, y, 9.0), (0, 0, -1), P.INSERT_M3_D, P.INSERT_M3_L, tear=False))
    zt = P.EXT + 0.4 + 7.0
    cut.append(cbore((0, -50.0, zt), (0, 0, -1), P.M5_CLEAR, 8.0, P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("BT-1", (10.0, 26.0, 9.0), (0, 0, 1), (1, 0, 0)))
    return Part("BT-1", part, (0, 0, 1), "TPL", 1, "ball cup (template)")


def template_pin():
    """BT-2: stick end with an upright M6 pin that locates in the hub lug.  The stick
    arrives from +Y; the block stays clear of the lug tab."""
    set_up((0, 0, 1))
    m = P.EXT / 2 + P.CLR
    zt = P.EXT + 0.4 + 7.0
    base = box(-m - 4, m + 4, -14.0, 22.0, 0, 8.0)
    tube = box(-m - 4, m + 4, 22.0, 74.0, 0, zt)
    part = base.union(tube)
    cut = [box(-m, m, -SOCK_END, 75.0, -0.01, P.EXT + 0.4)]
    cut.append(cbore((0, 50.0, zt), (0, 0, -1), P.M5_CLEAR, 8.0, P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    cut.append(hole((0, 0, -1), (0, 0, 1), P.M6_CLEAR, 10.0, tear=False))
    cut.append(hexpocket((0, 0, -0.01), (0, 0, 1), 10.0 + 0.4, 4.5))
    part = cut_all(part, cut)
    part = part.union(emboss("BT-2", (0, 24.0, zt), (0, 0, 1), (1, 0, 0), size=4.5))
    return Part("BT-2", part, (0, 0, 1), "TPL", 1, "hub-lug pin block (template)")


def template_card():
    """CT-1: holder for a 3 mm calibration card, centred on the ball position."""
    set_up((0, 0, 1))
    blk, pocket, m = _socket()
    plate = box(-65, 65, -26, 20, 0, 8.0)
    rib = box(-65, 65, -8, 8, 8.0, 34.0)
    part = union(plate, rib, blk)
    cut = [pocket, box(-62, 62, -(P.CARD[0] + 0.4) / 2, (P.CARD[0] + 0.4) / 2, 12.0, 40.0)]
    zt = P.EXT + 0.4 + 7.0
    cut.append(cbore((0, -50.0, zt), (0, 0, -1), P.M5_CLEAR, 8.0, P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    part = cut_all(part, cut)
    part = part.union(emboss("CT-1", (0, -14.0, 8.0), (0, 0, 1), (1, 0, 0)))
    return Part("CT-1", part, (0, 0, 1), "TPL", 1, "calibration card holder (template)")


def sensor_holder():
    """OS-1: holder for the optional optical sensor head, bolted on BT-1, face SENSOR_GAP from the ball."""
    set_up((0, 0, 1))
    xf = -(P.BALL_R + P.SENSOR_GAP)
    w, h, ln = P.SENSOR_HEAD
    block = box(xf - ln - 4.0, xf + 2.5, -14.0, 14.0, 9.0, P.BALL_R + h / 2 + 4.0)
    cut = [box(xf - ln - 5.0, xf + 3.0, -(w + 0.6) / 2, (w + 0.6) / 2, P.BALL_R - (h + 0.6) / 2, P.BALL_R + (h + 0.6) / 2)]
    zt = P.BALL_R + h / 2 + 4.0
    for y in (-9.5, 9.5):
        cut.append(cbore((xf - ln / 2 - 4.0 if False else -45.0, y, zt), (0, 0, -1), P.M3_CLEAR, zt - 8.0, P.M3_BTN_D, P.M3_HEAD_H, tear=False))
    part = cut_all(block, cut)
    part = part.union(emboss("OS-1", (xf - ln / 2 - 1.0, 0, zt), (0, 0, 1), (1, 0, 0), size=4.0))
    return Part("OS-1", part, (0, 0, 1), "TPL", 1, "optical sensor head holder (on BT-1)")


def mic_holder():
    """MM-1: microphone board holder clamped on the template stick, MIC_FROM_BALL from the ball."""
    set_up((0, 0, 1))
    z0 = P.EXT + 0.1
    block = box(-16, 16, -32, 32, z0, z0 + 8.0)
    mw, mh, mt = P.MIC_BOARD
    cut = [box(-(mw + 0.6) / 2, (mw + 0.6) / 2, -(mh + 0.6) / 2, (mh + 0.6) / 2, z0 + 8.0 - 3.0, z0 + 9.0),
           cyl((0, 0, z0 - 1), (0, 0, 1), 3.0, 10.0)]
    for y in (-22.0, 22.0):
        cut.append(cbore((0, y, z0 + 8.0), (0, 0, -1), P.M5_CLEAR, 9.0, P.M5_HEAD_D, P.M5_HEAD_H, tear=False))
    part = cut_all(block, cut)
    part = part.union(emboss("MM-1", (-13.0, 0, z0 + 8.0), (0, 0, 1), (0, 1, 0), size=3.5))
    return Part("MM-1", part, (0, 0, 1), "TPL", 1, "microphone board holder (clamps on the stick)")
