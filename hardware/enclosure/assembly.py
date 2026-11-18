"""Assembly model: printed parts + proxy bodies, nominal world pose (pitched beam)."""
import math
import numpy as np
import cadquery as cq
import params as P
import pose
import parts as PT
from lib import *

COLORS = {"cam": (0.95, 0.55, 0.15), "led": (0.85, 0.25, 0.25), "box": (0.20, 0.55, 0.85),
          "shield": (0.90, 0.80, 0.20), "tripod": (0.25, 0.70, 0.40), "card": (0.20, 0.75, 0.75),
          "ext": (0.72, 0.74, 0.78), "elec": (0.10, 0.45, 0.20), "optic": (0.15, 0.15, 0.20),
          "glass": (0.55, 0.80, 0.95), "ball": (1.0, 1.0, 1.0), "mat": (0.35, 0.35, 0.38)}


class Body:
    def __init__(self, name, wp, kind, group, mass=0.0, transparent=False, part=None, ignore_clip=False, moves=True):
        self.name, self.wp, self.kind, self.group = name, wp, kind, group
        self.mass, self.transparent, self.part = mass, transparent, part
        self.ignore_clip, self.moves = ignore_clip, moves

    @property
    def color(self):
        return COLORS.get(self.group, (0.6, 0.6, 0.6))


def printed_mass(wp):
    return wp.val().Volume() / 1000.0 * P.PETG_DENSITY * P.INFILL_EFF


build_parts = PT.build_parts
tag = PT.tag


def camera_proxies_k():
    pb = P.CAM_PCB / 2
    pcb = box(-pb, pb, -P.CAM_PCB_T, 0.0, -pb, pb)
    hw = P.CAM_HOLDER_W / 2
    holder = box(-hw, hw, 0.0, P.HOLDER_FRONT_K, -hw, hw)
    conn = box(-P.CAM_CONN_W / 2, P.CAM_CONN_W / 2, -P.CAM_REAR_H - P.CAM_PCB_T, -P.CAM_PCB_T, pb - P.CAM_CONN_D, pb)
    lens = cyl((0, P.HOLDER_FRONT_K, 0), (0, 1, 0), P.LENS_D / 2, P.LENS_L)
    filt = cyl((0, P.POCKET_K[0] + 0.2, 0), (0, 1, 0), P.FILTER_D / 2, P.FILTER_T)
    return pcb, holder, conn, lens, filt


def build_assembly(theta=None, with_card=True, parts=None):
    th = P.PITCH_NOM if theta is None else theta
    Mbf = pose.bf2w_matrix(th)
    parts = parts or build_parts()
    bodies = []

    def printed(name, group, key=None, M=None, ignore_clip=False, moves=True):
        p = parts[key or name]
        wp = apply(p.wp, Mbf if M is None else M)
        bodies.append(Body(name, wp, "printed", group, printed_mass(wp), part=p.name, ignore_clip=ignore_clip, moves=moves))

    def proxy(name, wp, group, mass, transparent=False, ignore_clip=False, M=None, moves=True):
        M = Mbf if (M is None and moves) else M
        wpp = apply(wp, M) if M is not None else wp
        bodies.append(Body(name, wpp, "proxy", group, mass, transparent, ignore_clip=ignore_clip, moves=moves))

    G = pose.led_geom()
    for s in (1, -1):
        t = tag(s)
        printed("CM-1" + t, "cam")
        printed("LT-1" + t, "led")
        printed("SC-1" + t, "shield")
        Mk = Mbf @ pose.k2bf(s)
        pcb, holder, conn, lens, filt = camera_proxies_k()
        proxy("PCB#" + t, pcb, "elec", P.CAM_MASS * 0.5, M=Mk)
        proxy("HOLDER#" + t, holder, "elec", P.CAM_MASS * 0.5, M=Mk)
        proxy("FPCCONN#" + t, conn, "elec", 1.0, M=Mk)
        proxy("LENS#" + t, lens, "optic", P.LENS_MASS, M=Mk)
        proxy("FILTER#" + t, filt, "glass", 3.0, transparent=True, M=Mk)
        zc, xl = P.LED_ZC, s * P.LED_X
        plate = box(xl - P.LED_PLATE[0] / 2, xl + P.LED_PLATE[0] / 2, G["plate_y0"], G["plate_y1"], zc - P.LED_PLATE[1] / 2, zc + P.LED_PLATE[1] / 2)
        proxy("LEDPLATE#" + t, plate, "elec", 80.0)
        stars, lenses = [], []
        for i in range(P.LED_GRID[0]):
            for j in range(P.LED_GRID[1]):
                if (i, j) in P.LED_SKIP:
                    continue
                x, z = pose.led_star_xz(i, j, s)
                stars.append(cyl((x, G["plate_y1"], z), (0, 1, 0), P.LED_STAR_D / 2, P.LED_STAR_T))
                lenses.append(cyl((x, G["stars_y0"] + P.LED_STAR_T, z), (0, 1, 0), P.LED_STAR_D / 2 - 0.2, P.LED_LENS_H))
        proxy("LEDSTARS#" + t, union(*stars), "elec", 36.0)
        proxy("LEDLENS#" + t, union(*lenses), "optic", 72.0)
        dif = box(xl - 55.0, xl + 55.0, G["slit_y"] + 0.2, G["slit_y"] + 0.2 + P.DIFFUSER_T, zc - 43.0, zc + 43.0)
        proxy("DIFFUSER#" + t, dif, "glass", 4.0, transparent=True)
        proxy("MODPCB#" + t, box(xl - 45, xl + 45, G["pcb_rear"], G["pcb_front"], zc - 35, zc + 35), "elec", P.MOD_MASS * 0.4)
        proxy("MODPARTS#" + t, box(xl - 42, xl + 42, G["pcb_front"], G["parts_front"], zc - 32, zc + 32), "elec", P.MOD_MASS * 0.6)
    printed("EB-1", "box")
    printed("EL-1", "box")
    printed("TA-1", "tripod")

    proxy("EXT-BEAM", ext_proxy((P.BEAM_X0, 0, 0), (1, 0, 0), P.BEAM_LEN), "ext", P.BEAM_LEN * P.EXT_MASS_PER_M / 1000.0)

    g = PT.box_geom()
    yb = g["yb"]
    y_p = yb - P.BOARD_STANDOFF
    pw, ph, pt = P.PI_PCB
    proxy("PI5-PCB", box(P.PI_X - pw / 2, P.PI_X + pw / 2, y_p - pt, y_p, P.PI_Z - ph / 2, P.PI_Z + ph / 2), "elec", P.PI_MASS * 0.4)
    cx, cz = PT.pi_xz(36.0, 28.0)
    proxy("PI5-COOLER", box(cx - P.PI_COOLER[0] / 2, cx + P.PI_COOLER[0] / 2, y_p - pt - P.PI_COOLER_H, y_p - pt, cz - P.PI_COOLER[1] / 2, cz + P.PI_COOLER[1] / 2), "elec", P.PI_MASS * 0.35)
    xp = P.PI_X - pw / 2
    proxy("PI5-PORTS", box(xp, xp + 17.0, y_p - pt - 16.0, y_p - pt, P.PI_Z - ph / 2 + 2, P.PI_Z + ph / 2 - 2), "elec", P.PI_MASS * 0.25)
    y_c = yb - P.BOARD_STANDOFF
    cw, ch, ct = P.CTRL
    proxy("CTRLBOARD", box(g["c_x0"], g["c_x1"], y_c - ct, y_c, g["c_cz"] - ch / 2, g["c_cz"] + ch / 2), "elec", P.CTRL_MASS * 0.5)
    proxy("CTRL-COMP", box(g["c_x0"] + 58, g["c_x1"] - 6, y_c - ct - P.CTRL_COMP_H, y_c - ct, g["c_cz"] - ch / 2 + 6, g["c_cz"] + ch / 2 - 6), "elec", P.CTRL_MASS * 0.5)
    # Pico: USB facing the Pi (-X side of the control board)
    px0 = g["c_x0"] + 3.0
    proxy("PICO", box(px0, px0 + P.PICO_PCB[0], y_c - ct - 0.5 - P.PICO_PCB[2], y_c - ct - 0.5, g["c_cz"] - P.PICO_PCB[1] / 2, g["c_cz"] + P.PICO_PCB[1] / 2), "elec", P.PICO_MASS)

    ysh = pose.shield_y0()
    sheet = box(-P.SHIELD_W / 2, P.SHIELD_W / 2, ysh, ysh + P.SHIELD_T, P.SHIELD_Z[0], P.SHIELD_Z[1])
    proxy("SHIELD", sheet, "glass", P.SHIELD_W * (P.SHIELD_Z[1] - P.SHIELD_Z[0]) * P.SHIELD_T / 1000.0 * 1.2, transparent=True)

    # tripod head platform: plate under the adapter stud, axis along the beam-fixed stud direction
    S = np.array([P.TA_X, P.TA_Y, P.Z_BEAM_BOT - P.TA_T])
    hd = P.TRIPOD_HEAD
    head = box(S[0] - hd[0] / 2, S[0] + hd[0] / 2, S[1] - hd[1] / 2, S[1] + hd[1] / 2, S[2] - hd[2], S[2])
    stud = cyl((S[0], S[1], S[2]), (0, 0, 1), 3.2, P.TA_T - 1.0)
    proxy("TRIPOD-HEAD", head, "ext", 0.0, ignore_clip=True)
    proxy("TRIPOD-STUD", stud, "ext", 0.0, ignore_clip=True)

    ball = cq.Workplane("XY").sphere(P.BALL_R).translate((0, 0, P.BALL_R))
    proxy("BALL", ball, "ball", 45.9, ignore_clip=True, moves=False)
    proxy("MAT", box(-450, 450, -800, 250, -3.0, 0.0), "mat", 0.0, ignore_clip=True, moves=False)
    if with_card:
        p = parts["AC-1"]
        bodies.append(Body("AC-1", p.wp, "printed", "card", 0.0, part="AC-1", ignore_clip=True, moves=False))
    return bodies
