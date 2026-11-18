"""Assembly model: printed parts + proxy bodies in the nominal world pose."""
import math
import numpy as np
import cadquery as cq
import params as P
import pose
import parts_cam as C
import parts_led as L
import parts_frame as F
from lib import *

# plate layout on the rear of the beam (BF)
PI_POS = (P.PI_PLATE_X, P.PI_PLATE_Z)
CTRL_POS = (P.CTRL_PLATE_X, P.CTRL_PLATE_Z)
CLIP_X = P.CLIP_X

COLORS = {
    "cam": (0.95, 0.55, 0.15), "led": (0.85, 0.25, 0.25), "pivot": (0.20, 0.55, 0.85),
    "base": (0.25, 0.70, 0.40), "rear": (0.65, 0.45, 0.85), "shield": (0.90, 0.80, 0.20),
    "tmpl": (0.20, 0.75, 0.75), "ext": (0.72, 0.74, 0.78), "elec": (0.10, 0.45, 0.20),
    "optic": (0.15, 0.15, 0.20), "glass": (0.55, 0.80, 0.95), "ball": (1.0, 1.0, 1.0),
    "mat": (0.35, 0.35, 0.38),
}


class Body:
    def __init__(self, name, wp, kind, group, mass=0.0, transparent=False, part=None, ignore_clip=False, moves=False):
        self.moves = moves                 # True when the body pitches with the beam
        self.name, self.wp, self.kind, self.group = name, wp, kind, group
        self.mass, self.transparent, self.part = mass, transparent, part
        self.ignore_clip = ignore_clip     # not an occluder for the field-of-view test

    @property
    def color(self):
        return COLORS.get(self.group, (0.6, 0.6, 0.6))


def printed_mass(wp):
    return wp.val().Volume() / 1000.0 * P.PETG_DENSITY * P.INFILL_EFF


def build_parts():
    """All printed part designs (dict name -> Part) in their own design frames."""
    parts = {}
    def add(p):
        parts[p.name] = p
    for s in (1, -1):
        add(C.clamp_base(s))
    add(C.yaw_stage()); add(C.cam_plate()); add(C.filter_housing()); add(C.filter_ring())
    for sd in (1, -1):
        add(L.led_arm(sd)); add(L.led_frame(sd))
    add(L.led_backplate()); add(L.module_plate()); add(L.armed_holder())
    add(F.pitch_drum()); add(F.pivot_ears()); add(F.mast_collar())
    add(F.hub()); add(F.foot_bracket()); add(F.foot_pad()); add(F.foot_knob())
    add(F.rear_arm(F.pi_arm_xs()[1]))
    add(F.pi_plate(*PI_POS)); add(F.ctrl_plate(*CTRL_POS)); add(F.rear_panel_arm())
    for s in (1, -1):
        add(F.shield_arm(s))
    add(F.sight_blade("F")); add(F.sight_blade("R"))
    add(F.cable_clip(CLIP_X[0]))
    add(F.template_cup()); add(F.template_pin()); add(F.template_card()); add(F.sensor_holder()); add(F.mic_holder())
    return parts


# ---------------------------------------------------------------------------
def _cyl_x(x0, x1, y, z, r):
    return cyl((x0, y, z), (1, 0, 0), r, x1 - x0)


def ox_half():
    return P.LED_GRID[0] * P.LED_STAR_D / 2 + 1.0


def oz_half():
    return P.LED_GRID[1] * P.LED_STAR_D / 2 + 1.0


def led_xz(i, j):
    """Emitter centre (x, z) in LP; the array extent includes the 3 mm chip size."""
    gx, gz = P.LED_GRID
    ax, az = P.LED_ARRAY[0] - 3.0, P.LED_ARRAY[1] - 3.0
    return (-ax / 2 + i * ax / (gx - 1), -az / 2 + j * az / (gz - 1))


def camera_proxies(side):
    """Proxy bodies of one camera in KP frame: board, lens holder, FPC connector, lens, filter."""
    zc = P.Z_AX
    pb = P.CAM_PCB / 2
    pcb = box(-pb, pb, P.PCB_BACK_Y, P.PCB_FRONT_Y, zc - pb, zc + pb)
    hw = P.CAM_HOLDER_W / 2
    holder = box(-hw, hw, P.PCB_FRONT_Y, P.HOLDER_FRONT_Y, zc - hw, zc + hw)
    conn = box(-P.CAM_CONN_W / 2, P.CAM_CONN_W / 2, P.PCB_BACK_Y - P.CAM_REAR_H, P.PCB_BACK_Y,
               zc + pb - P.CAM_CONN_D, zc + pb)
    lens = cyl((0, P.HOLDER_FRONT_Y, zc), (0, 1, 0), P.LENS_D / 2, P.LENS_L)
    filt = cyl((0, P.FH_FRONT_Y - P.SEAT_DEPTH + 0.05, zc), (0, 1, 0), P.FILTER_D / 2, P.FILTER_T)
    return pcb, holder, conn, lens, filt


def build_assembly(theta=None, with_template=True):
    """Return list[Body] in the world frame for pitch angle `theta` (deg)."""
    th = P.PITCH_NOM if theta is None else theta
    Mbf = pose.bf2w_matrix(th)
    Mpf = pose.pf2w_matrix()
    Mhf = pose.hf2w_matrix()
    parts = build_parts()
    bodies = []
    head_frames = [Mbf]

    def is_head(M):
        return any(M is h for h in head_frames)

    def printed(name, M, group, key=None, ignore_clip=False):
        p = parts[key or name.split("#")[0]]
        wp = apply(p.wp, M)
        bodies.append(Body(name, wp, "printed", group, printed_mass(wp), part=p.name,
                           ignore_clip=ignore_clip, moves=is_head(M)))

    def proxy(name, wp_bf_or_w, M, group, mass, transparent=False, ignore_clip=False):
        wp = apply(wp_bf_or_w, M) if M is not None else wp_bf_or_w
        bodies.append(Body(name, wp, "proxy", group, mass, transparent, ignore_clip=ignore_clip, moves=is_head(M)))

    # --- camera stages -----------------------------------------------------
    for s in (1, -1):
        t = C.side_tag(s)
        Mkp = Mbf @ C.kp2bf(s)
        head_frames.append(Mkp)
        printed("CB-1" + t, Mbf, "cam")
        printed("YS-1#" + t, Mkp, "cam", "YS-1")
        printed("CP-1#" + t, Mkp, "cam", "CP-1")
        printed("FH-1#" + t, Mkp, "cam", "FH-1")
        printed("FR-1#" + t, Mkp, "cam", "FR-1")
        pcb, holder, conn, lens, filt = camera_proxies(s)
        proxy("PCB#" + t, pcb, Mkp, "elec", P.CAM_MASS * 0.5)
        proxy("HOLDER#" + t, holder, Mkp, "elec", P.CAM_MASS * 0.5)
        proxy("FPCCONN#" + t, conn, Mkp, "elec", 1.0)
        proxy("LENS#" + t, lens, Mkp, "optic", P.LENS_MASS)
        proxy("FILTER#" + t, filt, Mkp, "glass", 3.0, transparent=True)

    # --- LED side boards (+X = R, -X = L) -------------------------------------
    y1 = pose.led_y0()
    zc = P.LED_ZC
    ytip = pose.led_tip_y()
    for sd in (1, -1):
        t = C.side_tag(sd)
        xl = sd * P.LED_X
        printed("LE-1" + t, Mbf, "led")
        printed("LD-2" + t, Mbf, "led")
        Mt = Mbf @ T4(np.eye(3), (xl, 0, 0))
        head_frames.append(Mt)
        printed("LP-2#" + t, Mt, "led", "LP-2")
        printed("SM-1#" + t, Mt, "led", "SM-1")
        yp = y1 + pose.LED_PLATE_OFF
        plate = box(-P.LED_PLATE[0] / 2, P.LED_PLATE[0] / 2, yp, yp + P.LED_PLATE[2], zc - P.LED_PLATE[1] / 2, zc + P.LED_PLATE[1] / 2)
        proxy("LEDPLATE#" + t, plate, Mt, "elec", 80.0)
        stars, lenses = [], []
        ys = yp + P.LED_PLATE[2]
        for i in range(P.LED_GRID[0]):
            for j in range(P.LED_GRID[1]):
                if (i, j) in P.LED_SKIP:
                    continue
                x = (i - (P.LED_GRID[0] - 1) / 2.0) * P.LED_STAR_D
                z = zc + (j - (P.LED_GRID[1] - 1) / 2.0) * P.LED_STAR_D
                stars.append(cyl((x, ys, z), (0, 1, 0), P.LED_STAR_D / 2, P.LED_STAR_T))
                lenses.append(cyl((x, ys + P.LED_STAR_T, z), (0, 1, 0), P.LED_STAR_D / 2 - 0.2, P.LED_LENS_H))
        proxy("LEDSTARS#" + t, union(*stars), Mt, "elec", 18 * 2.0)
        proxy("LEDLENS#" + t, union(*lenses), Mt, "optic", 18 * 4.0)
        yd = ytip + 2.0 + 0.2
        dif = box(-ox_half() + 2.0, ox_half() - 2.0, yd, yd + P.DIFFUSER_T, zc - oz_half() + 2.0, zc + oz_half() - 2.0)
        proxy("DIFFUSER#" + t, dif, Mt, "glass", 4.0, transparent=True)
        ym = y1 - pose.FLANGE_Y - 4.0 - P.MOD_STANDOFF
        mod = box(-P.MOD[0] / 2, P.MOD[0] / 2, ym - P.MOD[2], ym, zc - P.MOD[1] / 2, zc + P.MOD[1] / 2)
        proxy("MODPCB#" + t, mod, Mt, "elec", P.MOD_MASS * 0.4)
        modp = box(-P.MOD[0] / 2 + 3, P.MOD[0] / 2 - 3, ym - P.MOD[2] - P.MOD_PARTS_H, ym - P.MOD[2], zc - P.MOD[1] / 2 + 3, zc + P.MOD[1] / 2 - 3)
        proxy("MODPARTS#" + t, modp, Mt, "elec", P.MOD_MASS * 0.6)
    printed("AL-1", Mbf, "led")
    proxy("ARMED-LED", cyl((P.AL_X, P.Y_BEAM_FRONT + 14.0, -3.0), (0, 1, 0), P.ARMED_LED_D / 2 - 0.3, 14.0), Mbf, "elec", 1.0)

    # --- pitch pivot ---------------------------------------------------------
    Mdrum = Mbf @ T4(np.eye(3), (pose.PIVOT_X, 0, P.Z_PIVOT))
    head_frames.append(Mdrum)
    printed("DR-1", Mdrum, "pivot")
    printed("PV-1", Mpf, "pivot")
    printed("MC-1", Mpf, "pivot")

    # --- extrusions ----------------------------------------------------------
    beam = ext_proxy((P.BEAM_X0, 0, 0), (1, 0, 0), P.BEAM_LEN)
    proxy("EXT-BEAM", beam, Mbf, "ext", P.BEAM_LEN * P.EXT_MASS_PER_M / 1000.0)
    mast = ext_proxy((0, 0, pose.MAST_BOTTOM_W), (0, 0, 1), pose.MAST_LEN)
    mast = apply(mast, T4(np.eye(3), (pose.PIVOT_W[0], pose.PIVOT_W[1], 0)))
    proxy("EXT-MAST", mast, None, "ext", pose.MAST_LEN * P.EXT_MASS_PER_M / 1000.0)

    # --- hub, legs, feet -------------------------------------------------------
    printed("HB-1", Mhf, "base")
    hub_xy = np.array([pose.PIVOT_W[0], pose.PIVOT_W[1]])
    FOOT_XY = []
    for k, phi in enumerate(P.LEG_ANGLES):
        R = rot_z(phi)
        d = R @ np.array([1.0, 0, 0])
        zc = pose.HUB_Z0 + P.HUB_ARM_H / 2
        p0 = np.array([hub_xy[0], hub_xy[1], zc]) + d * (P.HUB_ARM_L - P.LEG_SOCKET_DEPTH)
        leg = ext_proxy(p0, d, P.LEG_LEN)
        proxy("EXT-LEG%d" % k, leg, None, "ext", P.LEG_LEN * P.EXT_MASS_PER_M / 1000.0)
        Mfb = T4(R, np.array([hub_xy[0], hub_xy[1], pose.HUB_Z0]) + R @ np.array([P.LEG_LEN, 0, 0]))
        printed("FB-1#%d" % k, Mfb, "base", "FB-1")
        fx = hub_xy[0] + (R @ np.array([P.FOOT_R, 0, 0]))[0]
        fy = hub_xy[1] + (R @ np.array([P.FOOT_R, 0, 0]))[1]
        zpad0 = P.RUBBER_T - P.RUBBER_RECESS
        printed("FP-1#%d" % k, T4(np.eye(3), (fx, fy, zpad0)), "base", "FP-1")
        proxy("RUBBER#%d" % k, cyl((fx, fy, 0), (0, 0, 1), P.RUBBER_D / 2, P.RUBBER_T), None, "mat", 3.0)
        zh = P.Z_PAD_TOP - 3.0 + P.FOOT_BOLT_L          # bolt head underside
        bolt = union(cyl((fx, fy, P.Z_PAD_TOP - 3.0), (0, 0, 1), 3.0, P.FOOT_BOLT_L),
                     hexpocket((fx, fy, zh), (0, 0, 1), 10.0, 4.0))
        proxy("BOLT#%d" % k, bolt, None, "ext", 12.0)
        printed("KN-1#%d" % k, T4(np.eye(3), (fx, fy, zh - 0.5)), "base", "KN-1")
        FOOT_XY.append((fx, fy))
    global FOOT_POS
    FOOT_POS = FOOT_XY

    # --- rear plates and boards -------------------------------------------------
    printed("PI-1", Mbf, "rear")
    printed("PB-2", Mbf, "rear")
    for key, xs in (("PI-1", F.pi_arm_xs()), ("PB-2", F.ctrl_arm_xs())):
        for k, xa in enumerate(xs):
            ba = F.rear_arm(xa)
            wp = apply(ba.wp, Mbf)
            bodies.append(Body("BA-1#%s_%d" % (key, k), wp, "printed", "rear", printed_mass(wp), part="BA-1", moves=True))
    printed("RJ-1", Mbf, "rear")
    y_b = P.REAR_PLATE_Y - P.PLATE_T - P.BOARD_STANDOFF       # board face on the bosses (rear side of the plate)
    xc, zc = PI_POS
    pw, ph = P.PI_PCB[0], P.PI_PCB[1]
    pcb = box(xc - pw / 2, xc + pw / 2, y_b - P.PI_PCB[2], y_b, zc - ph / 2, zc + ph / 2)
    proxy("PI5-PCB", pcb, Mbf, "elec", P.PI_MASS * 0.4)
    cx = xc - (36.0 - pw / 2)
    cool = box(cx - P.PI_COOLER[0] / 2, cx + P.PI_COOLER[0] / 2, y_b - P.PI_PCB[2] - P.PI_COOLER_H, y_b - P.PI_PCB[2],
               zc - 28 + 10, zc - 28 + 10 + P.PI_COOLER[1])
    proxy("PI5-COOLER", cool, Mbf, "elec", P.PI_MASS * 0.35)
    port = box(xc - pw / 2, xc - pw / 2 + 18.0, y_b - P.PI_PCB[2] - 16.0, y_b - P.PI_PCB[2], zc - ph / 2 + 2, zc + ph / 2 - 2)
    proxy("PI5-PORTS", port, Mbf, "elec", P.PI_MASS * 0.25)
    xc, zc = CTRL_POS
    cw, ch = P.CTRL[0], P.CTRL[1]
    pcb = box(xc - cw / 2, xc + cw / 2, y_b - P.CTRL[2], y_b, zc - ch / 2, zc + ch / 2)
    proxy("CTRLBOARD", pcb, Mbf, "elec", P.CTRL_MASS * 0.5)
    comp = box(xc - cw / 2 + 6, xc + cw / 2 - 60, y_b - P.CTRL[2] - P.CTRL_COMP_H, y_b - P.CTRL[2], zc - ch / 2 + 6, zc + ch / 2 - 6)
    proxy("CTRL-COMP", comp, Mbf, "elec", P.CTRL_MASS * 0.5)
    # Pico on the control board, USB facing +X (towards the Pi 5)
    px1 = xc + cw / 2 - 4.0
    pico = box(px1 - P.PICO_PCB[0] - 0.0, px1, y_b - P.CTRL[2] - 0.5 - P.PICO_PCB[2], y_b - P.CTRL[2] - 0.5, zc - P.PICO_PCB[1] / 2, zc + P.PICO_PCB[1] / 2)
    proxy("PICO", pico, Mbf, "elec", P.PICO_MASS)
    pcbh = box(px1 - P.PICO_PCB[0] + 3, px1 - 2, y_b - P.CTRL[2] - 0.5 - P.PICO_PCB[2] - P.PICO_HEADER_H, y_b - P.CTRL[2] - 0.5 - P.PICO_PCB[2], zc - P.PICO_PCB[1] / 2 + 1, zc + P.PICO_PCB[1] / 2 - 1)
    proxy("PICO-HDR", pcbh, Mbf, "elec", 1.0)

    # --- shield ------------------------------------------------------------------
    for s in (1, -1):
        printed("SA-1" + C.side_tag(s), Mbf, "shield")
    ysh = pose.shield_y0()
    sheet = box(-P.SHIELD_W / 2, P.SHIELD_W / 2, ysh, ysh + P.SHIELD_T, P.Z_BEAM_TOP + 6.0, P.Z_BEAM_TOP + 6.0 + P.SHIELD_H)
    mass = P.SHIELD_W * P.SHIELD_H * P.SHIELD_T / 1000.0 * P.MASS["shield_per_cm3"]
    proxy("SHIELD", sheet, Mbf, "glass", mass, transparent=True)

    # --- sight blades, clips -------------------------------------------------------
    printed("SG-1F", Mbf, "rear")
    printed("SG-1R", Mbf, "rear")
    for k, x in enumerate(CLIP_X):
        cc = F.cable_clip(x)
        wp = apply(cc.wp, Mbf)
        bodies.append(Body("CC-1#%d" % k, wp, "printed", "rear", printed_mass(wp), part="CC-1", moves=True))

    # --- ball, mat, template -----------------------------------------------------
    ball = cq.Workplane("XY").sphere(P.BALL_R).translate((0, 0, P.BALL_R))
    proxy("BALL", ball, None, "ball", P.MASS["ball"], ignore_clip=True)
    mat = box(-450, 450, -800, 250, -3.0, 0.0)
    proxy("MAT", mat, None, "mat", 0.0, ignore_clip=True)
    if with_template:
        lug, u, D, rot = pose.template_geometry()
        Rt = rot_z(rot)
        Tpin = T4(Rt, (lug[0], lug[1], 0))
        ball_xy = np.array([0.0, 0.0])
        Tball = T4(Rt, (0, 0, 0))
        printed("BT-2", Tpin, "tmpl", ignore_clip=True)
        printed("BT-1", Tball, "tmpl", ignore_clip=True)
        printed("OS-1", Tball, "tmpl", ignore_clip=True)
        sh = P.SENSOR_HEAD
        xf = -(P.BALL_R + P.SENSOR_GAP)
        head = box(xf - sh[2], xf, -sh[0] / 2, sh[0] / 2, P.BALL_R - sh[1] / 2, P.BALL_R + sh[1] / 2)
        proxy("SENSORHEAD", head, Tball, "elec", 5.0, ignore_clip=True)
        stick = ext_proxy((0, 33.0, P.EXT / 2), (0, 1, 0), pose.stick_len())
        proxy("EXT-STICK", stick, Tpin, "ext", 0.0, ignore_clip=True)
        pin = cyl((0, 0, 4.5), (0, 0, 1), 3.0, 80.0 - 4.5)
        proxy("TPL-PIN", pin, Tpin, "ext", 0.0, ignore_clip=True)
        # microphone holder on the stick, MIC_FROM_BALL from the ball (local frame of the ball: stick along -y)
        Tmic = T4(Rt, (0, 0, 0)) @ T4(np.eye(3), (0, -P.MIC_FROM_BALL, 0))
        printed("MM-1", Tmic, "tmpl", ignore_clip=True)
        mw, mh, mt = P.MIC_BOARD
        proxy("MICBOARD", box(-mw / 2, mw / 2, -mh / 2, mh / 2, P.EXT + 0.1 + 5.0, P.EXT + 0.1 + 5.0 + mt), Tmic, "elec", 3.0, ignore_clip=True)
    return bodies


FOOT_POS = []
