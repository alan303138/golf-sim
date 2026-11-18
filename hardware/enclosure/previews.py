"""Preview renders (docs/*.png): assembly views, close-ups, exploded view, one image per
printed part (print orientation), field-of-view diagram."""
import math
import os
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

import params as P
import pose
import parts_cam as C
import parts_led as L
import assembly as A
import render
import checks as K
from lib import best_print
from meshing import to_mesh

HERE = os.path.dirname(os.path.abspath(__file__))
DOCS = os.path.join(HERE, "docs")
os.makedirs(os.path.join(DOCS, "parts"), exist_ok=True)


def _items(bodies, meshes, pred, offsets=None, transp=0.3):
    out = []
    for k, (b, m) in enumerate(zip(bodies, meshes)):
        if not pred(b):
            continue
        mm = m
        if offsets is not None and offsets[k] is not None:
            mm = m.copy()
            mm.apply_translation(offsets[k])
        out.append((mm, b.color, transp if b.transparent else 1.0))
    return out


def _w(p_bf, th=None):
    return (pose.bf2w_matrix(th) @ np.append(p_bf, 1.0))[:3]


def explode_offsets(bodies):
    """World offset per body for the exploded view."""
    Mbf = pose.bf2w_matrix()
    Rbf = Mbf[:3, :3]
    Rkp = {s: (Mbf @ C.kp2bf(s))[:3, :3] for s in (1, -1)}
    kp_y = {"CP-1": 10, "PCB": -25, "HOLDER": -25, "FPCCONN": -25, "LENS": 50, "FH-1": 105, "FILTER": 130, "FR-1": 155}
    lp_y = {}
    bf_off = {"CB-1": (0, 0, -10), "YS-1": (0, 0, 22), "LE-1": (0, 0, -35), "AL-1": (0, 40, 0), "SA-1": (0, 20, 45), "SG-1": (0, 0, 50),
              "SHIELD": (0, 120, 45), "BA-1": (0, -25, 0), "CC-1": (0, 0, -45), "PI-1": (0, -50, 0), "PB-2": (0, -50, 0),
              "RJ-1": (0, -60, 0), "PI5": (0, -110, 0), "CTRL": (0, -110, 0), "PICO": (0, -130, 0), "DR-1": (0, 0, -45),
              "LD-2": (0, 75, 0), "DIFFUSER": (0, 55, 0), "LEDLENS": (0, 38, 0), "LEDSTARS": (0, 26, 0), "LEDPLATE": (0, 14, 0),
              "SM-1": (0, -22, 0), "MODPCB": (0, -45, 0), "MODPARTS": (0, -62, 0), "ARMED": (0, 40, 0)}
    w_off = {"PV-1": (0, 0, -70), "MC-1": (0, 0, -110), "EXT-MAST": (0, 0, -150), "HB-1": (0, 0, -200),
             "RUBBER": (0, 0, -90), "FP-1": (0, 0, -50), "KN-1": (0, 0, 70), "BOLT": (0, 0, 20)}
    out = []
    for b in bodies:
        base = b.name.split("#")[0]
        tag = b.name.split("#")[1] if "#" in b.name else ""
        off = None
        if tag in ("R", "L") and base in kp_y:
            s = 1 if tag == "R" else -1
            off = Rkp[s] @ np.array([0, kp_y[base], 0])
        elif base == "YS-1":
            s = 1 if tag == "R" else -1
            off = Rbf @ np.array([0, 0, 22.0])
        elif base in lp_y:
            off = Rlp @ np.array([0, lp_y[base], 0])
        else:
            for key, v in bf_off.items():
                if base.startswith(key):
                    off = Rbf @ np.array(v, float)
                    break
            if off is None:
                for key, v in w_off.items():
                    if base.startswith(key):
                        off = np.array(v, float)
                        break
        if base.startswith("EXT-LEG") or base.startswith("FB-1"):
            k = int(b.name[-1])
            phi = math.radians(P.LEG_ANGLES[k])
            d = np.array([math.cos(phi), math.sin(phi), 0.0])
            off = d * (90 if base.startswith("EXT-LEG") else 160) + np.array([0, 0, -170 if base.startswith("EXT-LEG") else -120])
            off = d * (60 if base.startswith("EXT-LEG") else 130) + np.array([0, 0, -190 if base.startswith("EXT-LEG") else -150])
        if base.startswith("EXT-BEAM"):
            off = np.zeros(3)
        out.append(off)
    return out


def make_all(bodies, meshes, parts, rep):
    rig = lambda b: b.name not in ("MAT",)
    nomat = lambda b: b.name not in ("MAT", "BALL") and b.group != "tmpl" and not b.name.startswith("EXT-STICK") and b.name != "TPL-PIN"
    nobase = lambda b: nomat(b) and b.group != "base" and not b.name.startswith("EXT-LEG")
    size = (1500, 950)
    ctr = np.array([60.0, -400.0, 130.0])

    # ---------- overall views
    it = _items(bodies, meshes, rig)
    mat = [x for x in it]
    render.render(it, os.path.join(DOCS, "assembly_iso.png"), eye=ctr + np.array([1000, -1250, 800]), focal=ctr, size=size, view_angle=30)
    render.render(it, os.path.join(DOCS, "assembly_iso_back.png"), eye=ctr + np.array([-900, -1300, 700]), focal=ctr, size=size, view_angle=30)
    render.render(it, os.path.join(DOCS, "assembly_iso_ball_side.png"), eye=ctr + np.array([700, 1300, 700]), focal=ctr, size=size, view_angle=30)
    render.render(it, os.path.join(DOCS, "assembly_top.png"), eye=(60, -400, 2600), focal=(60, -400, 0), up=(0, 1, 0), size=size, parallel_scale=520)
    render.render(it, os.path.join(DOCS, "assembly_side.png"), eye=(2600, -400, 150), focal=(60, -400, 150), size=size, parallel_scale=480)
    render.render(it, os.path.join(DOCS, "assembly_front.png"), eye=(60, 2600, 200), focal=(60, -400, 200), size=size, parallel_scale=420)
    render.render(it, os.path.join(DOCS, "assembly_rear.png"), eye=(60, -2600, 200), focal=(60, -400, 200), size=size, parallel_scale=420)

    # ---------- close-ups of the rig (base hidden where it only gets in the way)
    c = _w((0, 40, 45))
    render.render(_items(bodies, meshes, nobase), os.path.join(DOCS, "detail_cameras.png"),
                  eye=c + np.array([260, 520, 330]), focal=c, size=size, view_angle=28)
    c = _w((140, 80, -20))
    render.render(_items(bodies, meshes, nobase), os.path.join(DOCS, "detail_led.png"),
                  eye=c + np.array([260, 380, 260]), focal=c, size=size, view_angle=30)
    render.render(_items(bodies, meshes, nobase), os.path.join(DOCS, "detail_led_rear.png"),
                  eye=c + np.array([-300, -420, 120]), focal=c, size=size, view_angle=30)
    c = _w((-60, -30, 10))
    render.render(_items(bodies, meshes, nobase), os.path.join(DOCS, "detail_rear_boards.png"),
                  eye=c + np.array([260, -520, 300]), focal=c, size=size, view_angle=30)
    c = pose.PIVOT_W + np.array([0, 0, -30])
    render.render(_items(bodies, meshes, nomat), os.path.join(DOCS, "detail_pivot.png"),
                  eye=c + np.array([-330, -280, 150]), focal=c, size=size, view_angle=28)
    render.render(_items(bodies, meshes, nomat), os.path.join(DOCS, "detail_pivot_side.png"),
                  eye=c + np.array([420, 0, 40]), focal=c, size=size, view_angle=28, up=(0, 0, 1))
    c = np.array([pose.PIVOT_W[0], pose.PIVOT_W[1], 60.0])
    render.render(_items(bodies, meshes, lambda b: nomat(b) and b.group in ("base", "ext", "pivot")), os.path.join(DOCS, "detail_hub_feet.png"),
                  eye=c + np.array([420, 520, 520]), focal=c + np.array([0, -40, 0]), size=size, view_angle=34)
    c = np.array([60.0, -250.0, 20.0])
    render.render(_items(bodies, meshes, lambda b: b.group in ("tmpl", "ball", "ext", "base") or b.name in ("BALL",)),
                  os.path.join(DOCS, "detail_template.png"),
                  eye=c + np.array([380, 650, 380]), focal=c, size=size, view_angle=32)

    # ---------- exploded view
    offs = explode_offsets(bodies)
    expl = lambda b: nomat(b)
    c = ctr + np.array([0, -60, 20])
    render.render(_items(bodies, meshes, expl, offs), os.path.join(DOCS, "exploded.png"),
                  eye=c + np.array([1150, 900, 650]), focal=c, size=(1700, 1100), view_angle=32)
    render.render(_items(bodies, meshes, expl, offs), os.path.join(DOCS, "exploded_side.png"),
                  eye=c + np.array([2200, 0, 120]), focal=c, size=(1700, 1000), parallel_scale=560)

    # ---------- one picture per printed part (print orientation)
    pics = []
    for name, p in parts.items():
        w, _ = best_print(p.wp, p.up)
        m = to_mesh(w, 0.05, 0.2)
        grp = {"CB": "cam", "YS": "cam", "CP": "cam", "FH": "cam", "FR": "cam", "LB": "led", "LC": "led", "LD": "led",
               "DR": "pivot", "PV": "pivot", "MC": "pivot", "HB": "base", "FB": "base", "FP": "base", "KN": "base",
               "BT": "tmpl", "CT": "tmpl", "SA": "shield", "LE": "led", "LP": "led", "SM": "led", "AL": "led", "OS": "tmpl", "MM": "tmpl"}.get(name[:2], "rear")
        col = A.COLORS[grp]
        ctrp = m.bounds.mean(axis=0)
        diag = float(np.linalg.norm(m.extents))
        bed = trimesh.creation.box(extents=(max(m.extents[0], 40) + 20, max(m.extents[1], 40) + 20, 0.6))
        bed.apply_translation([ctrp[0], ctrp[1], -0.3])
        path = os.path.join(DOCS, "parts", name + ".png")
        render.render([(bed, (0.82, 0.82, 0.85), 1.0), (m, col, 1.0)], path,
                      eye=ctrp + np.array([0.9, -1.2, 0.85]) * diag * 1.25, focal=ctrp, size=(900, 700), view_angle=30)
        pics.append((name, p.qty, path, m.extents))
    n = len(pics)
    cols = 5
    rows = int(math.ceil(n / cols))
    fig, axs = plt.subplots(rows, cols, figsize=(cols * 3.4, rows * 2.9))
    for ax in axs.ravel():
        ax.axis("off")
    for ax, (name, qty, path, ext) in zip(axs.ravel(), pics):
        ax.imshow(Image.open(path))
        ax.set_title("%s  x%d   %.0fx%.0fx%.0f mm" % (name, qty, ext[0], ext[1], ext[2]), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "parts_sheet.png"), dpi=110)
    plt.close(fig)

    fov_figure(bodies, rep)
    # remove scratch files
    for f in os.listdir(DOCS):
        if f.startswith("_t_"):
            os.remove(os.path.join(DOCS, f))


def fov_figure(bodies, rep):
    """What each camera sees: ball at rest and flight paths in the image (azimuth / elevation)."""
    hu, hv = math.degrees(K.HU), math.degrees(K.HV)
    fig, axs = plt.subplots(1, 3, figsize=(17, 5.4))
    ball = np.array([0.0, 0.0, P.BALL_R])
    cols = {0: "tab:blue", 15: "tab:green", 30: "tab:orange", 45: "tab:red"}
    for ax, side in zip(axs[:2], (-1, 1)):
        cam = K.cam_frame(side)
        ax.add_patch(plt.Rectangle((-hu, -hv), 2 * hu, 2 * hv, fill=False, lw=2, color="k"))
        ax.add_patch(plt.Rectangle((-hu + P.FOV_MARGIN_DEG, -hv + P.FOV_MARGIN_DEG), 2 * (hu - P.FOV_MARGIN_DEG),
                                   2 * (hv - P.FOV_MARGIN_DEG), fill=False, lw=1, ls="--", color="gray"))
        az, el, d = K.cam_coords(cam, [ball])
        ax.add_patch(plt.Circle((math.degrees(az[0]), math.degrees(el[0])), math.degrees(math.atan(P.BALL_R / d[0])),
                                color="gray", alpha=0.7))
        for a, c in cols.items():
            pts = np.array([ball + np.array([dx, 0, dx * math.tan(math.radians(a))]) for dx in np.arange(0, 301, 10)])
            az, el, _ = K.cam_coords(cam, pts)
            ax.plot(np.degrees(az), np.degrees(el), "-", color=c, label="launch %d deg" % a)
            for dx in (100, 200, 250):
                q = ball + np.array([dx, 0, dx * math.tan(math.radians(a))])
                z, e, _ = K.cam_coords(cam, [q])
                ax.plot(math.degrees(z[0]), math.degrees(e[0]), "o", color=c, ms=4)
                if a == 0:
                    ax.annotate("%d" % dx, (math.degrees(z[0]), math.degrees(e[0])), fontsize=7, xytext=(2, -9),
                                textcoords="offset points")
        ax.set_xlim(-hu - 8, hu + 8)
        ax.set_ylim(-hv - 6, hv + 6)
        ax.set_aspect("equal")
        ax.set_title("camera %s (x = %+.0f mm): image of ball and flight (dots at 100/200/250 mm)" % (C.side_tag(side), side * P.BASELINE / 2), fontsize=9)
        ax.set_xlabel("azimuth in image [deg]  (+ = down-range)")
        ax.set_ylabel("elevation [deg]")
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=7, loc="upper left")
    # top view of the frusta
    ax = axs[2]
    for side, col in ((1, "tab:red"), (-1, "tab:blue")):
        cam = K.cam_frame(side)
        for sgn in (-1, 1):
            d = cam["fwd"] + math.tan(K.HU) * sgn * cam["right"]
            p0, p1 = cam["pupil"], cam["pupil"] + d / np.linalg.norm(d) * 650
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=col, lw=1)
        ax.plot(cam["pupil"][0], cam["pupil"][1], "s", color=col)
    ax.plot([0, 300], [0, 0], "k-", lw=2)
    ax.add_patch(plt.Circle((0, 0), P.BALL_R, color="gray"))
    ax.set_aspect("equal")
    ax.set_xlim(-250, 450)
    ax.set_ylim(-520, 150)
    ax.set_title("top view: horizontal field of view of both cameras, target line", fontsize=9)
    ax.set_xlabel("x down-range [mm]")
    ax.set_ylabel("y [mm]")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "fov.png"), dpi=110)
    plt.close(fig)
