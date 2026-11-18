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
import parts as PT
import assembly as A
import render
import checks as K
from lib import best_print
from meshing import to_mesh

tag = PT.tag
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


def _w(p_bf):
    return (pose.bf2w_matrix() @ np.append(p_bf, 1.0))[:3]


def explode_offsets(bodies):
    """World offset per body for the exploded view (BF / K directions rotated to the world)."""
    Mbf = pose.bf2w_matrix()
    Rbf = Mbf[:3, :3]
    Rk = {s: (Mbf @ pose.k2bf(s))[:3, :3] for s in (1, -1)}
    k_off = {"PCB": (0, -45, 0), "HOLDER": (0, -45, 0), "FPCCONN": (0, -45, 0), "LENS": (0, 70, 0), "FILTER": (0, 0, 70)}
    bf_off = {"CM-1": (0, 0, 35), "LT-1": (0, 60, 0), "SC-1": (0, 90, 70), "SHIELD": (0, 160, 70),
              "LEDPLATE": (0, 110, 0), "LEDSTARS": (0, 125, 0), "LEDLENS": (0, 142, 0), "DIFFUSER": (0, 175, 0),
              "MODPCB": (0, -45, 0), "MODPARTS": (0, -45, 0),
              "EB-1": (0, -50, 0), "PI5": (0, -100, 0), "CTRL": (0, -100, 0), "PICO": (0, -110, 0), "EL-1": (0, -190, 0),
              "TA-1": (0, 0, -60), "TRIPOD": (0, 0, -120)}
    out = []
    for b in bodies:
        base = b.name.split("#")[0]
        t = b.name.split("#")[1] if "#" in b.name else ""
        off = None
        if t in ("R", "L") and base in k_off:
            off = Rk[1 if t == "R" else -1] @ np.array(k_off[base], float)
        else:
            for key, v in bf_off.items():
                if base.startswith(key):
                    off = Rbf @ np.array(v, float)
                    break
        out.append(off)
    return out


def make_all(bodies, meshes, parts, rep):
    rig = lambda b: b.name != "MAT"
    norig = lambda b: b.name not in ("MAT", "BALL", "AC-1")
    size = (1500, 950)
    ctr = np.array([100.0, -400.0, 150.0])
    it = _items(bodies, meshes, rig)
    itr = _items(bodies, meshes, norig)
    cr = np.array([110.0, -470.0, 190.0])
    render.render(it, os.path.join(DOCS, "assembly_overview.png"), eye=np.array([900, -1500, 750]), focal=np.array([90, -260, 130]), size=size, view_angle=30)
    render.render(itr, os.path.join(DOCS, "assembly_iso.png"), eye=cr + np.array([650, -800, 420]), focal=cr, size=size, view_angle=30)
    render.render(itr, os.path.join(DOCS, "assembly_iso_ball_side.png"), eye=cr + np.array([450, 700, 330]), focal=cr, size=size, view_angle=30)
    render.render(itr, os.path.join(DOCS, "assembly_iso_back.png"), eye=cr + np.array([-600, -750, 400]), focal=cr, size=size, view_angle=30)
    render.render(it, os.path.join(DOCS, "assembly_top.png"), eye=(110, -300, 2600), focal=(110, -300, 0), up=(0, 1, 0), size=size, parallel_scale=330)
    render.render(it, os.path.join(DOCS, "assembly_side.png"), eye=(2600, -300, 150), focal=(110, -300, 150), size=size, parallel_scale=420)
    render.render(it, os.path.join(DOCS, "assembly_front.png"), eye=(110, 2600, 200), focal=(110, -300, 200), size=size, parallel_scale=300)
    render.render(it, os.path.join(DOCS, "assembly_rear.png"), eye=(110, -2600, 200), focal=(110, -300, 200), size=size, parallel_scale=300)
    c = _w((0, 40, 20))
    render.render(_items(bodies, meshes, norig), os.path.join(DOCS, "detail_cameras.png"), eye=c + np.array([250, 500, 330]), focal=c, size=size, view_angle=28)
    c = _w((140, 60, -20))
    render.render(_items(bodies, meshes, norig), os.path.join(DOCS, "detail_led.png"), eye=c + np.array([250, 380, 260]), focal=c, size=size, view_angle=30)
    render.render(_items(bodies, meshes, lambda b: norig(b) and b.name != "SHIELD"), os.path.join(DOCS, "detail_led_back.png"), eye=c + np.array([-300, -420, 120]), focal=c, size=size, view_angle=30)
    c = _w((60, -30, 80))
    render.render(_items(bodies, meshes, lambda b: norig(b) and b.name != "EL-1"), os.path.join(DOCS, "detail_box.png"), eye=c + np.array([-250, -520, 300]), focal=c, size=size, view_angle=30)
    render.render(_items(bodies, meshes, norig), os.path.join(DOCS, "detail_box_closed.png"), eye=c + np.array([-250, -520, 300]), focal=c, size=size, view_angle=30)
    c = _w((P.TA_X, P.TA_Y * 0.6, -25))
    hid = ("LT-1", "LED", "MOD", "SC-1", "SHIELD")
    render.render(_items(bodies, meshes, lambda b: norig(b) and not b.name.startswith(hid)), os.path.join(DOCS, "detail_tripod.png"), eye=c + np.array([-300, -330, -120]), focal=c, size=size, view_angle=28)
    c = np.array([60.0, -240.0, 20.0])
    render.render(_items(bodies, meshes, lambda b: b.name in ("AC-1", "BALL")), os.path.join(DOCS, "detail_card.png"), eye=np.array([200, -380, 300]), focal=np.array([40, -20, 0]), size=size, view_angle=30)
    offs = explode_offsets(bodies)
    c = ctr + np.array([0, -60, 20])
    render.render(_items(bodies, meshes, norig, offs), os.path.join(DOCS, "exploded.png"), eye=c + np.array([1000, 900, 650]), focal=c, size=(1700, 1100), view_angle=32)
    render.render(_items(bodies, meshes, norig, offs), os.path.join(DOCS, "exploded_side.png"), eye=c + np.array([2200, 0, 120]), focal=c, size=(1700, 1000), parallel_scale=480)

    pics = []
    grpmap = {"CM": "cam", "LT": "led", "SC": "shield", "EB": "box", "EL": "box", "TA": "tripod", "AC": "card"}
    for name, p in parts.items():
        w, _ = best_print(p.wp, p.up)
        m = to_mesh(w, 0.05, 0.2)
        col = A.COLORS[grpmap[name[:2]]]
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
        ax.set_title("%s  %.0fx%.0fx%.0f mm" % (name, ext[0], ext[1], ext[2]), fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(DOCS, "parts_sheet.png"), dpi=110)
    plt.close(fig)
    fov_figure(bodies, rep)


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
        ax.set_title("camera %s (x = %+.0f mm): image of ball and flight (dots at 100/200/250 mm)" % (tag(side), side * P.BASELINE / 2), fontsize=9)
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
