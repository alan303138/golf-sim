"""Concept render of the finished (phase 3) unit. Proportions only, not a CAD design.
Run from hardware/enclosure so its VTK renderer can be imported:
  cd hardware/enclosure && python3 ../../docs/concept_render/render_concept.py
World: mm, ball at origin on the mat (z=0), +X down the target line, unit on -Y side facing +Y.
"""
import math, os, sys
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, icosphere
sys.path.insert(0, os.getcwd())
import render

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)))
R = trimesh.transformations

def T(m, x=0, y=0, z=0):
    m = m.copy(); m.apply_translation((x, y, z)); return m

def rot(m, deg, axis, about=(0, 0, 0)):
    m = m.copy(); m.apply_transform(R.rotation_matrix(math.radians(deg), axis, about)); return m

def cyl_y(r, h, sections=48):          # cylinder along +Y, base at y=0
    c = cylinder(radius=r, height=h, sections=sections)
    c.apply_transform(R.rotation_matrix(-math.pi / 2, (1, 0, 0)))
    c.apply_translation((0, h / 2, 0)); return c

C_BODY = (0.20, 0.23, 0.26); C_FACE = (0.10, 0.11, 0.12); C_IR = (0.55, 0.08, 0.10)
C_LENS = (0.04, 0.04, 0.05); C_RING_B = (0.85, 0.25, 0.22); C_RING_C = (0.15, 0.62, 0.58)
C_ALU = (0.70, 0.72, 0.74); C_SCREEN = (0.05, 0.25, 0.35); C_MAT = (0.20, 0.45, 0.25)
C_BALL = (0.97, 0.97, 0.97); C_PERSON = (0.62, 0.66, 0.72); C_SHIELD = (0.75, 0.85, 0.95)

# ---------------------------------------------------------------- sensor head
W, D, H = 400.0, 150.0, 120.0      # body: width (X), depth (Y), height (Z)
PITCH = 17.0                       # head pitched down toward the ball
HEAD_Y, HEAD_Z = -470.0, 255.0     # front-face centre (lens axis)
head = []
body = T(box((W, D, H)), 0, -D / 2, 0)                       # front face at y=0
head.append((body, C_BODY, 1.0))
head.append((T(box((W - 16, 4, H - 16)), 0, 1.5, 0), C_FACE, 1.0))
for sx in (-1, 1):                                           # LED windows
    head.append((T(box((110, 3, 88)), sx * 140, 4.0, 0), C_IR, 0.9))
for sx in (-1, 1):                                           # ball cameras (long lens)
    head.append((T(cyl_y(19, 14), sx * 80, 3, -18), C_RING_B, 1.0))
    head.append((T(cyl_y(13, 16), sx * 80, 3, -18), C_LENS, 1.0))
    head.append((T(cyl_y(12, 10), sx * 32, 3, 30), C_RING_C, 1.0))   # club cameras
    head.append((T(cyl_y(8, 12), sx * 32, 3, 30), C_LENS, 1.0))
head.append((T(cylinder(radius=3.5, height=6), 175, 4, -47), (0.9, 0.2, 0.1), 1.0))   # armed LED
head.append((T(box((70, 40, 25)), 0, -D - 10, -30), (0.12, 0.13, 0.14), 1.0))        # battery slot door
# clear shield on a frame in front
head.append((T(box((W + 20, 3, H + 10)), 0, 26, 0), C_SHIELD, 0.25))
for sx in (-1, 1):
    head.append((T(box((8, 26, H + 10)), sx * (W / 2 + 6), 13, 0), C_BODY, 1.0))
head = [(T(rot(m, -PITCH, (1, 0, 0)), 0, HEAD_Y, HEAD_Z), c, o) for m, c, o in head]

items = list(head)
# legs (three, adjustable)
hip = np.array([0, HEAD_Y - 75, HEAD_Z - 60])
for ang in (90, 210, 330):
    foot = np.array([0, HEAD_Y - 75, 0]) + 210 * np.array([math.cos(math.radians(ang)), math.sin(math.radians(ang)), 0])
    v = foot - hip; L = np.linalg.norm(v)
    leg = cylinder(radius=7, height=L, sections=24)
    leg.apply_transform(trimesh.geometry.align_vectors((0, 0, 1), v / L))
    leg.apply_translation(hip + v / 2)
    items.append((leg, C_ALU, 1.0))
    items.append((T(cylinder(radius=16, height=8), *foot[:2], 4), C_FACE, 1.0))

# telescoping mast with screen and replay camera
MX, MY = 0, HEAD_Y - 110
items.append((T(cylinder(radius=14, height=600), MX, MY, 290 + 300), C_ALU, 1.0))
items.append((T(cylinder(radius=11, height=420), MX, MY, 880 + 210), C_ALU, 1.0))
scr = T(box((180, 14, 112)), 0, 0, 0); bez = T(box((164, 4, 96)), 0, 8, 0)
for m, c in ((scr, C_BODY), (bez, C_SCREEN)):
    m = rot(m, -12, (1, 0, 0)); items.append((T(m, MX, MY + 30, 960), c, 1.0))
items.append((T(box((30, 40, 22)), MX, MY + 18, 960), C_BODY, 1.0))           # screen arm
cam = T(box((60, 46, 44)), MX, MY + 10, 1312)
items.append((cam, C_BODY, 1.0))
items.append((T(rot(cyl_y(12, 12), -10, (1, 0, 0)), MX, MY + 33, 1312), C_LENS, 1.0))

# scene: mat, ball, tee line, golfer (simple mannequin at address)
items.append((T(box((1600, 1400, 4)), 250, 120, -2), C_MAT, 1.0))
items.append((T(icosphere(subdivisions=3, radius=21.3), 0, 0, 21.3), C_BALL, 1.0))
gy = 640
def limb(a, b, r):
    a, b = np.array(a, float), np.array(b, float); v = b - a; L = np.linalg.norm(v)
    c = cylinder(radius=r, height=L, sections=20)
    c.apply_transform(trimesh.geometry.align_vectors((0, 0, 1), v / L)); c.apply_translation(a + v / 2); return c
P = C_PERSON
person = [
    limb((-120, gy, 0), (-110, gy - 40, 480), 55), limb((120, gy, 0), (110, gy - 40, 480), 55),     # legs
    limb((0, gy - 40, 500), (0, gy - 230, 1180), 140),                                             # torso, bent
    T(icosphere(subdivisions=3, radius=100), 0, gy - 330, 1270),                                  # head
    limb((-120, gy - 220, 1140), (-20, gy - 300, 700), 38), limb((120, gy - 220, 1140), (20, gy - 300, 700), 38),
]
for m in person: items.append((m, P, 1.0))
items.append((limb((0, gy - 300, 690), (25, 40, 30), 7), (0.15, 0.15, 0.15), 1.0))                 # club shaft
items.append((T(box((90, 55, 30)), 45, 20, 15), (0.25, 0.25, 0.28), 1.0))                          # club head

views = {
    "concept_golfer_view.png": dict(eye=(1500, 2600, 1500), focal=(0, -300, 500)),
    "concept_side_view.png": dict(eye=(3800, -200, 700), focal=(0, 0, 550)),
    "concept_closeup.png": dict(eye=(450, 350, 650), focal=(0, HEAD_Y, 330), view_angle=34),
    "concept_rear_view.png": dict(eye=(-1300, -2600, 1300), focal=(0, -300, 500)),
}
for name, v in views.items():
    render.render(items, os.path.join(OUT, name), eye=v["eye"], focal=v["focal"],
                  view_angle=v.get("view_angle", 30), size=(1400, 950))
    print("wrote", name)
