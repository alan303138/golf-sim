"""CadQuery helpers shared by the part modules (English comments, mm)."""
import math
import numpy as np
import cadquery as cq
from cadquery import Vector as V

import params as P

# print-up direction (in the part's own frame) used for teardrop holes
_CTX = {"up": (0.0, 0.0, 1.0)}


def set_up(up):
    _CTX["up"] = tuple(float(c) for c in up)


def get_up():
    return _CTX["up"]


def _v(p):
    return V(*[float(c) for c in p])


def _unit(a):
    a = np.array(a, float)
    return a / np.linalg.norm(a)


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane(obj=cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0)))


def cyl(p, a, r, length):
    """Cylinder from point p along direction a."""
    return cq.Workplane(obj=cq.Solid.makeCylinder(r, length, _v(p), _v(_unit(a))))


def cone(p, a, r0, r1, length):
    return cq.Workplane(obj=cq.Solid.makeCone(r0, r1, length, _v(p), _v(_unit(a))))


def union(*ws):
    out = ws[0]
    for w in ws[1:]:
        out = out.union(w)
    return out


def cut_all(part, cutters):
    """Cut a list of solids one after another (cutters may overlap each other)."""
    for c in cutters:
        if c is not None:
            part = part.cut(c)
    return part


def _plane(p, a, xdir):
    return cq.Plane(origin=_v(p), xDir=_v(xdir), normal=_v(a))


def _perp_up(a, up=None):
    """Unit vector perpendicular to a, as close to the print-up direction as possible."""
    up = np.array(up if up is not None else _CTX["up"], float)
    a = _unit(a)
    v = up - a * np.dot(up, a)
    if np.linalg.norm(v) < 1e-6:
        return None
    return v / np.linalg.norm(v)


def hole(p, a, d, depth, tear=True, up=None):
    """Hole cutter from p along a.  Teardrop profile (apex towards print-up) when the
    hole axis is horizontal in the print orientation, so no support is needed."""
    a = _unit(a)
    r = d / 2.0
    vu = _perp_up(a, up) if (tear and d > 4.7) else None   # small holes bridge fine as circles
    if vu is None:
        return cyl(p, a, r, depth)
    xdir = np.cross(vu, a)
    pl = _plane(p, a, xdir)
    c = cq.Workplane(pl).circle(r).extrude(depth)
    k = r * 0.70711
    t = cq.Workplane(pl).polyline([(-k, k), (0, r * 1.41421), (k, k), (0, 0)]).close().extrude(depth)
    return c.union(t)


def cbore(p, a, d, depth, d2, depth2, tear=True):
    """Counterbored hole: through hole d for `depth`, pocket d2 for the first depth2."""
    return union(hole(p, a, d, depth, tear), hole(p, a, d2, depth2, tear))


def slot(p, a, along, length, d, depth, tear=False):
    """Stadium slot (overall `length` along `along`, width d) from p along a."""
    a = _unit(a)
    pl = _plane(p, a, _unit(along))
    return cq.Workplane(pl).slot2D(length, d, 0).extrude(depth)


def arc_slot(center, normal, u, R, a0, a1, w, depth, z0=0.0):
    """Arc slot of width w, radius R, from angle a0 to a1 (deg, measured from the
    in-plane axis u, counter-clockwise about `normal`), extruded `depth` along normal,
    starting z0 from `center`."""
    n = _unit(normal)
    u = _unit(u)
    c = _v(np.array(center, float) + n * z0)
    pl = _plane(c, n, u)
    ring = cq.Workplane(pl).circle(R + w / 2).circle(R - w / 2).extrude(depth)
    big = R + w
    am = 0.5 * (a0 + a1)
    pts = [(0, 0)] + [(big * math.cos(math.radians(a)), big * math.sin(math.radians(a)))
                       for a in (a0, (a0 + am) / 2, am, (am + a1) / 2, a1)]
    wedge = cq.Workplane(pl).polyline(pts).close().extrude(depth)
    seg = ring.intersect(wedge)
    ends = []
    for a in (a0, a1):
        x, y = R * math.cos(math.radians(a)), R * math.sin(math.radians(a))
        ends.append(cq.Workplane(pl).center(x, y).circle(w / 2).extrude(depth))
    return union(seg, *ends)


def emboss(txt, origin, normal, xdir, size=None, height=None, halign="center"):
    """Raised text starting slightly inside the face at `origin`."""
    size = size or P.EMBOSS_SIZE
    height = height or P.EMBOSS_H
    n = _unit(normal)
    o = np.array(origin, float) - n * 0.1
    pl = _plane(o, n, xdir)
    return cq.Workplane(pl).text(txt, size, height + 0.1, combine=False, halign=halign, valign="center")


# ---------------------------------------------------------------------------
# rigid transforms
# ---------------------------------------------------------------------------
def rot_x(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def T4(R=None, t=(0, 0, 0)):
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    M[:3, 3] = t
    return M


def apply(shape, M):
    """Apply a rigid 4x4 transform to a cq.Workplane / Shape; returns Workplane."""
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    s = shape.val() if isinstance(shape, cq.Workplane) else shape
    tr = gp_Trsf()
    tr.SetValues(*[float(M[i, j]) for i in range(3) for j in range(4)])
    return cq.Workplane(obj=cq.Shape.cast(BRepBuilderAPI_Transform(s.wrapped, tr, True).Shape()))


def mirror_x(wp, x0=0.0):
    """Mirror about the plane x = x0."""
    return cq.Workplane(obj=wp.val().mirror("YZ", (x0, 0, 0)))


def vol(wp):
    return wp.val().Volume()


def bbox(wp):
    b = wp.val().BoundingBox()
    return (b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax)


# ---------------------------------------------------------------------------
# print orientation
# ---------------------------------------------------------------------------
def print_matrix(up, spin=0.0):
    """4x4 rotation that maps the print-up direction to +Z (then spun about Z)."""
    up = _unit(up)
    z = np.array([0, 0, 1.0])
    ax = np.cross(up, z)
    s = np.linalg.norm(ax)
    c = float(np.dot(up, z))
    if s < 1e-9:
        R = np.eye(3) if c > 0 else rot_x(180.0)
    else:
        ax = ax / s
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        R = np.eye(3) + s * K + (1 - c) * (K @ K)
    return T4(rot_z(spin) @ R)


def to_print(wp, up, spin=0.0):
    """Part in print orientation: bottom on z = 0, footprint centred on the origin."""
    w = apply(wp, print_matrix(up, spin))
    b = w.val().BoundingBox()
    return w.translate((-(b.xmin + b.xmax) / 2, -(b.ymin + b.ymax) / 2, -b.zmin))


def fits(wp):
    b = wp.val().BoundingBox()
    vx, vy, vz = P.BUILD_VOLUME
    return (b.xlen <= vx and b.ylen <= vy and b.zlen <= vz)


def best_print(wp, up):
    """Pick the Z-spin that fits the build volume (tries 0/90/45/135 deg)."""
    best = None
    for spin in (0.0, 90.0, 45.0, 135.0):
        w = to_print(wp, up, spin)
        if fits(w):
            return w, spin
        best = (w, spin)
    return best


# ---------------------------------------------------------------------------
# aluminium extrusion proxy
# ---------------------------------------------------------------------------
def ext_proxy(p0, direction, length, roll_deg=0.0, up=(0.0, 0.0, 1.0)):
    """2020 T-slot extrusion proxy starting at p0, running `length` along `direction`.
    Cross-section: 20x20, four T-slots, centre bore.  `roll_deg` rolls it about its axis."""
    d = _unit(direction)
    # build along +Z at origin, then rotate into place
    s = P.EXT
    body = cq.Workplane("XY").rect(s, s).extrude(length)
    cutters = []
    h = s / 2.0
    op, lip, cw, cd = P.EXT_SLOT_OPEN / 2, P.EXT_LIP, P.EXT_CAV_W / 2, P.EXT_CAV_D
    cb = 3.2   # cavity half width at its bottom (trapezoid, keeps neighbouring slots apart)
    tpts = [(h + 1, -op), (h - lip, -op), (h - lip, -cw), (h - cd, -cb),
            (h - cd, cb), (h - lip, cw), (h - lip, op), (h + 1, op)]
    for k in range(4):
        c = cq.Workplane("XY").polyline(tpts).close().extrude(length)
        cutters.append(c.rotate((0, 0, 0), (0, 0, 1), 90 * k))
    bore = cq.Workplane("XY").circle(P.EXT_BORE / 2).extrude(length)
    body = cut_all(body, cutters + [bore])
    # orient: local Z -> d, local X -> the side of the profile that is "up" (so faces stay level)
    z3 = d
    up = np.array(up, float)
    xv = up - z3 * np.dot(up, z3)
    if np.linalg.norm(xv) < 1e-6:
        xv = np.array([1.0, 0.0, 0.0]) - z3 * z3[0]
    xv = xv / np.linalg.norm(xv)
    yv = np.cross(z3, xv)
    R = np.column_stack([xv, yv, z3])
    t = math.radians(roll_deg)
    Rr = rot_z(roll_deg)
    return apply(body, T4(R @ Rr, p0))


def hexpocket(p, a, af, depth, rot=0.0):
    """Hexagonal nut pocket from p along a (across flats af); rot turns it about its axis (deg)."""
    a = _unit(a)
    ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    xdir = np.cross(a, np.cross(ref, a))
    pl = _plane(p, a, xdir)
    r = af / math.sqrt(3.0)
    pts = [(r * math.cos(math.radians(60 * k + rot)), r * math.sin(math.radians(60 * k + rot))) for k in range(6)]
    return cq.Workplane(pl).polyline(pts).close().extrude(depth)


class Part:
    """A printed part: solid in its design frame, print-up vector, quantity."""

    def __init__(self, name, wp, up, frame, qty=1, desc="", settings=""):
        self.name, self.wp, self.up, self.frame = name, wp, tuple(up), frame
        self.qty, self.desc, self.settings = qty, desc, settings
