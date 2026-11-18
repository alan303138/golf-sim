"""Thin helper layer over schemdraw used by all sheet scripts.

* part labels (ref + value) are read from circuit.py, so a drawing can never show a value that the
  netlist does not have;
* every drawn ref and every net label is recorded in out/<sheet>.json; tools/check_design.py compares
  these records with netlist/netlist.yaml.
Coordinates are drawing units (1 unit = 0.5 inch at 100 % scale).
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import schemdraw
import schemdraw.elements as elm

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import circuit as CK   # noqa: E402

schemdraw.use("matplotlib")
OUTDIR = os.path.join(HERE, "out")
os.makedirs(OUTDIR, exist_ok=True)

C_NET = "#0b5394"
C_TXT = "#111111"
C_NOTE = "#7f6000"


class Sheet:
    def __init__(self, name, title, covers, fontsize=8):
        self.name, self.title, self.covers = name, title, covers
        self.d = schemdraw.Drawing(show=False)
        self.d.config(unit=2.0, fontsize=fontsize, lw=1.1)
        self.refs, self.nets, self.values = set(), set(), {}
        self.fs = fontsize

    # ---------------------------------------------------------------- basics
    def reg(self, ref):
        if ref in CK.PARTS:
            self.refs.add(ref)
            self.values[ref] = CK.PARTS[ref]["value"]
        else:
            raise KeyError(ref)

    def text(self, xy, s, ha="left", va="center", size=None, color=C_TXT, weight="normal", rot=0):
        self.d += elm.Label().at(xy).label(s, loc="center", halign=ha, valign=va, fontsize=size or self.fs - 1,
                                           color=color, rotate=rot, ofst=0)

    def net(self, xy, name, ha="left", va="center", size=None, record=True):
        if record:
            self.nets.add(name)
        self.text(xy, name, ha, va, size or self.fs - 1, C_NET, "bold")

    def note(self, xy, s, ha="left", size=None):
        self.text(xy, s, ha, "top", size or self.fs - 1.5, C_NOTE)

    def title_block(self, x, y, subtitle=""):
        self.d += elm.Label().at((x, y)).label(self.title, loc="center", halign="left", valign="top", fontsize=self.fs + 5, color="#000000", ofst=0)
        if subtitle:
            self.d += elm.Label().at((x, y - 0.9)).label(subtitle, loc="center", halign="left", valign="top", fontsize=self.fs - 1, color="#444444", ofst=0)

    def wire(self, *pts, color=None):
        for a, b in zip(pts[:-1], pts[1:]):
            e = elm.Line().at(a).to(b)
            if color:
                e = e.color(color)
            self.d += e

    def dot(self, xy, open=False):
        self.d += elm.Dot(open=open).at(xy)

    def gnd(self, xy, kind="gnd", label=None):
        if kind == "pgnd":
            self.d += elm.GroundChassis().at(xy)
            self.net((xy[0] + 0.3, xy[1] - 0.45), label or "PGND", va="center")
        else:
            self.d += elm.Ground().at(xy)
            self.net((xy[0] + 0.3, xy[1] - 0.45), label or "GND_L", va="center")

    def dash_box(self, x0, y0, x1, y1, label="", color="#888888"):
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
        for a, b in zip(pts[:-1], pts[1:]):
            self.d += elm.Line().at(a).to(b).linestyle("--").color(color).linewidth(0.8)
        if label:
            self.text((x0 + 0.15, y1 - 0.2), label, "left", "top", self.fs - 1, "#555555")

    # ---------------------------------------------------------------- 2-terminal parts
    def two(self, cls, ref, p1, p2, lab=None, side="auto", extra="", also=(), dx=0.0, **kw):
        """draw a 2-terminal part between absolute points; label = ref + value (+extra).
        lab overrides the text; also = further refs drawn by this one symbol (parallel group)."""
        self.reg(ref)
        for a in also:
            self.reg(a)
        e = cls(**kw) if kw else cls()
        self.d += e.at(p1).to(p2)
        v = CK.PARTS[ref]["value"]
        txt = lab if lab is not None else f"{ref}\n{v}" + (f"\n{extra}" if extra else "")
        mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
        vertical = abs(p1[0] - p2[0]) < abs(p1[1] - p2[1])
        if side == "auto":
            side = "right" if vertical else "top"
        o = 0.55 + dx
        if side == "right":
            self.text((mx + o, my), txt, "left", "center")
        elif side == "left":
            self.text((mx - o, my), txt, "right", "center")
        elif side == "top":
            self.text((mx, my + o), txt, "center", "bottom")
        else:
            self.text((mx, my - o), txt, "center", "top")
        return e

    def R(self, ref, p1, p2, **kw):
        return self.two(elm.Resistor, ref, p1, p2, **kw)

    def C(self, ref, p1, p2, polar=False, **kw):
        return self.two(elm.Capacitor, ref, p1, p2, polar=polar, **kw) if polar else self.two(elm.Capacitor, ref, p1, p2, **kw)

    def D(self, ref, p1, p2, kind="diode", **kw):
        cls = {"diode": elm.Diode, "schottky": elm.Schottky, "zener": elm.Zener, "led": elm.LED}[kind]
        return self.two(cls, ref, p1, p2, **kw)

    def F(self, ref, p1, p2, **kw):
        return self.two(elm.Fuse, ref, p1, p2, **kw)

    # ---------------------------------------------------------------- IC / connector boxes
    def ic(self, ref, x, y, w, h, left=(), right=(), top=(), bottom=(), title=None, skip=(), subtitle=None, stub=0.5):
        """own box drawer. pins: (pin_number, pin_name, net); net 'NC' drawn with a cross.
        returns dict str(pin) -> stub end xy."""
        self.reg(ref)
        pts = [(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)]
        for p, q in zip(pts[:-1], pts[1:]):
            self.d += elm.Line().at(p).to(q)
        res = {}
        spec = {"left": left, "right": right, "top": top, "bottom": bottom}
        for side, lst in spec.items():
            n = len(lst)
            for k, (pn, name, net) in enumerate(lst):
                f = 1 - (k + 0.5) / n if side in ("left", "right") else (k + 0.5) / n
                if side == "left":
                    e0, e1 = (x, y + f * h), (x - stub, y + f * h)
                    self.text((x + 0.1, e0[1]), str(name), "left", "center", self.fs - 1.5)
                    self.text((x - 0.08, e0[1] + 0.17), str(pn), "right", "bottom", self.fs - 3, "#777777")
                elif side == "right":
                    e0, e1 = (x + w, y + f * h), (x + w + stub, y + f * h)
                    self.text((x + w - 0.1, e0[1]), str(name), "right", "center", self.fs - 1.5)
                    self.text((x + w + 0.08, e0[1] + 0.17), str(pn), "left", "bottom", self.fs - 3, "#777777")
                elif side == "top":
                    e0, e1 = (x + f * w, y + h), (x + f * w, y + h + stub)
                    self.text((e0[0], e0[1] - 0.1), str(name), "center", "top", self.fs - 1.5)
                    self.text((e0[0] + 0.1, e0[1] + 0.08), str(pn), "left", "bottom", self.fs - 3, "#777777")
                else:
                    e0, e1 = (x + f * w, y), (x + f * w, y - stub)
                    self.text((e0[0], e0[1] + 0.1), str(name), "center", "bottom", self.fs - 1.5)
                    self.text((e0[0] + 0.1, e0[1] - 0.08), str(pn), "left", "top", self.fs - 3, "#777777")
                self.d += elm.Line().at(e0).to(e1)
                res[str(pn)] = e1
                if pn in skip:
                    if net != "NC":
                        self.nets.add(net)
                else:
                    self._pinlabel(side, e1, net)
        v = CK.PARTS[ref]["value"]
        ttl = v if title is None else title
        if ttl != "":
            self.text((x + w / 2, y + h + (stub + 0.45 if top else 0.45)), f"{ref}  {ttl}", "center", "bottom", self.fs, C_TXT, "bold")
        else:
            self.text((x + w / 2, y + h / 2), ref, "center", "center", self.fs, C_TXT, "bold")
        if subtitle:
            self.text((x + w / 2, y - (stub + 0.2 if bottom else 0.2)), subtitle, "center", "top", self.fs - 2, C_NOTE)
        return res

    def _pinlabel(self, side, xy, net):
        x, y = xy
        if net == "NC":
            self.text((x + (-0.15 if side == "left" else 0.15), y), "x NC" if side == "right" else "NC x", "right" if side == "left" else "left", "center", self.fs - 2, "#aa0000")
            return
        self.nets.add(net)
        if side == "left":
            self.text((x - 0.1, y), net, "right", "center", self.fs - 1.5, C_NET, "bold")
        elif side == "right":
            self.text((x + 0.1, y), net, "left", "center", self.fs - 1.5, C_NET, "bold")
        elif side == "top":
            self.text((x, y + 0.12), net, "center", "bottom", self.fs - 1.5, C_NET, "bold", rot=90)
        else:
            self.text((x, y - 0.12), net, "center", "top", self.fs - 1.5, C_NET, "bold", rot=90)

    # ---------------------------------------------------------------- save
    def save(self, dpi=110):
        base = os.path.join(OUTDIR, self.name)
        for ext in ("svg", "pdf", "png"):
            self.d.save(f"{base}.{ext}", dpi=dpi) if ext == "png" else self.d.save(f"{base}.{ext}")
        with open(base + ".json", "w") as f:
            json.dump({"sheet": self.name, "title": self.title, "covers": self.covers, "refs": sorted(self.refs),
                       "nets": sorted(self.nets), "values": self.values}, f, indent=1)
        print(f"{self.name}: {len(self.refs)} refs, {len(self.nets)} nets drawn")


# ------------------------------------------------------------------ netlist-style helpers (analogue sheets)
def _branch(self, x, y, ref, a, b, kind="R", extra="", plen=3.0, rec=True):
    """row:  NET_A --[part]-- NET_B   (nets joined by name)."""
    cls = {"R": elm.Resistor, "C": elm.Capacitor, "D": elm.Diode, "S": elm.Schottky, "L": elm.LED, "F": elm.Fuse, "Z": elm.Zener}[kind]
    self.net((x + 2.6, y), a, ha="right", size=7)
    self.wire((x + 2.7, y), (x + 3.3, y))
    self.reg(ref)
    self.d += cls().at((x + 3.3, y)).to((x + 3.3 + plen, y))
    v = CK.PARTS[ref]["value"]
    self.text((x + 3.3 + plen / 2, y + 0.42), f"{ref}  {v}" + (f"  {extra}" if extra else ""), "center", "bottom", self.fs - 1)
    self.wire((x + 3.3 + plen, y), (x + 3.9 + plen, y))
    self.net((x + 4.0 + plen, y), b, ha="left", size=7)


def _amp(self, cx, cy, ref, sub, plus, minus, out, pins):
    """op-amp symbol with net names at the terminals. pins = (out_pin, minus_pin, plus_pin)."""
    self.reg(ref)
    e = elm.Opamp().at((cx, cy)).anchor("center").theta(0)
    self.d += e
    a1, a2, ao = e.absanchors["in1"], e.absanchors["in2"], e.absanchors["out"]
    self.wire((a1.x, a1.y), (a1.x - 0.6, a1.y)); self.wire((a2.x, a2.y), (a2.x - 0.6, a2.y)); self.wire((ao.x, ao.y), (ao.x + 0.6, ao.y))
    self.nets.update({minus, plus, out})
    self.net((a1.x - 0.7, a1.y), f"{minus}  (-)  pin {pins[1]}", ha="right", size=7, record=False)
    self.net((a2.x - 0.7, a2.y), f"{plus}  (+)  pin {pins[2]}", ha="right", size=7, record=False)
    self.net((ao.x + 0.7, ao.y), f"{out}  pin {pins[0]}", ha="left", size=7, record=False)
    self.text((cx + 0.1, cy), f"{ref}{sub}", "center", "center", self.fs, C_TXT, "bold")


Sheet.branch = _branch
Sheet.amp = _amp


def _tprow(self, x, y, ref, net):
    self.reg(ref)
    self.d += elm.Dot(open=True).at((x + 2.7, y))
    self.text((x + 2.5, y), f"{ref}", "right", "center", 6.5, "#cc0000")
    self.net((x + 2.95, y), net, ha="left", size=7)


Sheet.tprow = _tprow


def _fet(self, ref, gx, gy):
    """N-MOSFET drawn with plain lines (gate left, drain up, source down). returns dict of anchors."""
    self.reg(ref)
    L = lambda a, b: self.d.__iadd__(elm.Line().at(a).to(b))
    L((gx, gy), (gx + 0.5, gy))
    L((gx + 0.5, gy - 0.55), (gx + 0.5, gy + 0.55))
    for yy in (-0.55, 0.0, 0.55):
        pass
    L((gx + 0.7, gy + 0.55), (gx + 0.7, gy + 0.3)); L((gx + 0.7, gy + 0.15), (gx + 0.7, gy - 0.15)); L((gx + 0.7, gy - 0.3), (gx + 0.7, gy - 0.55))
    L((gx + 0.7, gy + 0.45), (gx + 1.4, gy + 0.45)); L((gx + 1.4, gy + 0.45), (gx + 1.4, gy + 1.0))
    L((gx + 0.7, gy - 0.45), (gx + 1.4, gy - 0.45)); L((gx + 1.4, gy - 0.45), (gx + 1.4, gy - 1.0))
    L((gx + 0.7, gy), (gx + 1.4, gy)); L((gx + 1.4, gy), (gx + 1.4, gy - 0.45))
    L((gx + 0.9, gy + 0.12), (gx + 0.75, gy)); L((gx + 0.9, gy - 0.12), (gx + 0.75, gy))
    return {"gate": (gx, gy), "drain": (gx + 1.4, gy + 1.0), "source": (gx + 1.4, gy - 1.0)}


Sheet.fet = _fet
