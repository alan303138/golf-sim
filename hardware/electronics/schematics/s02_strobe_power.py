#!/usr/bin/env python3
"""Sheet 02: strobe module power stage (module A shown; module B = same with refs +100 and nets _B)."""
import schemdraw.elements as elm
from sd import Sheet, CK

S = Sheet("02_strobe_power", "02  Strobe module - power stage (module A; B is identical, refs +100, nets _A -> _B)", ["strobe"])
S.title_block(0, 16.6, "12 V rail -> low-ESR capacitor bank -> ballast resistors -> 3 x 850 nm LED per string -> flyback-clamped low-side MOSFET -> 10 mohm sense -> PGND")
YR, YG = 13.25, -1.0          # rail and PGND rail
# --- connectors
p = S.ic("J102", 0, 11.3, 2.6, 2.6, right=[(1, "V12_A in", "V12_A"), (2, "PGND", "PGND")], title="2P screw", skip=(1, 2),
         subtitle="from control board J2")
S.wire(p["1"], (36.2, YR)); S.net((3.4, YR + 0.25), "V12_A", size=8)
S.wire(p["2"], (3.6, p["2"][1]), (3.6, YG)); S.wire((3.6, YG), (37.0, YG)); S.net((3.7, YG - 0.35), "PGND", size=8)
S.dot((3.6, p["2"][1]))
# --- bank
for ref, x, pol in (("C101", 5.2, True), ("C102", 7.0, True)):
    S.C(ref, (x, YR), (x, YG), polar=pol, side="right"); S.dot((x, YR)); S.dot((x, YG))
S.C("C103", (8.9, YR), (8.9, YG), side="right", lab="C103-C106 (4x)\n10u 25V X7R\nHF bank", also=("C104", "C105", "C106")); S.dot((8.9, YR)); S.dot((8.9, YG))
S.C("C107", (11.1, YR), (11.1, YG), side="right"); S.dot((11.1, YR)); S.dot((11.1, YG))
S.R("R108", (12.9, YR), (12.9, YG), side="right", extra="bleeder"); S.dot((12.9, YR)); S.dot((12.9, YG))
# charged LED
S.R("R107", (14.9, YR), (14.9, 8.2), side="right"); S.dot((14.9, YR))
S.D("D104", (14.9, 8.2), (14.9, YG), kind="led", side="right", extra="BANK CHARGED"); S.dot((14.9, YG))
S.net((14.3, 8.2), "CHGLED_A", ha="right")
# sense + J103
S.R("R106", (19.4, YR), (19.4, 7.55), side="left", extra="rail sense"); S.dot((19.4, YR))
p3 = S.ic("J103", 16.9, 5.4, 2.6, 1.6, top=[(1, "GATE_A", "GATE_A"), (2, "PGND", "PGND"), (3, "SENSE_A", "SENSE_A")], title="3P header", skip=(1, 2, 3),
          subtitle="to control board J3")
S.net((p3["1"][0] - 0.0, p3["1"][1] + 0.12), "GATE_A -> sheet 03", ha="left", size=7)
S.net((p3["2"][0] + 0.15, p3["2"][1] + 0.3), "PGND", size=7)
S.net((p3["3"][0] + 0.2, p3["3"][1] + 0.5), "SENSE_A", size=7)
# strings
x0, w = 21.0, 9.0
xs = [x0 + 1.5 * (k + 0.5) for k in range(6)]
j104 = S.ic("J104", x0, 9.6, w, 0.9, top=[(k + 1, f"S{k+1}", f"STR_A{k+1}") for k in range(6)], title="", skip=tuple(range(1, 7)))
S.text((x0 + w + 0.3, 10.05), "J104 6P screw\nstring anodes", "left", "center", 7)
j105 = S.ic("J105", x0, 3.4, w, 0.9, top=[(k + 1, f"R{k+1}", "DRN_A") for k in range(6)], title="", skip=tuple(range(1, 7)))
S.text((x0 + w + 0.3, 3.85), "J105 6P screw\nreturns = DRN_A", "left", "center", 7)
ledno = 101
for k, x in enumerate(xs):
    S.R(f"RB{101+k}", (x, YR), (x, j104[str(k + 1)][1]), side="right", extra="" if k else "ballast"); S.dot((x, YR))
    S.net((x - 0.12, j104[str(k + 1)][1] + 0.35), f"STR_A{k+1}", ha="right", size=6)
    top = j104[str(k + 1)][1]
    ytop = 9.6 - 0.1
    S.wire((x, 9.6), (x, 9.25))
    ys = [9.25, 7.95, 6.65, 5.35]
    for j in range(3):
        ref = f"LD{ledno}"
        S.D(ref, (x, ys[j]), (x, ys[j + 1]), kind="led", lab=ref, side="left", dx=-0.1); ledno += 1
        if j < 2:
            pass
    S.wire((x, 5.35), (x, j105[str(k + 1)][1]))
    S.wire((x, 3.4), (x, 3.0))
S.nets.update({f"LA{k+1}_{j}" for k in range(6) for j in (1, 2)})
S.wire((xs[0], 3.0), (33.4, 3.0)); S.net((xs[0] - 0.2, 3.0), "DRN_A", ha="right", size=8)
S.dash_box(x0 - 0.3, 5.0, x0 + w + 0.3, 9.4, "LED side board: 6 strings x 3 LEDs (850 nm, 20 mm stars) wired to J104 / J105, keep wires <= 15 cm")
# flyback, FET, sense
S.D("D101", (33.4, 3.0), (33.4, YR), kind="schottky", side="right", extra="flyback"); S.dot((33.4, 3.0)); S.dot((33.4, YR))
fq = S.fet("Q101", 34.4, 2.0)
dxq, dyq = fq["drain"]; sx, sy = fq["source"]; gx, gy = fq["gate"]
S.wire((33.4, 3.0), (dxq, 3.0)); S.wire((dxq, dyq), (dxq, 3.0))
S.text((dxq + 0.5, 3.6), "Q101\nIRLB8721", "left", "center", 8)
S.wire((sx, sy), (sx, YG + 0.0))
S.R("RS101", (sx, sy - 0.1), (sx, YG), side="right"); S.net((sx + 0.3, sy - 0.35), "SRC_A", size=7)
S.dot((sx, YG))
S.net((gx - 0.1, gy + 0.3), "GFET_A", ha="right", size=7)
S.net((gx - 0.1, gy - 0.3), "(gate network: sheet 03)", ha="right", size=6, record=False)
# test points
for ref, xy, lab in (("TP101", (4.3, YR), (4.3, YR + 0.9)), ("TP102", (22.0, 3.0), (22.0, 2.4)), ("TP103", (sx + 1.6, sy), (sx + 1.9, sy + 0.3)), ("TP105", (4.4, YG), (4.4, YG + 0.7))):
    S.reg(ref); S.d += elm.Dot(open=True).at(xy)
    S.text(lab, ref, "center", "center", 6.5, "#cc0000")
S.wire((4.3, YR), (4.3, YR))
S.wire((22.0, 3.0), (22.0, 3.0))
S.wire((sx, sy), (sx + 1.6, sy))
# notes
S.note((0, 2.6), "Design point: 12 V rail, 3 LED/string, 1R8 ballast -> ~3.0 A per string; 6 strings -> ~18 A pulse per module (12 A with 4 strings).\n"
                 "Pulse <= 20 us, avg duty <= 2 %, <= 10 pulses/exposure (firmware).  Bank 2x2200 uF: droop ~0.4 V in the worst burst (sim).\n"
                 "HIGH-CURRENT LOOP: C103-C106 -> RBn -> LEDs -> D(drain) -> Q101 -> RS101 -> back to C103-C106 (keep < 2 cm of trace, wide / plane).\n"
                 "Charge limiting is on the control board (R1+R2 for A, R3+R4 for B = 5.5 ohm).", size=7)
S.save()
