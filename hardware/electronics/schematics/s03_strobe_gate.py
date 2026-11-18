#!/usr/bin/env python3
"""Sheet 03: strobe module gate driver, AC-coupled gate (max on-time cut-off), driver supply (module A)."""
import schemdraw.elements as elm
from sd import Sheet

S = Sheet("03_strobe_gate", "03  Strobe module - gate driver and hardware max-on-time cut-off (module A; B identical)", ["strobe"])
S.title_block(0, 17.2, "TC4427 (both channels paralleled) drives the MOSFET through an AC-coupling capacitor: a stuck-high GATE_A decays and switches the MOSFET off after ~55 us")


def gs(x, y, lab="SRC_A"):
    S.d += elm.Ground().at((x, y))
    S.net((x + 0.3, y - 0.45), lab, size=6.5)


# input
S.net((-0.2, 7.4), "GATE_A  (J103.1, from control board)", ha="right", size=7)
S.R("R102", (0.3, 7.4), (3.2, 7.4), extra="input damping")
S.wire((3.2, 7.4), (6.5, 7.4)); S.dot((5.2, 7.4))
S.R("R103", (5.2, 7.4), (5.2, 6.0), side="left", extra="pull-down"); gs(5.2, 6.0)
S.reg("TP106"); S.d += elm.Dot(open=True).at((5.2, 8.3)); S.wire((5.2, 7.4), (5.2, 8.3)); S.text((5.45, 8.5), "TP106", "left", "center", 6.5, "#cc0000")
S.net((5.3, 7.7), "DRV_IN_A", size=7)
p = S.ic("U101", 7, 4.0, 4.0, 5.5,
         left=[(1, "NC", "NC"), (2, "INA", "DRV_IN_A"), (3, "GND", "SRC_A"), (4, "INB", "DRV_IN_A")],
         right=[(7, "OUTA", "GOUT_A"), (5, "OUTB", "GOUT_A"), (8, "NC", "NC")],
         top=[(6, "VDD", "VDRV_A")], skip=(2, 3, 6, 7), subtitle="dual driver, channels paralleled")
gs(p["3"][0], p["3"][1])
S.wire(p["6"], (p["6"][0], 12.0))
# driver supply
S.net((7.4, 14.8), "V12_A", size=8)
S.R("R101", (9.0, 14.5), (9.0, 12.0), side="right", extra="supply filter")
S.wire((9.0, 12.0), (15.4, 12.0)); S.dot((9.0, 12.0)); S.net((9.5, 12.25), "VDRV_A", size=7)
S.C("C108", (11.4, 12.0), (11.4, 10.4), side="right"); S.dot((11.4, 12.0)); gs(11.4, 10.4)
S.C("C109", (13.0, 12.0), (13.0, 10.4), side="right"); S.dot((13.0, 12.0)); gs(13.0, 10.4)
S.D("D102", (14.8, 10.4), (14.8, 12.0), kind="zener", side="right"); S.dot((14.8, 12.0)); gs(14.8, 10.4)
# output chain
S.wire(p["7"], (12.0, p["7"][1]))
S.R("R104", (12.0, p["7"][1]), (14.6, p["7"][1]), extra="gate R"); S.dot((15.0, p["7"][1]))
S.wire((14.6, p["7"][1]), (15.4, p["7"][1]))
S.net((15.2, p["7"][1] + 0.3), "GACI_A", size=7)
yg = p["7"][1]
S.C("C110", (15.4, yg), (18.3, yg), extra="AC coupling"); 
S.wire((18.3, yg), (22.0, yg)); S.dot((19.0, yg))
S.net((18.4, yg + 0.3), "GFET_A", size=7)
S.R("R105", (19.0, yg), (19.0, yg - 2.6), side="right", extra="gate bias\ntau ~ 40 us"); gs(19.0, yg - 2.6)
S.D("D103", (20.6, yg - 2.6), (20.6, yg), kind="schottky", side="right", extra="DC restore"); S.dot((20.6, yg)); gs(20.6, yg - 2.6)
S.reg("TP104"); S.d += elm.Dot(open=True).at((19.0, yg + 1.0)); S.wire((19.0, yg), (19.0, yg + 1.0)); S.text((19.25, yg + 1.15), "TP104", "left", "center", 6.5, "#cc0000")
fq = S.fet("Q101", 22.0, yg)
dx_, dy_ = fq["drain"]; sx_, sy_ = fq["source"]
S.wire((dx_, dy_), (dx_, dy_ + 0.8)); S.net((dx_ + 0.2, dy_ + 0.8), "DRN_A  (sheet 02)", size=7)
S.wire((sx_, sy_), (sx_, sy_ - 0.8)); gs(sx_, sy_ - 0.8)
S.text((sx_ + 0.6, yg), "Q101\nIRLB8721", "left", "center", 8)
S.note((0, 3.0), "Kelvin source: the driver ground (U101.3), R103, R105, D103 and the decoupling all return to SRC_A at the MOSFET source pin, not to PGND\n"
                 "(PGND is 10 mohm / ~0.18 V away during a pulse).  SRC_A -> RS101 -> PGND is drawn on sheet 02.\n"
                 "Numbers: C110 10 nF, R105 3k3 -> tau = R*(C110 + Ceff) ~ 39 us; gate step ~10.3 V; gate < 2.5 V after ~55 us (ngspice: current falls to 10 % after 61 us).\n"
                 "Gate edge 10-90 % ~ 22-31 ns (sim); a bare 3.3 V Pico pin would need >= 400 ns and cannot pull the gate negative.\n"
                 "D102 15 V zener protects the 18 V-max driver supply from TVS clamp transients; U101 pins 1 and 8 are not connected (NC).", size=7)
S.save()
