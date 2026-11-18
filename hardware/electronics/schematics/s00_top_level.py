#!/usr/bin/env python3
"""Sheet 00: top-level block / wiring diagram."""
import schemdraw.elements as elm
from sd import Sheet

S = Sheet("00_top_level", "00  Top level: phase-1 timing / strobe electronics", [])
S.title_block(0, 25.0, "Pi 5 -USB- Pico (control board) -> camera triggers, two strobe modules (LED side boards), shot-trigger sensor.  Ground: PGND (strobe return) and GND_L (logic) meet only at JP5.")


def box(x, y, w, h, title, lines=(), color="#000000", fill=None):
    pts = [(x, y), (x + w, y), (x + w, y + h), (x, y + h), (x, y)]
    for a, b in zip(pts[:-1], pts[1:]):
        S.d += elm.Line().at(a).to(b).color(color).linewidth(1.6)
    S.text((x + 0.25, y + h - 0.35), title, "left", "center", 9, "#000000", "bold")
    for i, l in enumerate(lines):
        S.text((x + 0.25, y + h - 1.1 - 0.5 * i), l, "left", "center", 7)


def arrow(p, q, label=None, lp=None, color="#0b5394", ha="center"):
    S.d += elm.Line().at(p).to(q).color(color).linewidth(1.3)
    if label:
        S.nets.update(l.strip() for l in label.split("\n") if l.strip() and l.strip().replace("_", "").isalnum() and l.strip().isupper() and False)
        S.text(lp or ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2 + 0.35), label, ha, "bottom", 6.5, color)


# blocks
box(0, 17, 6.5, 4.8, "Raspberry Pi 5", ["host, own 27 W supply", "CSI0 / CSI1 -> 2x IMX296", "USB -> Pico"])
box(0, 9.5, 6.5, 6.0, "IMX296 camera x2", ["mono global-shutter MIPI modules", "external trigger input", "(level: VERIFY, 1.8 V?)", "850 nm band-pass filters", "lens f/2 .. f/2.8"])
box(9.5, 2.0, 15.5, 19.8, "CONTROL BOARD  (perfboard)  - netlist refs 1..99", [])
box(10.0, 17.0, 6.8, 3.6, "U1 Raspberry Pi Pico", ["GP2 CAM_TRIG   GP3 STROBE_GATE", "GP4 STROBE_EN  GP5 TRIG_IN", "GP6 STATUS      GP28 RAIL_SENSE"])
box(17.4, 17.0, 7.2, 3.6, "05 gating + interlock", ["U2 74HC08: GATE_BUS = GP3 AND EN_SW", "J8 interlock loop, D7 ARMED LED", "R22/R23 damping to modules"])
box(10.0, 12.6, 6.8, 3.8, "04 camera trigger", ["U3 LVC2G17 + U4 1.8 V LDO", "JP1: 1.8 V (default) / 3.3 V", "R12/R13 47R, D5/D6 clamps", "J6, J7 -> cameras"])
box(17.4, 12.6, 7.2, 3.8, "06/07/08 shot trigger", ["acoustic: M1, U5, U7, U8", "optical: U9, Q3, U6, U8, J11", "JP2 select, U10+JP3 polarity", "-> TRIG_IN (GP5)"])
box(10.0, 7.2, 6.8, 4.8, "01 power / grounding", ["J1 12 V, F1 2 A, D1, D2 TVS, C1", "R1..R4 charge limit 5.5 ohm each", "V5A / V5E filters, JP5 STAR", "JP6 + R7/R8: rail sense GP28"])
box(17.4, 7.2, 7.2, 4.8, "connectors to modules", ["J2/J3 -> module A (V12_A, GATE_A,", "SENSE_A, PGND)", "J4/J5 -> module B", "(twist GATE with PGND)"])
box(10.0, 2.6, 14.6, 3.8, "test points", ["TP1..TP24 (see README table)"])
box(28.5, 14.0, 9.5, 7.8, "STROBE MODULE A  (perfboard)  refs 101..", ["J102 V12_A, J103 GATE_A/SENSE_A", "C101..C107 bank, RB101..106", "Q101 IRLB8721 + U101 TC4427", "RS101 10 mohm, D101 flyback", "3 A per string, 6 strings -> 18 A"])
box(28.5, 4.0, 9.5, 7.8, "STROBE MODULE B  (perfboard)  refs 201..", ["identical to A (refs +100)", "J202 V12_B, J203 GATE_B/SENSE_B", "", "", ""])
box(40.5, 14.0, 7.5, 7.8, "LED side board A", ["18 x 850 nm LED (6 strings x 3)", "on Al plate ~110x90 mm", "TIR lens 60 deg (40 deg opt.)", "J104 anodes / J105 returns"])
box(40.5, 4.0, 7.5, 7.8, "LED side board B", ["same as A", "mounted on the other side", "of the camera pair"])
box(0, 2.0, 6.5, 5.2, "Shot-trigger sensor head", ["electret capsule (acoustic)", "or 940 nm LED + photodiode", "(optical, remote head, J11)"])
box(0, 22.3, 6.5, 1.5, "12 V DC supply 12 V / 2 A min", [])
# wires
arrow((6.5, 19.4), (10.0, 19.4), "USB (VBUS 5 V, D+/D-)", (6.7, 19.55), ha="left")
arrow((6.5, 12.0), (10.0, 13.6), "J6 / J7: CAM1_OUT, CAM2_OUT + GND_L", (6.0, 11.0), ha="left")
arrow((24.6, 18.5), (28.5, 18.5), "GATE_A  (J3)", (24.7, 18.65), ha="left")
arrow((24.6, 9.0), (28.5, 8.0), "GATE_B (J5)", (24.7, 9.2), ha="left")
arrow((24.6, 10.5), (28.5, 16.0), "V12_A (J2)", (24.7, 12.2), ha="left")
arrow((38.0, 18.0), (40.5, 18.0), "J104/J105", (38.1, 18.15), ha="left")
arrow((38.0, 8.0), (40.5, 8.0), "J204/J205", (38.1, 8.15), ha="left")
arrow((6.5, 4.6), (10.0, 14.2), "MIC_P/GND_L or J11", (3.0, 7.6), ha="left")
arrow((3.25, 22.3), (3.25, 21.8), "", None)
S.note((28.5, 2.4), "Key numbers (sim): ~3.0 A/string, 18 A/module pulse, current rise ~110 ns, droop <= 6 % flux over a worst burst,\n"
                    "AC-coupled gate cut-off ~55 us, charge limit 5.5 ohm per module, ~2 W average LED power per module at 2 % duty.", size=7)
S.save()
