#!/usr/bin/env python3
"""Sheet 01: power input, protection, charge limiting, grounding, 5 V / 3.3 V filtering, rail sense."""
import schemdraw.elements as elm
from sd import Sheet

S = Sheet("01_power_grounding", "01  Power input, protection, grounding (control board)", ["power"])
S.title_block(0, 14.0, "12 V DC in -> fuse -> reverse-polarity Schottky -> TVS -> bulk -> 5.5 ohm charge limit per strobe module.  PGND (strobe return) and GND_L (logic) meet only at JP5.")
YR, YG = 10.75, 4.0


def tp(ref, xy, lab_xy):
    S.reg(ref); S.d += elm.Dot(open=True).at(xy); S.text(lab_xy, ref, "center", "center", 6.5, "#cc0000")


def gl(x, y, lab="GND_L"):
    S.d += elm.Ground().at((x, y)); S.net((x + 0.3, y - 0.45), lab, size=6.5)


p = S.ic("J1", 0, 8.5, 2.4, 3.0, right=[(1, "+12V", "VIN_RAW"), (2, "GND", "PGND")], title="2P screw  12 V DC in", skip=(1, 2))
S.net((3.0, YR + 0.3), "VIN_RAW", size=7)
S.wire((6.8, YR), (7.4, YR))
S.wire(p["1"], (4.2, YR))
S.F("F1", (4.2, YR), (6.8, YR), extra="2 A time-lag")
S.net((7.0, YR + 0.3), "VIN_F", size=7)
S.D("D1", (7.4, YR), (10.2, YR), kind="schottky", extra="reverse-polarity block")
S.wire((10.2, YR), (30.0 - 12, YR)); S.dot((11.0, YR)); S.net((10.9, YR + 0.35), "V12", size=8)
S.wire(p["2"], (3.5, p["2"][1]), (3.5, YG)); S.wire((3.5, YG), (26.0, YG)); S.net((4.0, YG - 0.4), "PGND", size=8)
S.D("D2", (12.2, YG), (12.2, YR), kind="zener", side="right", extra="TVS 15 V"); S.dot((12.2, YR)); S.dot((12.2, YG))
S.C("C1", (14.4, YR), (14.4, YG), polar=True, side="right"); S.dot((14.4, YR)); S.dot((14.4, YG))
S.C("C2", (16.4, YR), (16.4, YG), side="right"); S.dot((16.4, YR)); S.dot((16.4, YG))
tp("TP1", (17.6, YR), (17.6, YR + 0.6)); tp("TP2", (5.0, YG), (5.0, YG + 0.6))
# charge limiting
S.wire((18.2, YR), (18.2, 5.75)); S.dot((18.2, YR))
for ref, y in (("R1", 10.75), ("R2", 9.25), ("R3", 7.25), ("R4", 5.75)):
    S.dot((18.2, y)); S.R(ref, (18.2, y), (22.2, y), extra="")
    S.dot((22.2, y))
S.wire((22.2, 10.75), (22.2, 9.25)); S.wire((22.2, 7.25), (22.2, 5.75))
S.net((22.5, 10.0), "V12_A  -> J2 (sheet 04)", size=7); S.net((22.5, 6.5), "V12_B  -> J4 (sheet 04)", size=7)
S.note((18.2, 12.3), "charge limit: R1||R2 = 5.5 ohm (module A), R3||R4 = 5.5 ohm (module B)\nsags the rail ~0.7 V per 0.1 A average; caps the fault current (FET short) at ~1.4 A", size=6.5)
# star
j5 = S.ic("JP5", 26.0, 2.3, 2.0, 1.4, left=[(1, "1", "PGND")], right=[(2, "2", "GND_L")], title="STAR link  (single point)", skip=(1, 2))
S.wire((25.5, YG), (25.5, j5["1"][1]))
gl(j5["2"][0], j5["2"][1])
tp("TP5", (j5["2"][0] + 1.0, j5["2"][1]), (j5["2"][0] + 1.0, j5["2"][1] + 0.5))
S.net((26.0, 1.0), "only connection between PGND and GND_L", size=6.5, record=False)
# 5 V / 3.3 V filtering (right)
X0 = 33.0
S.net((X0 - 3.3, YR), "V5 (Pico VBUS)", ha="right", size=7)
S.wire((X0 - 3.2, YR), (X0, YR))
S.C("C10", (X0 - 1.6, YR), (X0 - 1.6, 8.2), polar=True, side="left"); S.dot((X0 - 1.6, YR)); gl(X0 - 1.6, 8.2)
S.R("R5", (X0, YR), (X0 + 2.6, YR), extra="analogue filter"); S.dot((X0 + 3.4, YR)); S.wire((X0 + 2.6, YR), (X0 + 3.4, YR))
S.net((X0 + 3.1, YR + 0.35), "V5A", size=7)
S.wire((X0 + 3.4, YR), (X0 + 6.4, YR))
S.C("C4", (X0 + 4.0, YR), (X0 + 4.0, 8.2), polar=True, side="right"); S.dot((X0 + 4.0, YR)); gl(X0 + 4.0, 8.2)
S.C("C5", (X0 + 5.8, YR), (X0 + 5.8, 8.2), side="right"); S.dot((X0 + 5.8, YR)); gl(X0 + 5.8, 8.2)
Y2 = 6.2
S.net((X0 - 0.2, Y2), "V5 (Pico VBUS)", ha="right", size=7)
S.R("R6", (X0, Y2), (X0 + 2.6, Y2), extra="emitter filter"); S.wire((X0 + 2.6, Y2), (X0 + 6.4, Y2)); S.dot((X0 + 3.4, Y2))
S.net((X0 + 3.1, Y2 + 0.35), "V5E", size=7)
S.C("C6", (X0 + 4.0, Y2), (X0 + 4.0, 3.6), polar=True, side="right"); S.dot((X0 + 4.0, Y2)); gl(X0 + 4.0, 3.6)
S.C("C7", (X0 + 5.8, Y2), (X0 + 5.8, 3.6), side="right"); S.dot((X0 + 5.8, Y2)); gl(X0 + 5.8, 3.6)
Y3 = 1.0
S.net((X0 - 0.2, Y3), "3V3 (Pico 3V3 OUT)", ha="right", size=7)
S.wire((X0, Y3), (X0 + 10.0, Y3))
for ref, x, pol in (("C8", X0 + 1.2, True), ("C9", X0 + 3.4, False)):
    S.C(ref, (x, Y3), (x, -1.6), polar=pol, side="right"); S.dot((x, Y3)); gl(x, -1.6)
tp("TP3", (X0 + 8.6, Y3), (X0 + 8.6, Y3 + 0.5)); tp("TP4", (X0 + 6.4, YR), (X0 + 6.4, YR + 0.5))
S.nets.update({"V5"})
# rail sense
S.title_block(0, -2.2, "") if False else None
j6 = S.ic("JP6", 0.0, -6.0, 2.6, 3.0, left=[(1, "A", "SENSE_A"), (3, "B", "SENSE_B")], right=[(2, "common", "RAIL_SEL")], title="3P header: 1-2 = module A, 2-3 = module B", skip=())
S.wire(j6["2"], (6.0, j6["2"][1]))
S.R("R7", (6.0, j6["2"][1]), (9.0, j6["2"][1]))
yy = j6["2"][1]
S.wire((9.0, yy), (14.0, yy)); S.dot((10.0, yy)); S.net((9.3, yy + 0.35), "RAIL_SENSE", size=7)
S.R("R8", (10.0, yy), (10.0, yy - 2.8), side="right"); gl(10.0, yy - 2.8)
S.C("C11", (11.8, yy), (11.8, yy - 2.8), side="right"); S.dot((11.8, yy)); gl(11.8, yy - 2.8)
S.D("D4", (13.0, yy), (13.0, yy + 2.6), kind="schottky", side="right", extra="clamp"); S.dot((13.0, yy)); S.net((13.0, yy + 2.9), "3V3", ha="center", size=7)
S.net((14.3, yy), "-> Pico GP28/ADC2 (U1.34)", size=7)
tp("TP6", (14.0, yy), (14.0, yy - 0.5))
S.note((0, -8.2), "12 V bank -> 33k/10k divider = 0.2326 : 12 V -> 2.79 V, 13.5 V -> 3.14 V (RP2040 ADC full scale 3.3 V); rail value = ADC * 4.30.\n"
                   "Source impedance 7.7 k with 100 nF at the pin; the 100 ohm in the module sense feed (R106/R206) adds 100 ohm (negligible).", size=6.5)
S.save()
