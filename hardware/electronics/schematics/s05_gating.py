#!/usr/bin/env python3
"""Sheet 05: Pico socket pin map, strobe gating (AND with interlock), interlock, ARMED / status LEDs, module connectors."""
import schemdraw.elements as elm
from sd import Sheet, CK

S = Sheet("05_gating_interlock", "05  Pico socket, strobe gating, interlock and indicators (control board)", ["pico", "gating"])
S.title_block(0, 20.5, "GATE_BUS = GP3 AND EN_SW (hardware).  EN_SW = interlock loop closed.  ARMED LED is driven from EN_SW, not from firmware.  Fixed Pico pin map: GP2 CAM_TRIG, GP3 STROBE, GP4 STROBE_EN, GP5 TRIG_IN, GP6 status, GP28 rail sense.")


def gl(x, y, lab="GND_L"):
    S.d += elm.Ground().at((x, y)); S.net((x + 0.3, y - 0.45), lab, size=6.5)


def tp(ref, xy, lab_xy):
    S.reg(ref); S.d += elm.Dot(open=True).at(xy); S.text(lab_xy, ref, "center", "center", 6.5, "#cc0000")


# ---- Pico (real footprint order: pins 1-20 left top->bottom, 40-21 right top->bottom)
pp = CK.PARTS["U1"]["pins"]
left = [(n, pp[str(n)]["name"], pp[str(n)]["net"]) for n in range(1, 21)]
right = [(n, pp[str(n)]["name"], pp[str(n)]["net"]) for n in range(40, 20, -1)]
S.ic("U1", 4.0, 3.0, 5.0, 13.2, left=left, right=right, title="Raspberry Pi Pico  (USB at the top; powered from the Pi 5 over USB)", stub=0.4)
# ---- gating
Y1 = 15.8
S.net((16.8, Y1), "STROBE_GATE", ha="right", size=7)
S.wire((17.0, Y1), (18.5, Y1)); tp("TP12", (17.6, Y1), (17.6, Y1 + 0.55))
S.R("R14", (18.5, Y1), (20.3, Y1), extra="")
S.wire((20.3, Y1), (22.0, Y1)); S.dot((21.0, Y1)); S.net((20.5, Y1 + 0.5), "GATE_RAW", size=6.5)
S.R("R15", (21.0, Y1), (21.0, 13.0), side="left", extra="keep low in reset"); gl(21.0, 13.0)
u = S.ic("U2", 22.5, 8.3, 4.0, 8.0,
         left=[(1, "1A", "GATE_RAW"), (2, "1B", "EN_SW"), (4, "2A", "GND_L"), (5, "2B", "GND_L"), (9, "3A", "GND_L"), (10, "3B", "GND_L"), (12, "4A", "GND_L"), (13, "4B", "GND_L")],
         right=[(3, "1Y", "GATE_BUS"), (6, "2Y", "NC"), (8, "3Y", "NC"), (11, "4Y", "NC")],
         top=[(14, "VCC", "3V3")], bottom=[(7, "GND", "GND_L")], skip=(1, 3), title="74HC08", subtitle="unused gates: inputs tied low, outputs NC")
gl(*u["7"])
yg = u["3"][1]
S.wire(u["3"], (29.0, yg)); S.dot((28.2, yg)); S.net((27.0, yg + 0.35), "GATE_BUS", size=7)
tp("TP10", (28.2, yg + 0.7), (28.2, yg + 1.2)); S.wire((28.2, yg), (28.2, yg + 0.7))
S.R("R22", (29.0, yg), (31.8, yg), extra="damping"); S.wire((31.8, yg), (33.2, yg)); S.net((32.0, yg + 0.5), "GATE_A", size=7)
S.wire((28.2, yg), (28.2, yg - 3.8)); S.dot((28.2, yg - 3.8))
yb = yg - 3.8
S.R("R23", (28.2, yb), (31.8, yb), extra="damping"); S.wire((31.8, yb), (33.2, yb)); S.net((32.0, yb + 0.5), "GATE_B", size=7)
# module connectors
j3 = S.ic("J3", 33.7, yg - 2.5, 2.4, 3.0, left=[(1, "GATE", "GATE_A"), (2, "PGND", "PGND"), (3, "SENSE", "SENSE_A")], title="3P hdr -> module A J103", skip=(1,))
S.wire((33.2, yg), j3["1"])
j5 = S.ic("J5", 33.7, yb - 2.5, 2.4, 3.0, left=[(1, "GATE", "GATE_B"), (2, "PGND", "PGND"), (3, "SENSE", "SENSE_B")], title="3P hdr -> module B J203", skip=(1,))
S.wire((33.2, yb), j5["1"])
j2 = S.ic("J2", 38.5, yg + 0.8, 2.4, 1.8, left=[(1, "V12_A", "V12_A"), (2, "PGND", "PGND")], title="2P screw -> module A J102")
j4 = S.ic("J4", 38.5, yb - 3.6, 2.4, 1.8, left=[(1, "V12_B", "V12_B"), (2, "PGND", "PGND")], title="2P screw -> module B J202")
# ---- interlock
S.net((16.0, 6.25), "3V3", ha="right", size=7)
S.wire((16.1, 6.25), (17.0, 6.25)); S.R("R16", (17.0, 6.25), (19.5, 6.25), extra=""); S.wire((19.5, 6.25), (21.5, 6.25)); S.net((19.8, 6.55), "ILK_SRC", size=6.5)
j8 = S.ic("J8", 22.0, 4.6, 2.2, 2.2, left=[(1, "SRC", "ILK_SRC"), (2, "EN", "EN_SW")], title="2P hdr: interlock loop", skip=(1, 2),
          subtitle="closed = enabled; open/broken wire = disabled")
S.wire(j8["1"], (21.5, 6.25)) if abs(j8["1"][1] - 6.25) < 0.01 else None
y2 = j8["2"][1]
S.wire(j8["2"], (17.0, y2)); S.dot((19.6, y2)); S.net((19.0, y2 + 0.35), "EN_SW", size=7)
tp("TP11", (17.0, y2), (17.0, y2 + 0.55))
S.R("R17", (19.6, y2), (19.6, 2.2), side="right", extra=""); gl(19.6, 2.2)
S.C("C16", (18.0, y2), (18.0, 2.2), side="left"); S.dot((18.0, y2)); gl(18.0, 2.2)
S.net((28.2, 8.6), "EN_SW", ha="right", size=7)
S.wire((28.4, 8.6), (28.9, 8.6)); S.R("R18", (28.9, 8.6), (31.6, 8.6), extra="")
S.wire((31.6, 8.6), (33.0, 8.6)); S.net((33.1, 8.6), "STROBE_EN  (Pico GP4)", size=7)
# status LED
S.net((28.2, 5.4), "STATUS", ha="right", size=7)
S.wire((28.4, 5.4), (28.9, 5.4)); S.R("R21", (28.9, 5.4), (31.6, 5.4), extra="")
S.wire((31.6, 5.4), (32.4, 5.4)); S.dot((32.4, 5.4)); S.net((31.6, 5.9), "STATUS_LED_A", size=6)
S.D("D8", (32.4, 5.4), (32.4, 3.0), kind="led", side="right", extra="GP6 status"); gl(32.4, 3.0)
S.net((28.2 + 0.0, 5.0), "(Pico GP6)", ha="right", size=6, record=False)
# ARMED
XA = 41.5
S.net((XA, 9.8), "V12", ha="center", size=7)
S.wire((XA, 9.6), (XA, 8.4))
S.R("R20", (XA, 8.4), (XA, 5.05), side="right", extra="")
S.dot((XA, 5.05)); S.net((XA - 0.2, 5.05), "ARM_A", ha="right", size=6.5)
S.D("D7", (XA, 5.05), (XA, 3.2), kind="led", side="left", extra="ARMED"); S.dot((XA, 3.2)); S.net((XA - 0.2, 3.35), "ARM_K", ha="right", size=6.5)
q = elm.BjtNpn().at((XA, 3.2)).anchor("collector").theta(0)
S.d += q; S.reg("Q2")
bx, by = q.absanchors["base"].x, q.absanchors["base"].y
ex, ey = q.absanchors["emitter"].x, q.absanchors["emitter"].y
gl(ex, ey)
S.text((ex + 0.5, (3.2 + ey) / 2), "Q2\n2N3904", "left", "center", 7)
S.wire((bx, by), (bx - 1.0, by)); S.R("R19", (bx - 1.0, by), (bx - 3.4, by), extra="")
S.net((bx - 3.6, by), "EN_SW", ha="right", size=7); S.net((bx - 1.6, by + 0.4), "ARM_B", size=6.5)
j9 = S.ic("J9", 44.3, 2.275, 2.2, 3.7, left=[(1, "A", "ARM_A"), (2, "K", "ARM_K")], title="2P hdr: optional panel ARMED LED", skip=(1, 2))
S.wire(j9["1"], (XA, 5.05)); S.wire(j9["2"], (XA, 3.2))
S.note((16.0, 1.2), "ARMED LED current ~ (12 - 2 - 0.2)/1k = 9.8 mA.  EN_SW HIGH needs 3V3 -> J8 loop closed; broken wire -> R17 pulls LOW -> GATE_BUS stays LOW and the LED is dark.\n"
                    "The AND gate makes the interlock independent of firmware; firmware reads the same signal on GP4 (STROBE_EN) and reports GS_FAULT_INTERLOCK_OPEN.", size=6.5)
S.save()
