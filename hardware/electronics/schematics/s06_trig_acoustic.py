#!/usr/bin/env python3
"""Sheet 06: acoustic shot-trigger front end (electret -> band-pass amplifier -> window comparator), shared Vref."""
from sd import Sheet

S = Sheet("06_trig_acoustic", "06  Shot trigger, option (a): ACOUSTIC  (electret -> gain/band-pass -> comparator)", ["acoustic", "trigsel"])
S.title_block(0, 29.5, "Netlist-style drawing: every row is  NET_A --[part]-- NET_B ; identical net names are connected.  All op-amps / comparator: V5A and GND_L.  Vref = V5A/2 = 2.5 V.")
RH = 1.55
# ---- column 1
X = 0.0
S.text((X, 28.0), "1  Microphone + bias", "left", "center", 9, "#000000", "bold")
y = 27.0
for ref, a, b, k in (("R33", "V5A", "MIC_B", "R"), ("C24", "MIC_B", "GND_L", "C"), ("R32", "MIC_B", "MIC_P", "R")):
    S.branch(X, y, ref, a, b, k); y -= RH
p = S.ic("M1", X + 3.0, y - 1.2, 2.4, 1.2, left=[(1, "OUT+", "MIC_P")], right=[(2, "-", "GND_L")], title="electret capsule"); y -= 3.2
p = S.ic("J10", X + 3.0, y - 1.2, 2.4, 1.2, left=[(1, "1", "MIC_P")], right=[(2, "2", "GND_L")], title="2P hdr: remote capsule (optional)"); y -= 3.2
S.branch(X, y, "C25", "MIC_P", "MIC_AC", "C"); y -= RH
S.branch(X, y, "R34", "MIC_AC", "VREF", "R", extra="(bias)"); y -= RH
S.tprow(X, y, "TP19", "MIC_P"); y -= RH
S.text((X + 6.5, y + RH), "Electret ~6 mV/Pa (-44 dBV/Pa, verify)", "left", "center", 7, "#7f6000")
# vref block
y = 11.0
S.text((X, y + 1.0), "2  Reference Vref (shared by acoustic and optical)", "left", "center", 9, "#000000", "bold")
for ref, a, b, k in (("R30", "V5A", "VREF_RAW", "R"), ("R31", "VREF_RAW", "GND_L", "R"), ("C17", "VREF_RAW", "GND_L", "C"), ("C18", "VREF", "GND_L", "C"),
                     ("C19", "V5A", "GND_L", "C"), ("C20", "V5A", "GND_L", "C")):
    S.branch(X, y - 0.5, ref, a, b, k); y -= RH
S.amp(X + 5.0, y - 3.0, "U7", "A", "VREF_RAW", "VREF", "VREF", (1, 2, 3))
S.text((X, y - 5.6), "U7A = unity buffer; U7 decoupling C19, U8 decoupling C20", "left", "top", 7, "#7f6000")
# ---- column 2
X = 16.0
S.text((X, 28.0), "3  Stage 1: non-inverting, HP 1.6 kHz, LP ~15 kHz, gain 48", "left", "center", 9, "#000000", "bold")
S.amp(X + 5.0, 25.6, "U5", "A", "MIC_AC", "FB1", "AMP1", (1, 2, 3))
y = 22.2
for ref, a, b, k in (("R35", "AMP1", "FB1", "R"), ("C26", "AMP1", "FB1", "C"), ("R36", "FB1", "FB1G", "R"), ("C27", "FB1G", "VREF", "C")):
    S.branch(X, y, ref, a, b, k); y -= RH
S.text((X, 28.0 - 14.5), "4  Stage 2: inverting x2, HP 1.6 kHz, LP ~17 kHz", "left", "center", 9, "#000000", "bold")
y = 12.6
for ref, a, b, k in (("C28", "AMP1", "S2IN", "C"), ("R37", "S2IN", "FB2", "R"), ("R38", "FB2", "ACOU_SIG", "R"), ("C29", "FB2", "ACOU_SIG", "C"), ("C30", "V5A", "GND_L", "C")):
    S.branch(X, y, ref, a, b, k); y -= RH
S.amp(X + 5.0, y - 2.4, "U5", "B", "VREF", "FB2", "ACOU_SIG", (7, 6, 5))
y -= 5.6
S.text((X, y), "5  Inverted copy (so negative half-waves also trigger)", "left", "center", 9, "#000000", "bold")
y -= 1.3
S.branch(X, y, "R39", "ACOU_SIG", "INV_FB", "R"); y -= RH
S.branch(X, y, "R40", "INV_FB", "ACOU_INV", "R"); y -= RH
S.amp(X + 5.0, y - 2.4, "U7", "B", "VREF", "INV_FB", "ACOU_INV", (7, 6, 5))
S.tprow(X, 16.0, "TP17", "ACOU_SIG")
S.text((X, 15.0), "Total gain ~80 V/V mid-band (sim), -3 dB 1.9 ... 13 kHz", "left", "top", 7, "#7f6000")
# ---- column 3
X = 33.0
S.text((X, 28.0), "6  Threshold, hysteresis, comparators", "left", "center", 9, "#000000", "bold")
y = 27.0
S.branch(X, y, "R41", "V5A", "THR_TOP", "R"); y -= 2.6
S.ic("RV1", X + 3.0, y - 1.4, 2.8, 2.6, left=[(1, "1", "THR_TOP"), (2, "W", "THR_P")], right=[(3, "3", "THR_BOT")], title="10k trimmer: threshold")
y -= 4.4
S.branch(X, y, "R42", "THR_BOT", "GND_L", "R"); y -= RH
S.branch(X, y, "C31", "THR_P", "GND_L", "C"); y -= RH
S.branch(X, y, "R43", "ACOU_N", "HYS_M", "R", extra="hysteresis"); y -= 2.6
S.ic("RV2", X + 3.0, y - 1.5, 2.8, 2.6, left=[(1, "1", "HYS_M")], right=[(2, "W", "THR_P"), (3, "3", "THR_P")], title="500k trimmer: hysteresis (rheostat)")
y -= 4.4
S.branch(X, y, "R44", "3V3", "ACOU_N", "R", extra="pull-up"); y -= 2.8
S.ic("U8", X + 1.5, y - 8.4, 5.0, 7.6,
     left=[(4, "IN1-", "ACOU_SIG"), (5, "IN1+", "THR_P"), (6, "IN2-", "ACOU_INV"), (7, "IN2+", "THR_P"), (8, "IN3-", "THO"), (9, "IN3+", "ENV"), (10, "IN4-", "VREF"), (11, "IN4+", "GND_L")],
     right=[(2, "OUT1", "ACOU_N"), (1, "OUT2", "ACOU_N"), (14, "OUT3", "OPT_N"), (13, "OUT4", "NC")],
     top=[(3, "VCC", "V5A")], bottom=[(12, "GND", "GND_L")], title="LM339  (ch1/ch2 acoustic, ch3 optical, ch4 parked)")
S.tprow(X + 9.0, 26.0, "TP18", "THR_P")
S.text((X, -1.5), "Output low = event. ACOU_N goes to JP2 pin 1 (sheet 08).  ch3: see sheet 07.", "left", "top", 7, "#7f6000")
S.save()
