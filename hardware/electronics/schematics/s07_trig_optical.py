#!/usr/bin/env python3
"""Sheet 07: optical shot-trigger front end (modulated 940 nm reflective sensor)."""
from sd import Sheet

S = Sheet("07_trig_optical", "07  Shot trigger, option (b): OPTICAL  (100 kHz-modulated reflective sensor, ball leaves -> signal drops)", ["optical"])
S.title_block(0, 29.5, "Netlist-style drawing (NET_A --[part]-- NET_B).  Op-amps U6: V5A/GND_L.  Oscillator and emitter use the separately filtered V5E.  Comparator ch3 is U8 on sheet 06.")
RH = 1.55
# column 1: oscillator and emitter
X = 0.0
S.text((X, 28.0), "1  Modulator (TLC555 astable ~98.6 kHz) and emitter driver", "left", "center", 9, "#000000", "bold")
S.ic("U9", X + 3.0, 19.0, 4.6, 7.0,
     left=[(2, "TRIG", "OSC_C"), (6, "THR", "OSC_C"), (7, "DIS", "OSC_D"), (4, "RESET", "OSC_EN")],
     right=[(3, "OUT", "OSC_OUT"), (5, "CTRL", "OSC_CT")], top=[(8, "VCC", "V5E")], bottom=[(1, "GND", "GND_L")], title="TLC555")
y = 17.4
for ref, a, b, k in (("R55", "V5E", "OSC_D", "R"), ("R56", "OSC_D", "OSC_C", "R"), ("C39", "OSC_C", "GND_L", "C"), ("C40", "OSC_CT", "GND_L", "C"),
                     ("R57", "OSC_EN", "GND_L", "R"), ("C41", "V5E", "GND_L", "C"), ("R58", "OSC_OUT", "EMIT_B", "R")):
    S.branch(X, y, ref, a, b, k); y -= RH
S.ic("JP4", X + 3.0, y - 1.2, 2.4, 1.2, left=[(1, "1", "OSC_EN")], right=[(2, "2", "V5E")], title="2P hdr: install = emitter ON"); y -= 3.4
S.ic("Q3", X + 3.0, y - 2.4, 2.4, 2.0, left=[(2, "B", "EMIT_B")], right=[(3, "C", "EMIT_K")], bottom=[(1, "E", "GND_L")], title="2N3904"); y -= 4.8
S.branch(X, y, "R59", "V5E", "EMIT_A", "R", extra="~95 mA"); y -= RH
S.text((X, y - 0.2), "f = 1.44 / ((R55 + 2 R56) C39) = 98.6 kHz (sim script)", "left", "top", 7, "#7f6000")
S.tprow(X, y - 1.6, "TP24", "OSC_OUT")
# column 2: sensor head connector, TIA, band-pass
X = 16.0
S.text((X, 28.0), "2  Sensor head (remote) and transimpedance amplifier", "left", "center", 9, "#000000", "bold")
S.ic("J11", X + 3.0, 21.0, 3.2, 5.0, left=[(1, "LED_A", "EMIT_A"), (2, "LED_K", "EMIT_K"), (3, "PD_K", "PD_K"), (4, "PD_A", "TIA_IN"), (5, "SHLD", "GND_L")],
     title="5P hdr -> head")
S.branch(X + 8.0, 25.0, "D10", "EMIT_A", "EMIT_K", "D", extra="940 nm IR LED", plen=2.4)
S.branch(X + 8.0, 23.0, "D11", "TIA_IN", "PD_K", "D", extra="IR photodiode", plen=2.4)
S.text((X + 8.3, 26.2), "on the sensor head (wired through J11):", "left", "center", 7, "#7f6000")
y = 19.6
S.branch(X, y, "R47", "V5A", "PD_K", "R"); y -= RH
S.branch(X, y, "C32", "PD_K", "GND_L", "C"); y -= RH
S.branch(X, y, "R48", "TIA_IN", "TIA_OUT", "R"); y -= RH
S.branch(X, y, "C33", "TIA_IN", "TIA_OUT", "C"); y -= RH
S.amp(X + 5.0, y - 2.0, "U6", "A", "VREF", "TIA_IN", "TIA_OUT", (1, 2, 3)); y -= 4.4
S.text((X, y), "3  Band-pass (MFB, f0 ~100 kHz, Q ~3, gain 3)", "left", "center", 9, "#000000", "bold"); y -= 1.2
for ref, a, b, k in (("R49", "TIA_OUT", "MFB_A", "R"), ("R50", "MFB_A", "VREF", "R"), ("C34", "MFB_A", "MFB_INV", "C"), ("C35", "MFB_A", "OPT_BP", "C"), ("R51", "MFB_INV", "OPT_BP", "R"), ("C36", "V5A", "GND_L", "C")):
    S.branch(X, y, ref, a, b, k); y -= RH
S.amp(X + 5.0, y - 2.0, "U6", "B", "VREF", "MFB_INV", "OPT_BP", (7, 6, 5))
S.tprow(X, -1.5, "TP20", "TIA_OUT"); S.tprow(X + 8, -1.5, "TP21", "OPT_BP")
# column 3: envelope detector, threshold
X = 33.0
S.text((X, 28.0), "4  Envelope detector and threshold (ch3 of U8)", "left", "center", 9, "#000000", "bold")
y = 26.8
S.branch(X, y, "D9", "OPT_BP", "ENV", "S", extra="BAT85"); y -= RH
S.branch(X, y, "C37", "ENV", "VREF", "C"); y -= RH
S.branch(X, y, "R52", "ENV", "VREF", "R"); y -= RH
S.branch(X, y, "R46", "OPT_N", "ENV", "R", extra="hysteresis"); y -= RH + 0.3
S.branch(X, y, "R53", "V5A", "THO_TOP", "R"); y -= 2.6
S.ic("RV3", X + 3.0, y - 1.4, 2.8, 2.6, left=[(1, "1", "THO_TOP"), (2, "W", "THO")], right=[(3, "3", "THO_BOT")], title="10k trimmer: threshold"); y -= 4.4
S.branch(X, y, "R54", "THO_BOT", "GND_L", "R"); y -= RH
S.branch(X, y, "C38", "THO", "GND_L", "C"); y -= RH
S.tprow(X, y, "TP22", "ENV"); y -= RH
S.tprow(X, y, "TP23", "THO"); y -= RH + 0.5
S.text((X, y), "U8 ch3: IN3+ = ENV, IN3- = THO, OUT3 = OPT_N (low when ENV < THO,\ni.e. the reflected signal dropped = ball moved).  OPT_N -> JP2 pin 3 (sheet 08).", "left", "top", 7, "#7f6000")
S.text((X, y - 2.6), "Sim: ENV step 0.37 V -> ~0 V; mid-level crossing ~15-17 us after the\nphotocurrent drops (+ ~1.5 us comparator).  Signal estimate 3-10 uA pk\nat 15 mm (UNVERIFIED); minimum useful ~4 uA pk.", "left", "top", 7, "#7f6000")
S.save()
