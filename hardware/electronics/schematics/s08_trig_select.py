#!/usr/bin/env python3
"""Sheet 08: selection jumper, polarity XOR and output to Pico GP5."""
from sd import Sheet
import schemdraw.elements as elm

S = Sheet("08_trig_select", "08  Shot trigger: source select (JP2), polarity (JP3) and output to Pico GP5", ["trigsel"])
S.title_block(0, 17.5, "Comparator outputs are open-collector, active LOW.  JP2 picks acoustic or optical.  U10 (XOR) + JP3 set the TRIG_IN polarity.  Default: idle LOW, rising edge = shot.")


def gl(x, y, lab="GND_L"):
    S.d += elm.Ground().at((x, y)); S.net((x + 0.3, y - 0.45), lab, size=6.5)


S.ic("JP2", 3.0, 8.0, 3.0, 3.0, left=[(1, "1 acou", "ACOU_N"), (3, "3 opt", "OPT_N")], right=[(2, "2 out", "TRIG_RAW")], title="3P hdr: source", skip=(),
     subtitle="1-2 acoustic   2-3 optical")
S.branch(10.0, 9.5, "R60", "3V3", "TRIG_RAW", "R", extra="idle pull-up")
S.branch(10.0, 7.5, "R45", "3V3", "OPT_N", "R", extra="pull-up (ch3)")
u = S.ic("U10", 22.0, 4.0, 4.0, 8.0,
         left=[(1, "1A", "TRIG_RAW"), (2, "1B", "POL"), (4, "2A", "GND_L"), (5, "2B", "GND_L"), (9, "3A", "GND_L"), (10, "3B", "GND_L"), (12, "4A", "GND_L"), (13, "4B", "GND_L")],
         right=[(3, "1Y", "TRIG_X"), (6, "2Y", "NC"), (8, "3Y", "NC"), (11, "4Y", "NC")], top=[(14, "VCC", "3V3")], bottom=[(7, "GND", "GND_L")], title="74HC86 XOR")
S.ic("JP3", 14.0, 1.0, 3.0, 3.0, left=[(1, "1", "3V3"), (3, "3", "GND_L")], right=[(2, "2", "POL")], title="3P hdr: polarity",
     subtitle="1-2: idle LOW, rising = shot (default)   2-3: idle HIGH, falling = shot")
S.branch(27.5, u["3"][1], "R61", "TRIG_X", "TRIG_IN", "R", extra="")
S.branch(27.5, u["3"][1] - 1.8, "R62", "TRIG_IN", "GND_L", "R", extra="")
S.net((38.5, u["3"][1] + 0.9), "-> Pico GP5 (U1.7)", size=7)
S.branch(0.0, 14.0 - 0.0, "C21", "3V3", "GND_L", "C", extra="U10"); 
S.branch(14.0, 14.0, "C22", "3V3", "GND_L", "C", extra="U2")
S.branch(28.0, 14.0, "C23", "3V3", "GND_L", "C", extra="local")
for i, (ref, net) in enumerate((("TP13", "VREF"), ("TP14", "TRIG_IN"), ("TP15", "ACOU_N"), ("TP16", "OPT_N"))):
    S.tprow(30.0, 2.8 - 0.9 * i, ref, net)
S.note((0, -2.4), "Polarity of GP5 is not defined in the firmware headers available to the author (GP5 'fast shot-trigger in'): verify against the firmware and set JP3.\n"
                  "Only one comparator output should be selected; the unselected channel idles with its pull-up.  Firmware should accept the first edge and ignore ringing for a hold-off time.", size=6.5)
S.save()
