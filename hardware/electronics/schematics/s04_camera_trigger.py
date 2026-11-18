#!/usr/bin/env python3
"""Sheet 04: camera trigger level shifter / fan-out (1.8 V safe default, 3.3 V by jumper)."""
import schemdraw.elements as elm
from sd import Sheet

S = Sheet("04_camera_trigger", "04  Camera trigger interface (Pico GP2 -> two IMX296 module trigger inputs)", ["camtrig"])
S.title_block(0, 19.0, "SN74LVC2G17 Schmitt buffer powered from VTRIG (1.8 V LDO or 3.3 V by JP1).  Idle HIGH, active LOW.  VERIFY the trigger level and pin of YOUR camera module before connecting J6/J7.")


def gl(x, y, lab="GND_L"):
    S.d += elm.Ground().at((x, y)); S.net((x + 0.3, y - 0.45), lab, size=6.5)


def tp(ref, xy, lab_xy):
    S.reg(ref); S.d += elm.Dot(open=True).at(xy); S.text(lab_xy, ref, "center", "center", 6.5, "#cc0000")


YN = 10.0
S.net((-0.2, YN), "CAM_TRIG  (Pico GP2, U1.4)", ha="right", size=7)
S.R("R10", (0.3, YN), (3.3, YN), extra="series")
S.wire((3.3, YN), (7.0, YN)); S.dot((4.6, YN)); S.net((4.8, YN + 0.3), "CAMTRIG_IN", size=7)
S.R("R11", (4.6, YN), (4.6, 13.0), side="right", extra="idle-high"); S.net((4.6, 13.4), "3V3", ha="center", size=7)
tp("TP7", (4.6, 8.2), (5.4, 8.2)); S.wire((4.6, YN), (4.6, 8.2))
u = S.ic("U3", 7.5, 5.0, 4.0, 6.0, left=[(1, "1A", "CAMTRIG_IN"), (2, "GND", "GND_L"), (3, "2A", "CAMTRIG_IN")],
         right=[(6, "1Y", "CAM1_DRV"), (4, "2Y", "CAM2_DRV")], top=[(5, "VCC", "VTRIG")], skip=(1, 2, 5, 6, 4),
         title="SN74LVC2G17", subtitle="SOT-23-6 on adapter board - verify pin-out")
gl(*u["2"])
S.net((u["3"][0] - 0.1, u["3"][1] + 0.0), "CAMTRIG_IN", ha="right", size=6.5)
S.net((u["5"][0] + 0.15, u["5"][1] + 0.1), "VTRIG", size=7)
S.wire(u["5"], (u["5"][0], 12.6)); S.dot((u["5"][0], 12.6)); S.wire((u["5"][0], 12.6), (11.8, 12.6))
S.C("C14", (10.4, 12.6), (10.4, 14.4), side="right"); S.dot((10.4, 12.6)); S.net((10.4, 14.7), "GND_L", ha="center", size=6.5)
S.C("C15", (11.8, 12.6), (11.8, 14.4), side="right"); S.net((11.8, 14.7), "GND_L", ha="center", size=6.5)
# channels
for ch, (_nm, yy, rr, dd, jj) in enumerate((("1", 9.5, "R12", "D5", "J6"), ("2", 3.0, "R13", "D6", "J7"))):
    ydrv = u["6"][1] if ch == 0 else u["4"][1]
    if ch == 1:
        S.wire((12.0, ydrv), (12.0, yy)); S.wire((12.0, yy), (13.0, yy))
        start = (13.0, yy)
    else:
        S.wire((12.0, ydrv), (13.0, ydrv)); start = (13.0, ydrv)
    S.net((12.1, yy + 0.3 if ch == 0 else yy + 0.3), f"CAM{ch+1}_DRV", size=6.5)
    S.R(rr, start, (16.0, yy), extra="series / damping")
    S.wire((16.0, yy), (22.0 - 0.5, yy)); S.dot((17.4, yy)); S.net((18.2, yy + 0.3), f"CAM{ch+1}_OUT", size=7)
    d = S.ic(dd, 16.2, yy - 4.2, 2.4, 1.6, top=[(3, "J", f"CAM{ch+1}_OUT")], left=[(1, "A1", "GND_L")], right=[(2, "K2", "VTRIG")],
             title="BAT54S", skip=(3,), subtitle="clamp -0.3 V .. VTRIG+0.3 V")
    S.wire(d["3"], (17.4, yy))
    gl(*d["1"])
    j = S.ic(jj, 22.0, yy - 1.0, 2.4, 2.0, left=[(1, "TRIG", f"CAM{ch+1}_OUT"), (2, "GND", "GND_L")], title="2P header", skip=(1,),
             subtitle=f"to camera {ch+1} trigger pad + GND")
    S.wire((21.5, yy), j["1"])
    gl(*j["2"])
tp("TP8", (21.0, 9.5), (21.0, 10.1)); tp("TP9", (14.6, 12.6), (14.6, 13.2)); S.wire((11.8, 12.6), (14.6, 12.6))
S.net((14.9, 12.3), "VTRIG", size=6.5)
# LDO / jumper block
S.text((24.0, 17.6), "1.8 V reference and level select", "left", "center", 8, "#000000", "bold")
l = S.ic("U4", 25.0, 14.0, 2.6, 1.6, left=[(2, "VIN", "3V3")], right=[(3, "VOUT", "V1P8")], bottom=[(1, "GND", "GND_L")], title="MCP1700-1802", skip=(2, 3, 1),
         subtitle="1.8 V LDO, TO-92 - verify pin order")
gl(*l["1"])
S.wire(l["2"], (23.5, l["2"][1])); S.dot((23.5, l["2"][1])); S.net((22.9, l["2"][1] + 0.3), "3V3", ha="right", size=7)
S.C("C12", (23.5, l["2"][1]), (23.5, l["2"][1] - 1.9), side="left"); gl(23.5, l["2"][1] - 1.9)
S.wire(l["3"], (30.2, l["3"][1])); S.dot((29.2, l["3"][1])); S.net((28.7, l["3"][1] + 0.3), "V1P8", size=7)
S.C("C13", (29.2, l["3"][1]), (29.2, l["3"][1] - 1.9), side="left"); gl(29.2, l["3"][1] - 1.9)
y1 = l["3"][1]
j1 = S.ic("JP1", 31.0, y1 - 2.5, 2.4, 3.0, left=[(1, "1", "V1P8"), (2, "2", "VTRIG"), (3, "3", "3V3")], title="3P header", skip=(1,),
          subtitle="1-2 = 1.8 V (default, safe)  2-3 = 3.3 V  none = off")
S.wire((30.5, y1), j1["1"])
S.note((0, 1.4), "Edge estimate (sim/cam_trigger.py): R_out ~45 ohm + 47 ohm into ~30 pF -> 10-90 % ~6 ns at 1.8 V; skew between cameras ~1 ns.\n"
                 "Safety: with no jumper the buffer is unpowered and its 5 V-tolerant input does not load GP2; D5/D6 limit each camera line to <= VTRIG + 0.3 V (2.1 V in the 1.8 V position).\n"
                 "GP2 idles HIGH through R11; firmware 'trigger invert' = idle high, pulse low (exposure = low time + ~14 us, per SPEC).", size=6.5)
S.save()
