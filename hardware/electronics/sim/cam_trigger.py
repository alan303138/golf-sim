#!/usr/bin/env python3
"""Camera-trigger interface: edge speed, skew, clamp levels (design estimate).
Buffer: SN74LVC2G17 (Schmitt, VCC = 1.8 V or 3.3 V by JP1), 47 ohm series resistor per camera,
BAT54S clamp to GND / VTRIG, ~30 cm of wire to each camera.  Output resistance and capacitances are
datasheet-class numbers (verify)."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os
OUT = os.path.dirname(os.path.abspath(__file__))

CASES = {"VTRIG = 1.8 V": dict(v=1.8, ro=45.0), "VTRIG = 3.3 V": dict(v=3.3, ro=22.0)}
RS = 47.0
C_LINE = {"30 cm wire": 15e-12, "100 cm wire": 50e-12}
C_PIN, C_D = 10e-12, 4e-12     # camera input pin (assumed) and BAT54S pair (~4 pF at 0 V, verify)
TPD = 3.5e-9                   # propagation delay of the buffer (typ at 1.8 V, verify)

fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
print("camera trigger edge estimate (10-90 % of the swing):")
for (name, p), a in zip(CASES.items(), ax):
    for (ln, cl), ls in zip(C_LINE.items(), ("-", "--")):
        c = cl + C_PIN + C_D
        tau = (p["ro"] + RS) * c
        tr = 2.2 * tau
        t = np.linspace(0, 40e-9, 400)
        v = p["v"] * (1 - np.exp(-np.clip(t - TPD, 0, None) / tau))
        a.plot(t * 1e9, v, ls, label=f"{ln}: {tr*1e9:.1f} ns")
        print(f"  {name}, {ln}: C={c*1e12:.0f} pF, tau={tau*1e9:.1f} ns, 10-90 % {tr*1e9:.1f} ns, + tpd {TPD*1e9:.1f} ns")
    a.set_title(name + "  (R_out + 47 ohm into line)"); a.set_xlabel("ns"); a.set_ylabel("V at camera pin"); a.grid(alpha=0.3); a.legend()
fig.tight_layout(); fig.savefig(os.path.join(OUT, "cam_trigger_edges.png"), dpi=130)
print("skew between the two camera outputs: same IC (< 1 ns) + RC mismatch of the two wires (assume +-10 % of tau ~ 0.5 ns) -> ~1 ns,"
      " far below the sensor trigger synchronisation granularity (about one line time, order 10 us - verify in sensor docs)")
print("clamp levels: line is limited to about -0.3 V ... VTRIG + 0.3 V by D5/D6 (BAT54S), i.e. <= 2.1 V in the 1.8 V position")
print("fault: if the camera pin were tied to 3.3 V while VTRIG=1.8 V, clamp current = (3.3-2.1)/(47) = %.0f mA (BAT54S: 200 mA cont.)" % (1.2 / 47 * 1e3))
print("Pico side: GP2 -> 100 ohm -> LVC2G17 input (5 V tolerant, Schmitt, threshold ~0.9 V at VCC=1.8 V); idle high via 10 k pull-up")
