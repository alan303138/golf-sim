#!/usr/bin/env python3
"""Conservative ESTIMATE of ocular IR exposure against IEC 62471 style limits.
NOT a certification and not a measurement.  Formulas (IEC 62471:2006, exempt group) as
remembered by the author - check them against the standard before relying on them:
  cornea / lens IR-A (780-3000 nm):  E_IR  <= 18000 t^-0.75  W/m2   (t <= 1000 s);  100 W/m2 for t > 1000 s
  retinal thermal (burn):            L_R   <= 50000 / (alpha t^0.25)  W/m2/sr   (10 us <= t <= 10 s), R(850 nm)=0.5
  retinal thermal, weak stimulus:    L_IR  <= 6000 / alpha            W/m2/sr   (t > 10 s)
Worst-case choices: all LEDs of BOTH boards assumed to point at the eye with the narrow 40 deg lens
(real boards are 280 mm apart, so close to the unit the real exposure is lower); point-source
formula used down to 5 cm (over-estimates for an extended array = conservative).
"""
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.dirname(os.path.abspath(__file__))
N_LED = 36
PHI_3A = 1.0 * 3.0 ** 0.85                  # W per LED at 3 A (see radiometry.py)
FWHM40 = np.radians(40)
I0_40 = 0.9 * PHI_3A / (np.pi / (4 * np.log(2)) * FWHM40 ** 2)     # W/sr on axis, narrow lens
I_TOT = N_LED * I0_40                        # W/sr, all aligned (upper bound)
A_CHIP = 1e-6                                # m2, 1 mm2 chip (assumed)
R850 = 10 ** ((700 - 850) / 500)             # 0.50
L_CHIP = PHI_3A / (np.pi * A_CHIP)           # W/m2/sr  (chip radiance, conserved through the lens)
res = {"I_total_Wsr_peak": I_TOT, "I0_per_led_40deg_Wsr": I0_40, "L_chip_Wm2sr": L_CHIP, "R850": R850}
print(f"per LED at 3 A: {PHI_3A:.2f} W, on-axis {I0_40:.2f} W/sr (40 deg lens); all 36 aligned: {I_TOT:.0f} W/sr peak")
print(f"chip radiance {L_CHIP:.2e} W/m2/sr, R-weighted {L_CHIP*R850:.2e}")

duties = {"max firmware duty 2 %": 0.02, "typical session 0.1 %": 0.001}
dist = np.array([0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 1.0])
print("\n-- cornea/lens IR-A, long exposure (limit 100 W/m2 for t > 1000 s), time-average irradiance")
print(f"{'d [m]':>6} {'E_peak W/m2':>12} " + " ".join(f"{k:>24}" for k in duties))
tab = []
for d in dist:
    ep = I_TOT / d ** 2
    row = [d, ep] + [ep * x for x in duties.values()]
    tab.append(row)
    print(f"{d:6.2f} {ep:12.0f} " + " ".join(f"{ep*x:24.1f}" for x in duties.values()))
res["cornea_table"] = tab
d_haz = {k: float(np.sqrt(I_TOT * x / 100.0)) for k, x in duties.items()}
print("distance where time-average irradiance = 100 W/m2:", {k: f"{v:.2f} m" for k, v in d_haz.items()})
res["hazard_distance_m"] = d_haz
# time to reach the limit at 5 cm and 10 cm for 2 % duty
for d in (0.05, 0.10):
    e = I_TOT * 0.02 / d ** 2
    if e > 100:
        t = (18000 / e) ** (1 / 0.75)
        print(f"at {d*100:.0f} cm and 2 % duty the exempt limit 18000 t^-0.75 is reached after {t:.0f} s of staring (E = {e:.0f} W/m2)")
        res[f"t_limit_{int(d*100)}cm_s"] = t
# single burst radiant exposure
for n, w in ((10, 20e-6), (10, 10e-6), (4, 100e-6)):
    t_burst = 5e-3
    for d in (0.05, 0.10, 0.30):
        h = I_TOT / d ** 2 * n * w
        lim = 18000 * t_burst ** 0.25
        print(f"single burst {n} x {w*1e6:.0f} us at {d*100:.0f} cm: H = {h:.2f} J/m2 vs limit {lim:.0f} J/m2 (t={t_burst*1e3:.0f} ms) -> ratio {h/lim:.1e}")
print("\n-- retinal thermal (source = LED chip image; alpha from 1.7 mrad to 100 mrad)")
lr = L_CHIP * R850
for alpha in (0.0017, 0.011, 0.1):
    lim_pulse = 50000 / (alpha * (20e-6) ** 0.25)
    lim_train = 50000 / (alpha * (5e-3) ** 0.25)
    l_train = lr * (200e-6 / 5e-3)
    print(f"  alpha {alpha*1e3:5.1f} mrad: single 20 us pulse L_R {lr:.2e} vs {lim_pulse:.2e} (ratio {lr/lim_pulse:.1e}); "
          f"10x20 us train averaged over 5 ms {l_train:.2e} vs {lim_train:.2e} (ratio {l_train/lim_train:.1e})")
    res[f"retinal_ratio_pulse_alpha{alpha}"] = lr / lim_pulse
print("  weak-stimulus (t>10 s) average radiance vs 6000/alpha:")
for alpha in (0.011, 0.1):
    for dname, x in duties.items():
        l_avg = lr * x
        print(f"    alpha {alpha*1e3:5.1f} mrad, {dname}: {l_avg:.2e} vs {6000/alpha:.2e}  ratio {l_avg/(6000/alpha):.2f}")
        res[f"weak_ratio_a{alpha}_{x}"] = l_avg / (6000 / alpha)
# what duty would break the weak-stimulus limit
for alpha in (0.011, 0.1):
    xmax = (6000 / alpha) / lr
    print(f"  weak-stimulus limit reached at duty {100*xmax:.1f} % (alpha {alpha*1e3:.0f} mrad)")
    res[f"duty_limit_weak_alpha{alpha}"] = xmax

fig, ax = plt.subplots(figsize=(7, 4.5))
dd = np.logspace(np.log10(0.03), 0, 100)
for (k, x), c in zip(duties.items(), ("tab:red", "tab:blue")):
    ax.loglog(dd, I_TOT / dd ** 2 * x, color=c, label=f"time-average E, {k}")
ax.axhline(100, color="k", ls="--", label="100 W/m2 (exempt, t>1000 s)")
ax.axhline(18000 * 10 ** -0.75, color="gray", ls=":", label="exempt limit for 10 s exposure (3200 W/m2)")
ax.set_xlabel("distance from the LED boards [m]"); ax.set_ylabel("corneal irradiance [W/m2]")
ax.set_title("IR-A corneal exposure estimate (all 36 LEDs aligned, 40 deg lens: worst case)")
ax.grid(True, which="both", alpha=0.3); ax.legend(fontsize=7)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "eye_safety.png"), dpi=140)
json.dump(res, open(os.path.join(OUT, "eye_safety.json"), "w"), indent=1, default=float)
