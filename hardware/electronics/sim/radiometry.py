#!/usr/bin/env python3
"""Radiometric estimate: how many photo-electrons does a white golf ball give per pixel
for one IR strobe pulse?  Every assumption is a named parameter with a plausible
range; a Monte-Carlo over the ranges gives the (honest) uncertainty.

Chain:   LED radiant flux -> radiant intensity (lens) -> irradiance on ball ->
         diffuse reflection (Lambert) -> lens/filter -> sensor irradiance ->
         photons per pixel -> electrons (QE) -> compare with full well.
Run:  python3 radiometry.py          (writes radiometry_*.png and radiometry.json)
"""
import json
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.dirname(os.path.abspath(__file__))
H, C_, Q = 6.626e-34, 2.998e8, 1.602e-19
LAM = 850e-9
E_PH = H * C_ / LAM                    # J per photon (2.34e-19)
PIX = 3.45e-6                          # IMX296 pixel pitch
A_PIX = PIX ** 2
FULL_WELL = 10000.0                    # e-, IMX296 class (verify); read noise ~2-3 e-

# name: (low, nominal, high, comment)
P = {
    "phi1":   (0.7, 1.0, 1.3, "radiant flux of ONE LED at 1 A (W); class value of 850 nm 1 mm2-chip power LEDs (verify datasheet)"),
    "k":      (0.75, 0.85, 0.95, "flux ~ I^k at pulse over-drive (efficiency droop); I=3 A -> x2.5"),
    "eta_lens": (0.75, 0.85, 0.92, "optical efficiency of TIR lens (1.0 = no lens)"),
    "aim":    (0.7, 0.85, 1.0, "loss from LEDs not aimed exactly at the ball (beam-edge/pointing)"),
    "dist":   (0.45, 0.49, 0.53, "LED to ball distance (m)"),
    "rho":    (0.60, 0.75, 0.90, "diffuse reflectance of white Surlyn ball at 850 nm"),
    "shape":  (0.5, 0.67, 0.8, "mean cos(incidence) over the visible ball disk (Lambert sphere; 2/3 when lit from the camera)"),
    "T":      (0.55, 0.75, 0.90, "lens x 850 nm band-pass transmission"),
    "qe":     (0.15, 0.25, 0.35, "sensor QE at 850 nm (IMX296 class; verify, strongly wavelength dependent)"),
}
LENS_FWHM = np.radians(60.0)           # TIR lens full angle at half max (gaussian beam); 40 deg option below


def intensity_peak(phi_e, lens, fwhm=None):
    fwhm = LENS_FWHM if fwhm is None else fwhm
    """On-axis radiant intensity W/sr of one LED."""
    if not lens:
        return phi_e / np.pi                              # Lambertian bare LED
    return phi_e / (np.pi / (4 * np.log(2)) * fwhm ** 2)   # gaussian beam of FWHM alpha


def electrons(n_led, i_pk, t_pulse, fnum, lens, p, fwhm=None):
    """photo-electrons per pixel on the ball for ONE pulse."""
    phi = p["phi1"] * i_pk ** p["k"]
    if lens:
        phi = phi * p["eta_lens"]
    i0 = intensity_peak(phi, lens, fwhm) * (p["aim"] if lens else 1.0)
    e_ball = n_led * i0 / p["dist"] ** 2                  # W/m2 on the ball
    rad = p["rho"] * e_ball * p["shape"] / np.pi          # W/m2/sr, mean disk radiance
    e_sensor = rad * p["T"] * np.pi / (4 * fnum ** 2)     # W/m2 on sensor (m ~ 0.016 negligible)
    return p["qe"] * e_sensor * A_PIX * t_pulse / E_PH


def nominal():
    return {k: v[1] for k, v in P.items()}


def mc(n_led, i_pk, t_pulse, fnum, lens, n=20000, seed=1):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        p = {k: rng.triangular(v[0], v[1], v[2]) for k, v in P.items()}
        out.append(electrons(n_led, i_pk, t_pulse, fnum, lens, p))
    out = np.array(out)
    return np.percentile(out, [10, 50, 90])


def main():
    nom = nominal()
    res = {"assumptions": {k: {"low": v[0], "nominal": v[1], "high": v[2], "meaning": v[3]} for k, v in P.items()},
           "constants": {"photon_energy_J": E_PH, "pixel_area_m2": A_PIX, "full_well_e": FULL_WELL}}
    rows = []
    print(f"photon energy {E_PH:.3e} J, pixel area {A_PIX:.3e} m2")
    print("\nnominal electrons per ball pixel per pulse (median [10%..90%] of Monte-Carlo)")
    print(f"{'LEDs':>4} {'I[A]':>4} {'lens':>5} {'f/#':>4} {'t[us]':>5} | {'nominal':>8} {'p10':>7} {'p50':>7} {'p90':>7}  %FW(p50)")
    for n_led in (24, 36):
        for lens in (False, True):
            for fn in (2.0, 2.8):
                for t in (5e-6, 10e-6, 20e-6):
                    nm = electrons(n_led, 3.0, t, fn, lens, nom)
                    p10, p50, p90 = mc(n_led, 3.0, t, fn, lens)
                    rows.append(dict(n_led=n_led, i=3.0, lens=lens, fnum=fn, t_us=t * 1e6, nominal=nm, p10=p10, p50=p50, p90=p90))
                    print(f"{n_led:4d} {3.0:4.1f} {'TIR' if lens else 'bare':>5} {fn:4.1f} {t*1e6:5.0f} | {nm:8.0f} {p10:7.0f} {p50:7.0f} {p90:7.0f}  {100*p50/FULL_WELL:5.1f}")
    res["table"] = rows
    print("\nnarrow 40 deg TIR option (central ball pixel, nominal) electrons per pulse, 3 A:")
    for n_led in (24, 36):
        for fn in (2.0, 2.8):
            print(f"  {n_led} LEDs f/{fn}:  5 us {electrons(n_led,3.0,5e-6,fn,True,nom,np.radians(40)):5.0f}  10 us {electrons(n_led,3.0,10e-6,fn,True,nom,np.radians(40)):5.0f}"
                  f"  20 us {electrons(n_led,3.0,20e-6,fn,True,nom,np.radians(40)):5.0f}")
            res[f"tir40_{n_led}_f{fn}_10us"] = electrons(n_led,3.0,10e-6,fn,True,nom,np.radians(40))
    print("\n5 A pulse (if the datasheet allows), 36 LEDs, TIR 60:",
          [round(electrons(36,5.0,10e-6,fn,True,nom)) for fn in (2.0,2.8)], "e- at 10 us, f/2.0 / f/2.8")
    # --- how many LEDs for a given target?
    print("\nLEDs needed (nominal) for target electrons, 3 A pulses, TIR lens:")
    need = {}
    for tgt in (300, 1000, 3000):
        for fn in (2.0, 2.8):
            for t in (5e-6, 10e-6):
                e1 = electrons(1, 3.0, t, fn, True, nom)
                need[f"{tgt}e_f{fn}_{int(t*1e6)}us"] = tgt / e1
        print(f"  {tgt:5d} e-: f/2.0 5us {tgt/electrons(1,3.0,5e-6,2.0,True,nom):6.0f}  10us {tgt/electrons(1,3.0,10e-6,2.0,True,nom):6.0f} | "
              f"f/2.8 5us {tgt/electrons(1,3.0,5e-6,2.8,True,nom):6.0f}  10us {tgt/electrons(1,3.0,10e-6,2.8,True,nom):6.0f}")
    res["leds_needed_nominal"] = need
    # equivalent source intensity needed (for the record)
    e_target = 800
    e1 = electrons(1, 3.0, 10e-6, 2.0, True, nom)
    i0 = intensity_peak(nom["phi1"] * 3.0 ** nom["k"] * nom["eta_lens"], True) * nom["aim"]
    print(f"\n800 e- @10 us f/2: need total on-axis intensity {e_target/e1*i0:.0f} W/sr (one TIR LED at 3 A gives {i0:.2f} W/sr)")
    res["intensity_for_800e_10us_f2_Wsr"] = e_target / e1 * i0
    res["i0_per_LED_3A_TIR_Wsr"] = i0
    res["i0_per_LED_3A_bare_Wsr"] = intensity_peak(nom["phi1"] * 3.0 ** nom["k"], False)

    # --- figure 1: electrons vs pulse width
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    ts = np.array([5, 10, 20])
    for a, lens in zip(ax, (False, True)):
        for n_led, col in ((24, "tab:blue"), (36, "tab:green")):
            for fn, ls in ((2.0, "-"), (2.8, "--")):
                lo, md, hi = [], [], []
                for t in ts:
                    p10, p50, p90 = mc(n_led, 3.0, t * 1e-6, fn, lens, n=4000)
                    lo.append(p10); md.append(p50); hi.append(p90)
                a.plot(ts, md, ls, color=col, marker="o", label=f"{n_led} LEDs f/{fn}")
                a.fill_between(ts, lo, hi, color=col, alpha=0.08 if fn == 2.8 else 0.15)
        a.axhline(300, color="r", lw=0.8, ls=":"); a.axhline(1000, color="g", lw=0.8, ls=":")
        a.text(5.2, 320, "300 e-  marginal", color="r", fontsize=8); a.text(5.2, 1050, "1000 e-  good", color="g", fontsize=8)
        a.set_yscale("log"); a.set_xscale("log"); a.set_xticks(ts); a.set_xticklabels(ts); a.minorticks_off()
        a.set_xlabel("pulse width [us]"); a.set_title(f"{'TIR lens 60 deg FWHM' if lens else 'bare LEDs (Lambertian)'}, 3 A pulses")
        a.grid(True, which="both", alpha=0.3)
    ax[0].set_ylabel("photo-electrons / ball pixel / pulse (median, 10-90 % band)")
    ax[0].legend(fontsize=8)
    fig.suptitle("Ball signal per pulse, IMX296-class sensor, full well ~10 000 e-")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "radiometry_electrons.png"), dpi=140); plt.close(fig)

    # --- figure 2: irradiance map over the 0.35 x 0.25 m field for two LED boards
    def e_map(lens, x_led, toe_deg, z=0.47, n=61, fwhm=None):
        fwhm = LENS_FWHM if fwhm is None else fwhm
        xs = np.linspace(-0.175, 0.175, n); ys = np.linspace(-0.125, 0.125, int(n * 0.714))
        X, Y = np.meshgrid(xs, ys)
        E = np.zeros_like(X)
        for xl in x_led:
            # board centre at (xl,0,0); a 4x3 array of LEDs spread +-0.03 m; aim: toe-in toward centre
            aim = np.array([-np.sign(xl) * np.sin(np.radians(toe_deg)), 0, np.cos(np.radians(toe_deg))])
            for dx in np.linspace(-0.03, 0.03, 4):
                for dy in np.linspace(-0.025, 0.025, 3):
                    src = np.array([xl + dx, dy, 0.0])
                    v = np.stack([X - src[0], Y - src[1], np.full_like(X, z)], -1)
                    r = np.linalg.norm(v, axis=-1)
                    cos_ax = (v @ aim) / r
                    th = np.arccos(np.clip(cos_ax, -1, 1))
                    if lens:
                        I = np.exp(-4 * np.log(2) * (th / fwhm) ** 2)
                    else:
                        I = np.clip(np.cos(th), 0, None)
                    E += I * (z / r) / r ** 2
        return xs, ys, E / E.max()
    fig, axs = plt.subplots(1, 4, figsize=(16, 3.6))
    for a, (lens, toe, fw, ttl) in zip(axs, ((False, 0, None, "bare LEDs, no toe-in"), (True, 0, None, "TIR 60 deg, no toe-in"),
                                         (True, 10, None, "TIR 60 deg, 10 deg toe-in"), (True, 16, np.radians(40), "TIR 40 deg, 16 deg toe-in"))):
        xs, ys, E = e_map(lens, (-0.14, 0.14), toe, fwhm=fw)
        im = a.imshow(E / E.max(), extent=[xs[0] * 1000, xs[-1] * 1000, ys[0] * 1000, ys[-1] * 1000], origin="lower", vmin=0.4, vmax=1.0, cmap="viridis")
        a.set_title(f"{ttl}\nmin/max = {E.min()/E.max():.2f}"); a.set_xlabel("x [mm]")
        res[f"uniformity_{ttl}"] = float(E.min() / E.max())
    axs[0].set_ylabel("y [mm]"); fig.colorbar(im, ax=axs, shrink=0.8, label="relative irradiance")
    fig.suptitle("Irradiance over the 350 x 250 mm field at 0.47 m (two 12-LED boards at x = +-140 mm)")
    fig.savefig(os.path.join(OUT, "radiometry_field.png"), dpi=140, bbox_inches="tight"); plt.close(fig)

    # --- raking angles (LED -> ball -> camera) for candidate placements
    print("\nangle between LED and camera as seen from the ball (deg); cameras at +-80 mm, ball at 0.47 m")
    ang = {}
    for xl in (0.0, 0.14, 0.25, 0.33):
        near = np.degrees(np.arctan((xl) / 0.47) - np.arctan(0.08 / 0.47))
        far = np.degrees(np.arctan(xl / 0.47) + np.arctan(0.08 / 0.47))
        ang[f"{xl}"] = (abs(near), far)
        print(f"  LED board at x=+-{xl*1000:4.0f} mm: nearest camera {abs(near):5.1f} deg, far-side camera {far:5.1f} deg")
    res["raking_angles_deg"] = ang
    # raking penalty: mean cos incidence when light is at angle a to the view axis
    rng = np.random.default_rng(0)
    n = rng.normal(size=(400000, 3)); n /= np.linalg.norm(n, axis=1)[:, None]
    n = n[n[:, 2] > 0]
    w = n[:, 2]
    for a_deg in (0, 26, 45, 60):
        a = np.radians(a_deg); l = np.array([np.sin(a), 0, np.cos(a)])
        m = np.sum(w * np.clip(n @ l, 0, None)) / np.sum(w)
        print(f"  mean ball brightness factor for light at {a_deg:2d} deg to the view axis: {m:.2f}  (vs 0.67 at 0 deg)")
        res[f"shape_{a_deg}"] = float(m)
    json.dump(res, open(os.path.join(OUT, "radiometry.json"), "w"), indent=1, default=float)


if __name__ == "__main__":
    main()
