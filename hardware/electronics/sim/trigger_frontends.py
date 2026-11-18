#!/usr/bin/env python3
"""Shot-trigger front ends: (a) acoustic band-pass + comparator, (b) optical modulated reflective sensor.
(a) analytic transfer functions (scipy) + synthetic impact click, detection delay vs amplitude/threshold
(b) ngspice: TIA + MFB band-pass + envelope detector with a finite-GBW op-amp macro (10 MHz, rails 0.05-4.95 V)
Values are read from circuit.py so drawings, netlist and simulation cannot drift apart.
"""
import json
import os
import re
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy import signal

OUT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(OUT))
import circuit as CK

RES = {}


def num(v):
    """parse '47k 1%' / '1k58' / '9k53' / '4u7' / '220p C0G' ..."""
    toks = [t for t in v.split() if re.match(r"^\d", t)]
    s = toks[0]
    m = re.match(r"^(\d+)([RkMmunpG])(\d*)$", s)
    if m:
        a, u, b = m.groups()
        x = float(a + ("." + b if b else ""))
        return x * {"R": 1, "k": 1e3, "M": 1e6, "m": 1e-3, "u": 1e-6, "n": 1e-9, "p": 1e-12, "G": 1e9}[u]
    m = re.match(r"^(\d+(?:\.\d+)?)([kMunp]?)$", s)
    a, u = m.groups()
    return float(a) * {"": 1, "k": 1e3, "M": 1e6, "u": 1e-6, "n": 1e-9, "p": 1e-12}[u]


V = lambda ref: num(CK.val(ref))
MIC_SENS = 6.3e-3        # V/Pa  (-44 dBV/Pa electret, verify)


# ------------------------------------------------------------------ acoustic
def acoustic_tf():
    R34, C25 = V("R34"), V("C25")
    Rf, Cf, Rg, Cg = V("R35"), V("C26"), V("R36"), V("C27")
    C28, R37, R38, C29 = V("C28"), V("R37"), V("R38"), V("C29")
    P = np.polynomial.polynomial
    # polynomials in s, ascending order
    hin = ([0, R34 * C25], [1, R34 * C25])
    # G1 = 1 + Rf*s*Cg / ((1+s Rf Cf)(1+s Rg Cg))
    d1 = P.polymul([1, Rf * Cf], [1, Rg * Cg])
    n1 = P.polyadd(d1, [0, Rf * Cg])
    # G2 = - (R38/(1+sR38C29)) * (sC28/(1+sR37C28))
    n2 = [0, -R38 * C28]
    d2 = P.polymul([1, R38 * C29], [1, R37 * C28])
    num_ = P.polymul(P.polymul(hin[0], n1), n2)
    den_ = P.polymul(P.polymul(hin[1], d1), d2)
    return num_[::-1], den_[::-1]       # descending for scipy


def acoustic():
    print("== Acoustic channel ==")
    n, d = acoustic_tf()
    w = 2 * np.pi * np.logspace(1, 5, 2000)
    _, h = signal.freqs(n, d, w)
    f = w / 2 / np.pi
    g = np.abs(h)
    gmax = g.max()
    band = f[g >= gmax / np.sqrt(2)]
    f_lo, f_hi = band.min(), band.max()
    print(f"  mid-band gain {gmax:.1f} V/V ({20*np.log10(gmax):.1f} dB) incl. 2 stages; -3 dB band {f_lo:.0f} Hz .. {f_hi/1e3:.1f} kHz; "
          f"{gmax*MIC_SENS:.2f} V/Pa at the comparator")
    RES["acoustic_gain"] = gmax; RES["acoustic_f_lo"] = f_lo; RES["acoustic_f_hi"] = f_hi
    thr = [CK.val("R41"), CK.val("R42"), CK.val("RV1")]
    vtop = 5.0 * (V("RV1") + V("R42")) / (V("R41") + V("RV1") + V("R42")); vbot = 5.0 * V("R42") / (V("R41") + V("RV1") + V("R42"))
    print(f"  threshold pot range {vbot:.2f} .. {vtop:.2f} V  (Vref 2.50 V) -> +{vbot-2.5:.2f} .. +{vtop-2.5:.2f} V at comparator = "
          f"{(vbot-2.5)/gmax*1e3:.2f} .. {(vtop-2.5)/gmax*1e3:.1f} mV at the mic = {(vbot-2.5)/gmax/MIC_SENS:.2f} .. {(vtop-2.5)/gmax/MIC_SENS:.1f} Pa peak "
          f"= {20*np.log10(((vbot-2.5)/gmax/MIC_SENS)/np.sqrt(2)/20e-6):.0f} .. {20*np.log10(((vtop-2.5)/gmax/MIC_SENS)/np.sqrt(2)/20e-6):.0f} dB SPL (sine-equivalent)")
    RES["acoustic_thr_V"] = (vbot - 2.5, vtop - 2.5)
    # hysteresis
    rs = 1.0 / (1 / (V("R41") + (1 - 0.5) * V("RV1")) + 1 / (V("R42") + 0.5 * V("RV1")))
    for rh in (V("R43") + 0.0, V("R43") + V("RV2")):
        print(f"  hysteresis RH={rh/1e3:.0f} k: ~{(3.3-2.55)*rs/(rh+rs)*1e3:.0f} mV (Rs~{rs/1e3:.1f} k, THR_P pulled down while ACOU_N low)")
    # time-domain click through the filter (state space)
    sys_ = signal.TransferFunction(n, d)
    t = np.arange(0, 6e-3, 1e-6)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    ax[0].semilogx(f, 20 * np.log10(g)); ax[0].set_ylim(-20, 45); ax[0].set_xlim(10, 1e5); ax[0].grid(True, which="both", alpha=0.3)
    ax[0].set_title("acoustic chain gain (mic node -> ACOU_SIG)"); ax[0].set_xlabel("Hz"); ax[0].set_ylabel("dB")
    delays = {}
    vth = 0.40
    for fc, tau_s, col in ((4e3, 0.25e-3, "tab:blue"), (7e3, 0.15e-3, "tab:orange")):
        for amp_pa, ls in ((0.5, ":"), (2.0, "--"), (8.0, "-")):
            p = amp_pa * np.exp(-t / tau_s) * np.sin(2 * np.pi * fc * t)
            _, y, _ = signal.lsim(sys_, p * MIC_SENS, t)
            y = np.clip(y, -2.45, 2.45)
            idx = np.argmax(np.abs(y) > vth)
            tdet = t[idx] if np.abs(y).max() > vth else np.nan
            delays[(fc, amp_pa)] = tdet
            if amp_pa == 8.0 or amp_pa == 2.0:
                ax[1].plot(t * 1e3, y, ls, color=col, lw=0.8, label=f"{fc/1e3:.0f} kHz click, {amp_pa} Pa pk")
    ax[1].axhline(vth, color="k", lw=0.6); ax[1].axhline(-vth, color="k", lw=0.6); ax[1].set_xlim(0, 3); ax[1].legend(fontsize=7)
    ax[1].set_title(f"ACOU_SIG (rel. to Vref), threshold +-{vth} V"); ax[1].set_xlabel("ms from arrival at the mic"); ax[1].grid(alpha=0.3)
    print("  detection delay after the sound reaches the capsule (comparator threshold +-0.40 V, + ~1.5 us comparator):")
    for (fc, a), tdl in delays.items():
        print(f"    click {fc/1e3:.0f} kHz, {a:4.1f} Pa peak -> {tdl*1e6:6.1f} us" if np.isfinite(tdl) else f"    click {fc/1e3:.0f} kHz, {a:4.1f} Pa peak -> no trigger")
    RES["acoustic_delays_us"] = {f"{fc}_{a}": (None if not np.isfinite(v) else v * 1e6) for (fc, a), v in delays.items()}
    # noise: ambient 65 dB SPL broadband over the pass-band
    p_amb = 20e-6 * 10 ** (65 / 20)
    v_amb = p_amb * MIC_SENS * gmax * np.sqrt(min(1.0, (f_hi - f_lo) / 20e3))
    print(f"  ambient 65 dB SPL -> about {v_amb*1e3:.0f} mV rms at the comparator (threshold must stay several x above: >= 0.2 V)")
    RES["ambient_65dB_mV_rms"] = v_amb * 1e3
    # distance table
    v_ball = 70.0
    ax[2].axis("off")
    rows = []
    for dm in (0.10, 0.20, 0.45):
        ta = dm / 343.0
        rows.append((dm, ta * 1e3, ta * v_ball * 1e3))
    txt = "acoustic delay (343 m/s) and ball travel at 70 m/s\n  mic dist [m]  delay [ms]  ball moved [mm]\n" + "\n".join(f"  {a:6.2f}      {b:6.2f}      {c:6.0f}" for a, b, c in rows)
    txt += "\n\n(2.9 ms per metre; +~0.05 ms filter/comparator)"
    ax[2].text(0, 0.9, txt, family="monospace", va="top")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "trigger_acoustic.png"), dpi=130); plt.close(fig)
    RES["acoustic_travel_table"] = rows
    for r_ in rows:
        print(f"  mic at {r_[0]:.2f} m: acoustic delay {r_[1]:.2f} ms = ball moved {r_[2]:.0f} mm at 70 m/s")


# ------------------------------------------------------------------ optical (ngspice)
NG_OPT = """
.title optical front end
.model DBAT D (IS=2e-7 N=1.06 RS=8 CJO=8p)
.subckt oa vp vm out
Bg n1 0 V = 1e5*(V(vp)-V(vm))
Rp n1 n2 1k
Cp n2 0 1.59u
Bo out 0 V = max(0.05, min(4.95, V(n2)))
.ends
Vref vref 0 2.5
.nodeset v(tin)=2.5 v(tout)=2.5 v(mfa)=2.5 v(mfi)=2.5 v(obp)=2.5 v(env)=2.5
Cpd tin vref 16p
Rf tin tout {RF}
Cf tin tout {CF}
XA vref tin tout oa
R49 tout mfa {R49}
R50 mfa vref {R50}
C34 mfa mfi {C34}
C35 mfa obp {C35}
R51 mfi obp {R51}
XB vref mfi obp oa
D9 obp env DBAT
C37 env vref {C37}
R52 env vref {R52}
{src}
{analysis}
.control
run
{wr}
.endc
.end
"""


def run_opt(src, analysis, wr, name):
    p = {k: V(k) for k in ("R49", "R50", "R51", "C34", "C35", "C37", "R52")}
    net = NG_OPT.format(RF=V("R48"), CF=V("C33"), src=src, analysis=analysis, wr=wr, **p)
    cir = os.path.join(OUT, f"_opt_{name}.cir")
    open(cir, "w").write(net)
    r = subprocess.run(["ngspice", "-b", cir], capture_output=True, text=True, timeout=600)
    outf = os.path.join(OUT, f"_opt_{name}.txt")
    if not os.path.exists(outf):
        print(r.stdout[-1500:], r.stderr[-800:])
        raise SystemExit("ngspice optical failed")
    return np.loadtxt(outf)


def optical():
    print("== Optical channel (ngspice, finite-GBW op-amp macro) ==")
    d = run_opt("Iac 0 tin dc 0 ac 1", ".ac dec 60 10 5meg", f"wrdata {OUT}/_opt_ac.txt vdb(tout) vdb(obp)", "ac")
    f, a_tia, a_bp = d[:, 0], d[:, 1], d[:, 3]
    # V/A -> dBohm
    i0 = np.argmax(a_bp)
    f0 = f[i0]; pk = a_bp[i0]
    band = f[a_bp >= pk - 3]
    print(f"  TIA transimpedance {10**(a_tia[np.argmin(abs(f-1e4))]/20)/1e3:.1f} kohm at 10 kHz; band-pass peak at {f0/1e3:.1f} kHz, "
          f"{10**(pk/20)/1e3:.0f} kohm total; -3 dB {band.min()/1e3:.0f}..{band.max()/1e3:.0f} kHz (Q~{f0/(band.max()-band.min()):.1f})")
    for fq in (100, 120, 1e3, 1e4, 5e4, 2e5, 1e6):
        print(f"    at {fq:8.0f} Hz: {a_bp[np.argmin(abs(f-fq))]-pk:7.1f} dB relative to peak")
    RES["opt_f0_kHz"] = f0 / 1e3; RES["opt_gain_ohm"] = 10 ** (pk / 20)
    RES["opt_rej_120Hz_dB"] = float(a_bp[np.argmin(abs(f - 120))] - pk)
    # transient: 98.6 kHz square-wave photocurrent, step from 10 uA to 3 uA (ball moves away) at 300 us
    fm = 1.44 / ((V("R55") + 2 * V("R56")) * V("C39"))
    print(f"  555 oscillator: f = 1.44/((R55+2 R56) C39) = {fm/1e3:.1f} kHz")
    RES["opt_fosc_kHz"] = fm / 1e3
    res = {}
    for a1, a2, tag in ((10e-6, 3e-6, "10to3uA"), (10e-6, 0.0, "10to0uA")):
        src = f"Bph 0 tin I = ({a1} - {a1-a2}*u(time-300u)) * u(sin(2*pi*{fm}*time))"
        d = run_opt(src, ".tran 50n 800u 0 50n", f"wrdata {OUT}/_opt_tr_{tag}.txt v(obp) v(env) v(tout)", "tr_" + tag)
        t, obp, env, tout = d[:, 0], d[:, 1], d[:, 3], d[:, 5]
        e_before = env[(t > 250e-6) & (t < 299e-6)].mean() - 2.5
        e_after = env[(t > 700e-6)].mean() - 2.5
        thr = 2.5 + 0.5 * (e_before + e_after)
        m = t > 300e-6
        idx = np.argmax(env[m] < thr)
        td = t[m][idx] - 300e-6
        print(f"  step photocurrent {a1*1e6:.0f} -> {a2*1e6:.0f} uA pk: ENV {e_before:.2f} V -> {e_after:.2f} V above Vref; "
              f"mid-level crossing {td*1e6:.0f} us after the step (+ ~1.5 us comparator)")
        res[tag] = (t, obp, env, tout, thr, td, e_before, e_after)
        RES[f"opt_delay_us_{tag}"] = td * 1e6
        RES[f"opt_env_V_{tag}"] = (e_before, e_after)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    ax[0].semilogx(f, a_tia, label="TIA out (dB ohm)"); ax[0].semilogx(f, a_bp, label="band-pass out (dB ohm)")
    ax[0].set_xlim(10, 5e6); ax[0].set_ylim(-20, 130); ax[0].grid(True, which="both", alpha=0.3); ax[0].legend(fontsize=8); ax[0].set_xlabel("Hz")
    ax[0].set_title("optical chain, photocurrent -> volts")
    t, obp, env, tout, thr, td, eb, ea = res["10to3uA"]
    ax[1].plot(t * 1e6, obp - 2.5, lw=0.5, label="band-pass out (rel. Vref)"); ax[1].plot(t * 1e6, env - 2.5, label="ENV (rel. Vref)")
    ax[1].axhline(thr - 2.5, color="k", ls=":", label="comparator threshold (mid)")
    ax[1].axvline(300, color="r", lw=0.6); ax[1].set_xlim(200, 500); ax[1].legend(fontsize=7); ax[1].set_xlabel("us"); ax[1].grid(alpha=0.3)
    ax[1].set_title(f"ball leaves: 10 -> 3 uA pk, crossing after {td*1e6:.0f} us")
    t, obp, env, tout, thr, td, eb, ea = res["10to0uA"]
    ax[2].plot(t * 1e6, env - 2.5, label="ENV"); ax[2].axhline(thr - 2.5, color="k", ls=":")
    ax[2].axvline(300, color="r", lw=0.6); ax[2].set_xlim(200, 500); ax[2].set_xlabel("us"); ax[2].grid(alpha=0.3)
    ax[2].set_title(f"10 -> 0 uA, crossing after {td*1e6:.0f} us")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "trigger_optical.png"), dpi=130); plt.close(fig)
    # photocurrent estimate (documented in README)
    ie, d_, rho, a_p, a_pd = 0.10, 0.015, 0.8, 21e-6, 7.5e-6        # W/sr at 100 mA (verify), m, reflectance, lit patch m2, PD area m2
    e = ie / d_ ** 2
    rad = rho * e / np.pi
    pw = rad * a_p * a_pd / d_ ** 2
    print(f"  photocurrent estimate: E on ball {e:.0f} W/m2, P on photodiode {pw*1e6:.0f} uW -> {pw*0.6*1e6:.0f} uA DC-equivalent (0.6 A/W); "
          f"modulated peak (square) ~ the same x ~0.5-1 -> design point 3-10 uA after geometry/curvature losses (UNVERIFIED)")
    RES["opt_photocurrent_est_uA"] = pw * 0.6 * 1e6


if __name__ == "__main__":
    acoustic()
    optical()
    json.dump(RES, open(os.path.join(OUT, "trigger_frontends.json"), "w"), indent=1, default=float)
