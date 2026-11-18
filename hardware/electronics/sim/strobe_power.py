#!/usr/bin/env python3
"""Strobe power stage verification (one module = 6 strings x 3 LEDs, 12 V rail).

Part 1  analytic: string current vs rail, LED Vf spread, droop over a worst-case burst
Part 2  dissipation at the firmware limits (pulse <= 20 us, average duty <= 2 %)
Part 3  gate-drive time (bare Pico pin vs TC4427) and L/R current rise-time limit
Part 4  ngspice transient (behavioural MOSFET/LED models): pulse, burst, stuck-high cut-off
All device data are datasheet-class numbers marked 'verify' in the README; results are
design estimates, not measurements.
Run:  python3 strobe_power.py
"""
import json
import os
import subprocess
import tempfile

import numpy as np
from scipy.optimize import brentq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = os.path.dirname(os.path.abspath(__file__))
RES = {}

# ------------------------------------------------------------------ parameters
VIN = 12.0
N_STR, N_LED = 6, 3                 # strings per module, LEDs per string
RB = 1.8                            # ballast per string
R_CHG = 5.5                         # 2 x 11 ohm in parallel
C_BULK, ESR_BULK = 4400e-6, 0.015   # 2 x 2200 uF low ESR (verify ESR)
C_HF = 40e-6
RDS_ON, RSENSE = 0.009, 0.010
LED_IS, LED_N, LED_RS = 3.8e-13, 1.76, 0.25      # fits Vf(1 A)=1.55 V, Vf(3 A)=2.1 V (assumed, verify)
VT = 0.02585
K_FLUX = 0.85                       # flux ~ I^k


def vf(i, dv=0.0):
    return LED_N * VT * np.log(i / LED_IS + 1) + LED_RS * i + dv


def string_current(vrail, n_str=N_STR, dvf=0.0, rb=RB):
    """current per string for a given rail voltage at the ballast top (all strings equal)."""
    def f(i):
        return vrail - i * rb - N_LED * vf(i, dvf) - n_str * i * (RDS_ON + RSENSE)
    return brentq(f, 1e-6, 30)


# ------------------------------------------------------------------ part 1
def part1():
    print("== Part 1: string current ==")
    for dvf in (-0.2, 0.0, 0.2):
        print(f"  rail 12.0 V, per-LED Vf shift {dvf:+.1f} V -> {string_current(12.0, dvf=dvf):.2f} A per string (Vf(3A)={vf(3.0):.2f} V nominal)")
    v = np.linspace(9, 13, 9)
    RES["i_string_vs_rail"] = {f"{x:.1f}": string_current(x) for x in v}
    i_nom = string_current(12.0)
    RES["i_string_nominal_A"] = i_nom
    RES["i_module_nominal_A"] = i_nom * N_STR
    # burst droop: ideal rectangular pulses, bank + ESR, charged through R_CHG from 12 V
    def burst(width, n_pulse, period, n_str=N_STR, c=C_BULK, esr=ESR_BULK, rchg=R_CHG, dt=1e-6):
        """integer-sample time stepping; ideal rectangular pulses; bank = C + ESR charged via R_CHG."""
        n_per, n_on = int(round(period / dt)), int(round(width / dt))
        n_tot = n_pulse * n_per
        t = np.arange(n_tot) * dt
        vb = VIN
        itrace = np.zeros(n_tot); vtrace = np.zeros(n_tot)
        for n in range(n_tot):
            if (n % n_per) < n_on:
                i = string_current(vb, n_str)
                vterm = vb - i * n_str * esr
                i = string_current(vterm, n_str)
                itot = i * n_str
            else:
                i, itot, vterm = 0.0, 0.0, vb
            vb += dt * ((VIN - vb) / (rchg * c) - itot / c)
            itrace[n], vtrace[n] = i, vterm
        peaks = np.array([itrace[k * n_per + n_on - 1] for k in range(n_pulse)])
        vmin = vtrace[itrace > 0].min()
        return t, itrace, vtrace, peaks, vmin
    cases = []
    fig, ax = plt.subplots(2, 3, figsize=(13, 6))
    for col, (w, per) in enumerate(((5e-6, 250e-6), (10e-6, 500e-6), (20e-6, 1000e-6))):
        t, i, v, peaks, vmin = burst(w, 10, per)
        rel = peaks / peaks[0]
        cases.append(dict(width_us=w * 1e6, period_us=per * 1e6, duty=w / per, first_A=float(peaks[0]), last_A=float(peaks[-1]),
                          vmin=float(vmin), flux_last_rel=float(rel[-1] ** K_FLUX)))
        print(f"  burst 10 x {w*1e6:.0f} us every {per*1e6:.0f} us (duty {100*w/per:.1f} %), 6 strings: per-string I {peaks[0]:.2f} -> {peaks[-1]:.2f} A, "
              f"rail min {vmin:.2f} V, last-pulse flux {100*rel[-1]**K_FLUX:.1f} % of first")
        ax[0, col].plot(t * 1e3, v, lw=0.8); ax[0, col].set_title(f"10 x {w*1e6:.0f} us @ {100*w/per:.0f} % duty")
        ax[0, col].set_ylabel("bank terminal V [V]" if col == 0 else ""); ax[0, col].grid(alpha=0.3)
        ax[1, col].plot(np.arange(10) + 1, peaks, "o-"); ax[1, col].set_xlabel("pulse #"); ax[1, col].grid(alpha=0.3)
        ax[1, col].set_ylabel("per-string current [A]" if col == 0 else ""); ax[1, col].set_ylim(2.0, 3.3)
    ax[0, 0].set_xlabel("t [ms]"); fig.suptitle("Worst-case burst droop: 6 strings x 3 LEDs, 4.4 mF bank, 5.5 ohm charge resistor, ballast 1.8 ohm")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "strobe_burst_droop.png"), dpi=130); plt.close(fig)
    RES["burst"] = cases
    # sustained duty capability: avg current at duty d with R_CHG
    print("  sustained duty at the firmware limit (steady state, bank avg voltage):")
    for d in (0.005, 0.01, 0.02):
        # fixed-point: vavg = VIN - R_CHG * d * itot(vavg)
        vavg = VIN
        for _ in range(60):
            vavg = VIN - R_CHG * d * string_current(vavg) * N_STR
        print(f"    duty {100*d:.1f} % -> average rail {vavg:.2f} V, string current {string_current(vavg):.2f} A")
        RES[f"sustained_{d}"] = (vavg, string_current(vavg))


# ------------------------------------------------------------------ part 2
def part2():
    print("== Part 2: dissipation at firmware limits (6 strings, 12 V, pulse 20 us, duty 2 %) ==")
    i = string_current(12.0)
    d, w = 0.02, 20e-6
    itot = i * N_STR
    rows = {}
    rows["per-string pulse current A"] = i
    rows["LED forward voltage at pulse V"] = vf(i)
    rows["LED peak power W (each)"] = i * vf(i)
    rows["LED average power W (each, duty 2 %)"] = i * vf(i) * d
    rows["LED average power W (18 LEDs)"] = i * vf(i) * d * N_STR * N_LED
    rows["LED pulse energy mJ (each, 20 us)"] = i * vf(i) * w * 1e3
    rows["ballast peak power W"] = i ** 2 * RB
    rows["ballast pulse energy mJ (20 us)"] = i ** 2 * RB * w * 1e3
    rows["ballast average power W (each)"] = i ** 2 * RB * d
    rows["MOSFET peak power W (Rds 9 mohm)"] = itot ** 2 * RDS_ON
    rows["MOSFET average power W"] = itot ** 2 * RDS_ON * d
    rows["MOSFET peak Vds V"] = itot * RDS_ON
    rows["sense resistor peak power W"] = itot ** 2 * RSENSE
    rows["sense resistor average power W"] = itot ** 2 * RSENSE * d
    rows["sense voltage V (scope)"] = itot * RSENSE
    p_in = VIN * itot * d
    rows["input power W (module, ~ VIN*Iavg)"] = p_in
    rows["average input current A (module)"] = itot * d
    rows["charge resistor average power W (total R_CHG)"] = (itot * d) ** 2 * R_CHG
    rows["optical power W per LED at pulse (phi1*I^k)"] = 1.0 * i ** K_FLUX
    rows["wall-plug efficiency at pulse"] = 1.0 * i ** K_FLUX / (i * vf(i))
    rows["system efficiency (LED optical / input)"] = 18 * i ** K_FLUX / (VIN * itot)
    rows["LED junction pulse rise K (Zth(20us)~2 K/W assumed)"] = i * vf(i) * 2.0
    rows["LED plate average temperature rise K (2.25 W, ~6 K/W natural convection, assumed)"] = 18 * rows["LED average power W (each, duty 2 %)"] * 6.0
    for k, v in rows.items():
        print(f"  {k:70s} {v:9.3f}")
    RES["dissipation"] = rows
    # fault: MOSFET shorted drain-source (steady state) -> current limited by R_CHG
    def fault_i(n_str):
        def f(x):   # x = total current
            ips = x / n_str
            return VIN - x * R_CHG - ips * RB - N_LED * vf(ips)
        x = brentq(f, 1e-4, 20)
        return x
    for n in (4, 6):
        x = fault_i(n)
        print(f"  FAULT MOSFET shorted, {n} strings: total {x:.2f} A, per LED {x/n:.2f} A, R_CHG dissipation {x**2*R_CHG:.1f} W, LED each {x/n*vf(x/n):.2f} W")
        RES[f"fault_short_{n}_strings"] = dict(total_A=x, per_led_A=x / n, rchg_W=x ** 2 * R_CHG)


# ------------------------------------------------------------------ part 3
def part3():
    print("== Part 3: gate drive and L/R rise ==")
    qg10, qg33 = 17e-9, 6.5e-9      # total gate charge at Vgs 10 V / 3.3 V (IRLB8721 class, verify)
    # TC4427 route (AC coupled): 12 V step through Rout+Rg+Rgint into C_series
    rout, rg, rgi = 6.0, 2.2, 1.5
    cac, ceff = 10e-9, qg10 / 10.0
    cser = cac * ceff / (cac + ceff)
    tau = (rout + rg + rgi) * cser
    print(f"  TC4427 + AC coupling: C_series {cser*1e9:.2f} nF, R {rout+rg+rgi:.1f} ohm -> tau {tau*1e9:.1f} ns, 10-90 % {2.2*tau*1e9:.0f} ns, peak gate current {12/(rout+rg+rgi):.2f} A (limit 1.5 A x2 channels)")
    RES["gate_tc4427_10_90_ns"] = 2.2 * tau * 1e9
    # bare Pico pin: 3.3 V, ~ 80 ohm effective at 12 mA setting (assumed), plateau 2.5 V
    rpico = 80.0
    ig_avg = (3.3 - 2.0) / rpico
    t_pico = 6.5e-9 / ig_avg
    print(f"  bare Pico GPIO (3.3 V, ~{rpico:.0f} ohm): average gate current ~{ig_avg*1e3:.0f} mA -> gate transition >= {t_pico*1e9:.0f} ns, only 3.3 V Vgs (Rds ~2.5x higher), no turn-off pull-down strength")
    RES["gate_bare_pico_ns"] = t_pico * 1e9
    # L/R limit
    print("  current rise (10-90 %) vs string wiring inductance, 12 V rail, 3 A, ballast 1.8 ohm, 6 strings share the common loop")
    from scipy.integrate import solve_ivp
    rows = []
    for lstr in (30e-9, 60e-9, 120e-9, 250e-9):
        for lcom in (10e-9, 25e-9):
            leq = lstr + N_STR * lcom
            def rhs(t, y):
                i = y[0]
                v = 12.0 - i * (RB + RDS_ON + RSENSE * N_STR) - N_LED * vf(max(i, 1e-9))
                return [v / leq]
            sol = solve_ivp(rhs, [0, 3e-6], [0], max_step=1e-9, dense_output=True)
            tt = np.linspace(0, 3e-6, 30001); ii = sol.sol(tt)[0]
            i_end = ii[-1]
            t10 = tt[np.argmax(ii >= 0.1 * i_end)]; t90 = tt[np.argmax(ii >= 0.9 * i_end)]
            rows.append((lstr * 1e9, lcom * 1e9, (t90 - t10) * 1e9, i_end))
            print(f"    L_string {lstr*1e9:4.0f} nH, L_common {lcom*1e9:3.0f} nH -> L_eq {leq*1e9:4.0f} nH, 10-90 % rise {(t90-t10)*1e9:5.0f} ns")
    RES["rise_table"] = rows
    print("  inductive limit: t_rise ~ L_eq * I / V_headroom; headroom = 12 V - 3*Vf(0) ~ 8 V -> di/dt = V/L")


# ------------------------------------------------------------------ part 4 (ngspice)
NG = r"""
.title strobe module (6 strings x 3 LEDs)
.model DLED D (IS={IS} N={N} RS={RS} CJO=200p)
.model DSCH D (IS=3e-6 N=1.05 RS=0.015 CJO=300p)
.model NM NMOS (LEVEL=1 VTO=1.8 KP=25 LAMBDA=0.02)
Vin vin 0 DC 12
Rchg vin vb {RCHG}
Lbulk vb vb1 8n
Rbulk vb1 vb2 {ESR}
Cbulk vb2 0 {CB}
Lhf vb vh1 2n
Rhf vh1 vh2 4m
Chf vh2 0 {CH}
{strings}
Dfb drn vb DSCH
Lcom drn drn2 {LCOM}n
M1 drn2 gi s1 s1 NM W=1 L=1
Rgi g gi 1.5
Cgs g s1 1.0n
Cgd g drn2 0.25n
Lsrc s1 s2 2n
Rs s2 0 10m
Vdrv drvin s1 {pulse}
Rout drvin gout 6
Rg gout gaci 2.2
Cac gaci g 10n
Rb g s1 3.3k
Dcl s1 g DSCH
.options reltol=2e-3 abstol=1e-9 vntol=1e-5 itl4=100
.tran 20n {tstop} 0 {tmax}
.control
run
wrdata {out} v(s2) v(vb) v(g) v(drn) v(vb2)
.endc
.end
"""


def run_ng(pulse, tstop, tmax="20n", lstr=80, lcom=10, nstr=6, name="x"):
    strs = []
    for k in range(1, nstr + 1):
        strs.append(f"Rb{k} vb a{k} {RB}\nLs{k} a{k} b{k} {lstr}n\nDa{k} b{k} c{k} DLED\nDb{k} c{k} d{k} DLED\nDc{k} d{k} drn DLED")
    out = os.path.join(OUT, f"_ng_{name}.txt")
    net = NG.format(IS=LED_IS, N=LED_N, RS=LED_RS, RCHG=R_CHG, ESR=ESR_BULK, CB=C_BULK, CH=C_HF, strings="\n".join(strs),
                    LCOM=lcom, pulse=pulse, tstop=tstop, tmax=tmax, out=out)
    cir = os.path.join(OUT, f"_ng_{name}.cir")
    open(cir, "w").write(net)
    p = subprocess.run(["ngspice", "-b", cir], capture_output=True, text=True, timeout=600)
    if not os.path.exists(out):
        print(p.stdout[-2000:], p.stderr[-2000:])
        raise SystemExit("ngspice failed")
    d = np.loadtxt(out)
    t = d[:, 0]
    return dict(t=t, i=d[:, 1] / 0.010, vb=d[:, 3], vg=d[:, 5], vd=d[:, 7], vbulk=d[:, 9])


def edge_times(t, y, lo=0.1, hi=0.9, level=None, t_from=0.0, t_to=None):
    m = (t >= t_from) & (t <= (t_to if t_to else t[-1]))
    tt, yy = t[m], y[m]
    top = yy.max() if level is None else level
    i10 = np.argmax(yy >= lo * top); i90 = np.argmax(yy >= hi * top)
    return tt[i10], tt[i90], top


def part4():
    print("== Part 4: ngspice transient ==")
    ng = {}
    # single 10 us pulse
    r1 = run_ng("PULSE(0 12 2u 5n 5n 10u 100u)", "30u", "5n", name="one")
    t, i = r1["t"], r1["i"]
    t10, t90, top = edge_times(t, i, t_from=2e-6, t_to=8e-6, level=None)
    ipk = i[(t > 8e-6) & (t < 12e-6)].max()
    i_end = np.interp(11.9e-6, t, i)
    t10, t90, _ = edge_times(t, i, level=i_end, t_from=2e-6, t_to=8e-6)
    # fall: 90% -> 10% after pulse end
    mask = (t > 11.9e-6)
    tf = t[mask]; yf = i[mask]
    f90 = tf[np.argmax(yf <= 0.9 * i_end)]; f10 = tf[np.argmax(yf <= 0.1 * i_end)]
    g_end = np.interp(11.9e-6, t, r1["vg"])
    # gate rise
    vgm = r1["vg"]
    g10 = t[(t > 2e-6)][np.argmax(vgm[(t > 2e-6)] >= 0.1 * vgm.max())]; g90 = t[(t > 2e-6)][np.argmax(vgm[(t > 2e-6)] >= 0.9 * vgm.max())]
    print(f"  single 10 us pulse: module current {i_end:.1f} A at end ({i_end/6:.2f} A/string), rise 10-90 % {1e9*(t90-t10):.0f} ns, fall 90-10 % {1e9*(f10-f90):.0f} ns, "
          f"gate rise {1e9*(g90-g10):.0f} ns, gate at end of pulse {g_end:.1f} V, peak drain overshoot {r1['vd'].max():.1f} V")
    ng["one"] = dict(i_end=i_end, rise_ns=1e9 * (t90 - t10), fall_ns=1e9 * (f10 - f90), gate_rise_ns=1e9 * (g90 - g10), gate_end_V=g_end,
                     vdrain_max=float(r1["vd"].max()), i_peak=float(ipk))
    # energy delivered vs ideal rectangle: loss due to slow edges
    e = np.trapezoid(i * 1.0, t)  # A*s
    ng["charge_ratio_10us"] = e / (i_end * 10e-6)
    print(f"  charge in the pulse / ideal rectangle (10 us): {100*e/(i_end*10e-6):.1f} %")
    # 5 us pulse
    r5 = run_ng("PULSE(0 12 2u 5n 5n 5u 100u)", "20u", "5n", name="five")
    e5 = np.trapezoid(r5["i"], r5["t"]); ie5 = np.interp(6.9e-6, r5["t"], r5["i"])
    ng["charge_ratio_5us"] = e5 / (ie5 * 5e-6)
    print(f"  5 us pulse: current at end {ie5:.1f} A, charge / ideal rectangle {100*e5/(ie5*5e-6):.1f} %")
    # burst 10 x 10 us @ 500 us (2 % duty)
    rb = run_ng("PULSE(0 12 10u 5n 5n 10u 500u)", "5.2m", "100n", name="burst")
    t, i = rb["t"], rb["i"]
    peaks = []
    for k in range(10):
        a = 10e-6 + k * 500e-6
        m = (t > a + 7e-6) & (t < a + 10e-6)
        peaks.append(i[m].mean())
    print(f"  burst 10 x 10 us @ 500 us: module current pulse1 {peaks[0]:.1f} A -> pulse10 {peaks[-1]:.1f} A ({100*peaks[-1]/peaks[0]:.0f} %), bank min {rb['vbulk'].min():.2f} V")
    ng["burst10"] = dict(first=peaks[0], last=peaks[-1], vbank_min=float(rb["vbulk"].min()))
    # burst 10 x 20 us @ 1 ms (worst case)
    rw = run_ng("PULSE(0 12 10u 5n 5n 20u 1000u)", "10.3m", "100n", name="burstw")
    tw, iw = rw["t"], rw["i"]
    pw = []
    for k in range(10):
        a = 10e-6 + k * 1000e-6
        m = (tw > a + 15e-6) & (tw < a + 20e-6)
        pw.append(iw[m].mean())
    print(f"  worst burst 10 x 20 us @ 1 ms: {pw[0]:.1f} A -> {pw[-1]:.1f} A ({100*pw[-1]/pw[0]:.0f} %), bank min {rw['vbulk'].min():.2f} V")
    ng["burst10x20"] = dict(first=pw[0], last=pw[-1], vbank_min=float(rw["vbulk"].min()))
    # stuck-high gate: input high for 500 us
    rs = run_ng("PULSE(0 12 10u 5n 5n 500u 2000u)", "700u", "20n", name="stuck")
    ts, is_, vgs = rs["t"], rs["i"], rs["vg"]
    i_ref = np.interp(25e-6, ts, is_)
    m = ts > 25e-6
    t_90 = ts[m][np.argmax(is_[m] <= 0.9 * i_ref)] - 10e-6
    t_10 = ts[m][np.argmax(is_[m] <= 0.1 * i_ref)] - 10e-6
    t_1 = ts[m][np.argmax(is_[m] <= 0.01 * i_ref)] - 10e-6
    q_total = np.trapezoid(is_, ts)
    print(f"  STUCK-HIGH gate input: current {i_ref:.1f} A at 25 us, falls below 90 % after {t_90*1e6:.0f} us, 10 % after {t_10*1e6:.0f} us, 1 % after {t_1*1e6:.0f} us; "
          f"delivered charge {q_total*1e3:.2f} mC = {q_total/(i_ref*20e-6):.1f} x a normal 20 us pulse")
    ng["stuck"] = dict(t90_us=t_90 * 1e6, t10_us=t_10 * 1e6, t1_us=t_1 * 1e6, charge_vs_20us=q_total / (i_ref * 20e-6), i_ref=i_ref)
    # inductance sensitivity with ngspice
    ng["lstr"] = {}
    for lstr in (30, 80, 250):
        rr = run_ng("PULSE(0 12 2u 5n 5n 10u 100u)", "16u", "5n", lstr=lstr, name=f"l{lstr}")
        ie = np.interp(11.9e-6, rr["t"], rr["i"])
        a, b, _ = edge_times(rr["t"], rr["i"], level=ie, t_from=2e-6, t_to=8e-6)
        ng["lstr"][lstr] = (b - a) * 1e9
        print(f"  ngspice L_string {lstr:3d} nH: 10-90 % rise {1e9*(b-a):.0f} ns")
    RES["ngspice"] = ng

    # ---- plots
    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    a = ax[0, 0]
    a.plot(r1["t"] * 1e6, r1["i"], label="module current (V(sense)/10 mohm)")
    a.plot(r1["t"] * 1e6, r1["vg"], label="gate V"); a.plot(r1["t"] * 1e6, r1["vd"], label="drain V", lw=0.8)
    a.set_xlim(1, 16); a.set_title("single 10 us pulse"); a.set_xlabel("t [us]"); a.legend(fontsize=7); a.grid(alpha=0.3)
    a = ax[0, 1]
    a.plot(r5["t"] * 1e6, r5["i"]); a.set_xlim(1, 12); a.set_title("single 5 us pulse: current"); a.set_xlabel("t [us]"); a.set_ylabel("A"); a.grid(alpha=0.3)
    a = ax[1, 0]
    a.plot(rw["t"] * 1e3, rw["vbulk"], label="bank V, 10 x 20 us @ 1 ms")
    a.plot(rb["t"] * 1e3, rb["vbulk"], label="bank V, 10 x 10 us @ 0.5 ms")
    a.set_title("bank voltage during worst-case bursts"); a.set_xlabel("t [ms]"); a.legend(fontsize=8); a.grid(alpha=0.3)
    a = ax[1, 1]
    a.plot(rs["t"] * 1e6, rs["i"], label="module current"); a.plot(rs["t"] * 1e6, rs["vg"], label="gate V")
    a.set_xlim(0, 300); a.set_title("STUCK-HIGH gate signal: AC-coupling cut-off"); a.set_xlabel("t [us]"); a.legend(); a.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "strobe_ngspice.png"), dpi=130); plt.close(fig)


if __name__ == "__main__":
    part1(); part2(); part3(); part4()
    json.dump(RES, open(os.path.join(OUT, "strobe_power.json"), "w"), indent=1, default=float)
