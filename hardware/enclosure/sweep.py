"""Parameter sweeps run in sub-processes (the pose is computed at import time):
LED board centre 100..250 mm and baseline 120 / 220 mm (toe-in re-optimised)."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
LED_OFFSETS = (100, 140, 180, 250)
BASELINES = (120, 220)


def _run(overrides):
    env = dict(os.environ)
    env["RIG_OVERRIDES"] = json.dumps(overrides)
    out = subprocess.run([sys.executable, os.path.join(HERE, "build.py"), "--variant-check"], env=env,
                         capture_output=True, text=True, cwd=HERE)
    for line in out.stdout.splitlines():
        if line.startswith("VARIANT_JSON "):
            return json.loads(line[len("VARIANT_JSON "):])
    return {"error": (out.stderr or out.stdout)[-400:]}


def _yaws(overrides):
    env = dict(os.environ)
    env["RIG_OVERRIDES"] = json.dumps(overrides)
    out = subprocess.run([sys.executable, os.path.join(HERE, "optimize_fov.py"), "--json"], env=env,
                         capture_output=True, text=True, cwd=HERE)
    return json.loads(out.stdout.strip().splitlines()[-1])


def _digest(r):
    if "error" in r:
        return dict(ok=False, error=r["error"])
    f = r["fov"]
    return dict(
        interference=r["interference"],
        fov_ok=f["ok"], fov_clipped=sum(c["rays_clipped"] for c in f["cams"].values()),
        coverage_both=f["both_cameras_downrange_mm_at_launch_deg"],
        led_ok=r["led"]["ok"], stability_margin_mm=min(v["margin_mm"] for v in r["stability"]["cases"].values()),
        fpc_longest_mm=r["fpc"]["best_assignment_longest_mm"],
        ok=(not r["interference"]) and f["ok"] and r["led"]["ok"] and r["stability"]["ok"] and r["fpc"]["ok"])


def run_all():
    res = {"led_offset": {}, "baseline": {}}
    for off in LED_OFFSETS:
        res["led_offset"][str(off)] = _digest(_run({"LED_X": off}))
    for bl in BASELINES:
        y = _yaws({"BASELINE": bl})
        ov = {"BASELINE": bl, "TOE_IN_R": y["TOE_IN_R"], "TOE_IN_L": y["TOE_IN_L"]}
        d = _digest(_run(ov))
        d["toe_in"] = [y["TOE_IN_R"], y["TOE_IN_L"]]
        res["baseline"][str(bl)] = d
    return res


def summarize(res):
    L = ["== sweeps (each variant: full assembly, interference, field of view, LED lines, stability, FPC) =="]
    for k, v in res["led_offset"].items():
        L.append("  LED board centre +-%3s mm: ok=%s overlaps=%s clipped_rays=%s LED=%s coverage=%s" % (
            k, v.get("ok"), v.get("interference"), v.get("fov_clipped"), v.get("led_ok"), v.get("coverage_both")))
    for k, v in res["baseline"].items():
        L.append("  baseline %3s mm (toe-in %s): ok=%s overlaps=%s clipped_rays=%s margin=%s FPC=%s coverage=%s" % (
            k, v.get("toe_in"), v.get("ok"), v.get("interference"), v.get("fov_clipped"),
            v.get("stability_margin_mm"), v.get("fpc_longest_mm"), v.get("coverage_both")))
    return "\n".join(L)


if __name__ == "__main__":
    r = run_all()
    print(summarize(r))
    with open(os.path.join(HERE, "export", "sweep_report.json"), "w") as f:
        json.dump(r, f, indent=1)
