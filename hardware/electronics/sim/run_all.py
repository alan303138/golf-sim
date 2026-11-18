#!/usr/bin/env python3
"""Run every numeric verification script (needs numpy, scipy, matplotlib, ngspice on PATH)."""
import os, runpy, sys, glob
here = os.path.dirname(os.path.abspath(__file__))
os.chdir(here); sys.path.insert(0, here)
for name in ("radiometry", "strobe_power", "eye_safety", "cam_trigger", "trigger_frontends"):
    print("=" * 20, name)
    runpy.run_path(os.path.join(here, name + ".py"), run_name="__main__")
for f in glob.glob(os.path.join(here, "_*.cir")) + glob.glob(os.path.join(here, "_*.txt")):
    os.remove(f)
