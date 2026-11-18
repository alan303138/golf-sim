#!/usr/bin/env python3
"""Regenerate every schematic (SVG + PDF + PNG + JSON sidecar) from circuit.py values."""
import runpy, os, glob
here = os.path.dirname(os.path.abspath(__file__))
os.chdir(here)
import sys; sys.path.insert(0, here)
for f in sorted(glob.glob(os.path.join(here, "s0*.py"))):
    runpy.run_path(f, run_name="__main__")
