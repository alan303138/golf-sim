#!/usr/bin/env python3
"""Consistency / light-ERC checker for the phase-1 electronics.

Checks (all read the *generated* files, not circuit.py, so a stale file is caught):
  1  unique refs (duplicate YAML keys are an error)
  2  every pin has a net or is explicitly NC; pin counts match the package
  3  no single-pin nets; no net that only has input pins
  4  power / ground nets present and sourced
  5  Pico pin map equals the fixed map from the firmware spec
  6  single-point ground: only JP5 may join PGND and GND_L
  7  strobe module B is an exact copy of module A (refs +100, nets _A -> _B)
  8  BOM.csv matches the netlist (every ref exactly once, qty, value, package)
  9  schematic sidecars (schematics/out/*.json): every ref of a sheet is drawn,
     every drawn ref exists, drawn values equal netlist values, drawn nets exist
 10  netlist.yaml is identical to what circuit.py generates now
"""
import csv
import glob
import json
import os
import re
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

errors, warnings = [], []


def err(msg):
    errors.append(msg)


def warn(msg):
    warnings.append(msg)


class UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    seen = set()
    for k, _ in node.value:
        key = loader.construct_object(k, deep=deep)
        if key in seen:
            raise yaml.constructor.ConstructorError(None, None, f"duplicate key {key!r}", k.start_mark)
        seen.add(key)
    return yaml.SafeLoader.construct_mapping(loader, node, deep)


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)

PKG_PINS = {"DIP-8": 8, "DIP-14": 14, "TO-92": 3, "TO-220": 3, "SOT-23-6 on adapter": 6, "SOT-23 on adapter": 3,
            "2x20 DIP module on 2x 1x20 sockets": 40}
POWER_SRC = {"V12": "pwr_out", "V5": "pwr_out", "3V3": "pwr_out", "VIN_RAW": "pwr_out"}
EXPECT_PICO = {4: "CAM_TRIG", 5: "STROBE_GATE", 6: "STROBE_EN", 7: "TRIG_IN", 9: "STATUS", 34: "RAIL_SENSE"}
# GP2 -> pin 4, GP3 -> 5, GP4 -> 6, GP5 -> 7, GP6 -> 9, GP28/ADC2 -> 34


def main():
    try:
        with open(os.path.join(ROOT, "netlist", "netlist.yaml")) as f:
            doc = yaml.load(f, Loader=UniqueLoader)
    except Exception as e:  # noqa
        print("FAIL: cannot load netlist.yaml:", e)
        return 1
    parts = doc["parts"]
    print(f"[1] {len(parts)} unique refs")

    # ---- 2 pins
    nets = {}
    for ref, p in parts.items():
        npin = PKG_PINS.get(p["package"])
        if npin and len(p["pins"]) != npin:
            err(f"{ref}: package {p['package']} needs {npin} pins, has {len(p['pins'])}")
        for pin, d in p["pins"].items():
            if not d["net"]:
                err(f"{ref}.{pin}: no net and not NC")
            elif d["net"] != "NC":
                nets.setdefault(d["net"], []).append((ref, pin, d["type"]))
    n_nc = sum(1 for p in parts.values() for d in p["pins"].values() if d["net"] == "NC")
    print(f"[2] pins checked; {n_nc} explicit NC pins; {len(nets)} nets")

    # ---- 3 single-pin nets, input-only nets
    for net, pl in nets.items():
        if len(pl) < 2:
            err(f"net {net} has a single pin {pl}")
        types = {t for _, _, t in pl}
        if types and types <= {"in"}:
            err(f"net {net} has only input pins {pl}")
        outs = [x for x in pl if x[2] == "out"]
        # two outputs of the same IC (TC4427 channels in parallel) are allowed
        if len(outs) > 1 and len({o[0] for o in outs}) > 1:
            err(f"net {net} has {len(outs)} push-pull outputs: {outs}")
    print("[3] single-pin / input-only / multi-driver nets checked")

    # ---- 4 power nets
    for net, typ in POWER_SRC.items():
        if net not in nets:
            err(f"power net {net} missing")
        elif not any(t == typ for _, _, t in nets[net]):
            err(f"power net {net} has no {typ} pin")
    for net in ("GND_L", "PGND", "V12", "V5", "3V3", "V5A", "V5E", "VREF", "VTRIG", "V1P8", "V12_A", "V12_B"):
        if net not in nets:
            err(f"expected net {net} missing")
    print("[4] power/ground nets present and sourced")

    # ---- 5 Pico map
    u1 = parts["U1"]["pins"]
    for pin, net in EXPECT_PICO.items():
        if u1[str(pin)]["net"] != net:
            err(f"Pico pin {pin} ({u1[str(pin)]['name']}) is on {u1[str(pin)]['net']}, expected {net}")
    for pin, nm in {4: "GP2", 5: "GP3", 6: "GP4", 7: "GP5", 9: "GP6", 34: "GP28/ADC2"}.items():
        if u1[str(pin)]["name"] != nm:
            err(f"Pico pin {pin} is named {u1[str(pin)]['name']}, expected {nm}")
    print("[5] Pico pin map GP2/3/4/5/6/28 verified")

    # ---- 6 single-point ground
    for ref, p in parts.items():
        ns = {d["net"] for d in p["pins"].values()}
        if "PGND" in ns and "GND_L" in ns and ref != "JP5":
            err(f"{ref} joins PGND and GND_L (only JP5 may)")
    print("[6] PGND / GND_L joined only through JP5")

    # ---- 7 module symmetry
    def norm_a(ref, p):
        return {"ref": ref, "value": p["value"], "package": p["package"],
                "pins": {k: (d["net"], d["type"]) for k, d in p["pins"].items()}}
    a = {r: p for r, p in parts.items() if p["sheet"] == "strobe" and 100 < int(re.sub(r"\D", "", r)) < 200}
    b = {r: p for r, p in parts.items() if p["sheet"] == "strobe" and int(re.sub(r"\D", "", r)) > 200}
    if len(a) != len(b):
        err(f"module A has {len(a)} parts, module B {len(b)}")
    for ra, pa in a.items():
        m = re.match(r"([A-Za-z]+)(\d+)", ra)
        rb = f"{m.group(1)}{int(m.group(2)) + 100}"
        pb = b.get(rb)
        if pb is None:
            err(f"module B lacks {rb}")
            continue
        if pa["value"] != pb["value"] or pa["package"] != pb["package"]:
            err(f"{ra} vs {rb}: value/package differ")
        for k, d in pa["pins"].items():
            exp = d["net"].replace("_A", "_B").replace("A1", "B1") if d["net"] != "NC" else "NC"
            exp = re.sub(r"^(STR_|L)A(\d)", r"\1B\2", exp)
            got = pb["pins"][k]["net"]
            if exp != got:
                err(f"{rb}.{k} on {got}, expected {exp} (copy of {ra}.{k} on {d['net']})")
    print(f"[7] module B == module A (+100): {len(a)} parts compared")

    # ---- 8 BOM
    bom_refs = {}
    extras = 0
    with open(os.path.join(ROOT, "BOM.csv")) as f:
        for row in csv.DictReader(f):
            if row["ref"] == "TOTAL":
                continue
            if row["ref"] == "-":
                if "NOT IN NETLIST" not in row["notes"]:
                    err(f"BOM extra line without 'NOT IN NETLIST' marker: {row['value']}")
                extras += 1
                continue
            refs = row["ref"].split()
            if int(row["qty"]) != len(refs):
                err(f"BOM qty {row['qty']} != {len(refs)} refs for {row['ref'][:40]}")
            for ref in refs:
                if ref in bom_refs:
                    err(f"BOM lists {ref} twice")
                bom_refs[ref] = row
                if ref in parts:
                    if parts[ref]["value"] != row["value"] or parts[ref]["package"] != row["package"]:
                        err(f"BOM/netlist mismatch for {ref}: {row['value']}/{row['package']} vs "
                            f"{parts[ref]['value']}/{parts[ref]['package']}")
    for ref in parts:
        if ref not in bom_refs:
            err(f"{ref} in netlist but not in BOM")
    for ref in bom_refs:
        if ref not in parts:
            err(f"{ref} in BOM but not in netlist")
    print(f"[8] BOM covers all {len(parts)} refs; {extras} non-netlist hardware lines")

    # ---- 9 drawings
    sidecars = sorted(glob.glob(os.path.join(ROOT, "schematics", "out", "*.json")))
    if not sidecars:
        warn("no schematic sidecars found (run schematics/make_all.py)")
    drawn_by_sheet = {}
    all_nets = set(nets)
    for sc in sidecars:
        with open(sc) as f:
            j = json.load(f)
        for ref in j["refs"]:
            if ref not in parts:
                err(f"{os.path.basename(sc)}: draws unknown ref {ref}")
        for ref, v in j.get("values", {}).items():
            if ref in parts and parts[ref]["value"] != v:
                err(f"{os.path.basename(sc)}: {ref} drawn as '{v}' but netlist value is '{parts[ref]['value']}'")
        for net in j["nets"]:
            if net.startswith("->"):
                continue            # pure annotation
            net = re.split(r"[\s(]", net.strip())[0]   # 'GATE_A -> sheet 03' -> 'GATE_A'
            if net not in all_nets and not re.match(r"^(NC|GND|GNDL|\+?V?\d.*)$", net):
                err(f"{os.path.basename(sc)}: net label '{net}' not in netlist")
        for sh in j["covers"]:
            drawn_by_sheet.setdefault(sh, set()).update(j["refs"])
    for sh in ("power", "pico", "camtrig", "gating", "trigsel", "acoustic", "optical", "strobe"):
        want = {r for r, p in parts.items() if p["sheet"] == sh}
        if sh == "strobe":
            want = {r for r in want if int(re.sub(r"\D", "", r)) < 200}   # B is a copy of A
        have = drawn_by_sheet.get(sh, set())
        if sidecars:
            miss = sorted(want - have)
            if miss:
                err(f"sheet {sh}: refs not drawn: {miss}")
    print(f"[9] {len(sidecars)} schematic sidecars cross-checked")

    # ---- 10 yaml == circuit.py
    try:
        import circuit as C
        for p in C.all_parts():
            y = parts.get(p["ref"])
            if not y or y["value"] != p["value"]:
                err(f"netlist.yaml stale for {p['ref']} (rerun tools/generate.py)")
                continue
            for k, d in p["pins"].items():
                if y["pins"].get(k, {}).get("net") != d["net"]:
                    err(f"netlist.yaml stale for {p['ref']}.{k}")
        if len(C.all_parts()) != len(parts):
            err("part count differs from circuit.py (rerun tools/generate.py)")
        print("[10] netlist.yaml matches circuit.py")
    except ImportError:
        warn("circuit.py not importable; skip check 10")

    for w in warnings:
        print("WARNING:", w)
    if errors:
        print(f"\nFAILED with {len(errors)} error(s):")
        for e in errors:
            print("  -", e)
        return 1
    print("\nALL CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
