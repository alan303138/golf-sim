#!/usr/bin/env python3
"""Generate netlist/netlist.yaml, netlist/netlist.csv and BOM.csv from circuit.py."""
import csv, os, sys, collections
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
import circuit as C


def natural(ref):
    import re
    m = re.match(r"([A-Za-z]+)(\d+)", ref)
    return (m.group(1), int(m.group(2)))


def write_netlist():
    parts = {}
    for p in sorted(C.all_parts(), key=lambda p: (C.SHEETS.index(p["sheet"]), natural(p["ref"]))):
        parts[p["ref"]] = {
            "value": p["value"], "package": p["package"], "sheet": p["sheet"],
            "pins": {k: {"name": d["name"], "net": d["net"], "type": d["type"]}
                     for k, d in sorted(p["pins"].items(), key=lambda kv: (len(kv[0]), kv[0]))},
        }
    doc = {
        "meta": {"project": "golf-sim phase-1 electronics", "generated_by": "tools/generate.py",
                 "note": "cables are modelled as ideal wires: connectors on both ends share one net name"},
        "parts": parts,
    }
    with open(os.path.join(ROOT, "netlist", "netlist.yaml"), "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False, width=200)
    with open(os.path.join(ROOT, "netlist", "netlist.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ref", "value", "package", "pin", "pin_name", "net"])
        for ref, p in parts.items():
            for pin, d in p["pins"].items():
                w.writerow([ref, p["value"], p["package"], pin, d["name"], d["net"]])


def write_bom():
    groups = collections.OrderedDict()
    for p in sorted(C.all_parts(), key=lambda p: (p["key"], p["value"], natural(p["ref"]))):
        key = (p["value"], p["description"], p["package"], p["mpn"], p["notes"] if p["key"] in C.CAT else "", p["cost"])
        # group identical value/package/mpn irrespective of notes for passives
        gk = (p["value"], p["package"], p["mpn"], p["cost"])
        g = groups.setdefault(gk, {"refs": [], "desc": p["description"], "notes": set(), "cost": p["cost"],
                                   "value": p["value"], "package": p["package"], "mpn": p["mpn"]})
        g["refs"].append(p["ref"])
        if p["notes"]:
            g["notes"].add(p["notes"])
    rows = []
    for g in groups.values():
        refs = sorted(g["refs"], key=natural)
        notes = "; ".join(sorted(g["notes"]))[:240]
        rows.append([" ".join(refs), len(refs), g["value"], g["desc"], g["package"], g["mpn"], notes,
                     f"{g['cost']:.2f}", f"{g['cost']*len(refs):.2f}"])
    rows.sort(key=lambda r: natural(r[0].split()[0]))
    for (key, qty, value, desc, pkg, mpn, cost, notes) in C.EXTRAS:
        rows.append(["-", qty, value, desc, pkg, mpn, "NOT IN NETLIST (mechanical/hardware). " + notes,
                     f"{cost:.2f}", f"{cost*qty:.2f}"])
    total = sum(float(r[8]) for r in rows)
    # phase-1 population = 24 LEDs; the 36-LED option adds 12 LEDs + 12 lenses
    with open(os.path.join(ROOT, "BOM.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ref", "qty", "value", "description", "package", "part_number", "notes", "unit_cost_usd", "ext_cost_usd"])
        w.writerows(rows)
        w.writerow(["TOTAL", "", "", "approximate USD, hobby quantities, no shipping/tax; FULL build = 36 LEDs + 36 lenses (first 24-LED build is ~45 USD less); verify prices",
                    "", "", "", "", f"{total:.2f}"])
    return total


if __name__ == "__main__":
    write_netlist()
    t = write_bom()
    print(f"{len(C.PARTS)} parts; BOM total ~USD {t:.0f}")
