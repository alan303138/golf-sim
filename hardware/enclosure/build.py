#!/usr/bin/env python3
"""Regenerate everything: STL / STEP per printed part, assembly STEP, automated checks,
preview renders.   python3 build.py [--no-render] [--no-sweep] [--quick] [--variant-check]"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
EXPORT = os.path.join(HERE, "export")
STL = os.path.join(EXPORT, "stl")
STEP = os.path.join(EXPORT, "step")
DOCS = os.path.join(HERE, "docs")


def jdefault(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def export_parts(parts):
    import cadquery as cq
    import params as P
    from lib import best_print
    info = {}
    for name, p in parts.items():
        w, spin = best_print(p.wp, p.up)
        cq.exporters.export(w, os.path.join(STL, name + ".stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(w, os.path.join(STEP, name + ".step"))
        bb = w.val().BoundingBox()
        info[name] = dict(qty=p.qty, desc=p.desc, print_up_in_design_frame=list(p.up), spin_deg=spin,
                          size_mm=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                          volume_cm3=round(w.val().Volume() / 1000.0, 1),
                          est_mass_g=round(w.val().Volume() / 1000.0 * P.PETG_DENSITY * P.INFILL_EFF))
    return info


def export_assembly(bodies):
    import cadquery as cq
    asm = cq.Assembly(name="golf_rig_phase1")
    for b in bodies:
        r, g, bl = b.color
        alpha = 0.3 if b.transparent else 1.0
        nm = ("PROXY_" if b.kind == "proxy" else "") + b.name.replace("#", "_")
        asm.add(b.wp, name=nm, color=cq.Color(r, g, bl, alpha))
    asm.save(os.path.join(STEP, "assembly.step"))


def summarize(rep):
    import params as P
    lines = []

    def L(s=""):
        lines.append(s)
    L("== printed part files (build volume %s, watertight, on bed) ==" % (list(P.BUILD_VOLUME),))
    for n, r in rep["stl"]["parts"].items():
        L("  %-6s %-22s fits=%s watertight=%s winding=%s bodies=%d vol=%d mm3 -> %s" % (
            n, r["extents"], r["fits"], r["watertight"], r["winding"], r["bodies"], r["volume_mm3"], "OK" if r["ok"] else "FAIL"))
    i = rep["interference"]
    L("== interference (boolean intersection volume, threshold %.1f mm3) ==" % i["threshold_mm3"])
    L("  bodies %d, overlapping pairs tested %d, overlaps: %s -> %s" % (i["bodies"], i["pairs_tested"], i["overlaps"], "OK" if i["ok"] else "FAIL"))
    f = rep["fov"]
    L("== field of view (half angles %s deg, margin %.1f deg) ==" % (f["half_fov_deg"], f["margin_deg"]))
    for c, r in f["cams"].items():
        L("  cam %s: ball distance %d mm, ball margin %.2f deg, visible=%s, rays clipped %d/%d %s, shield edge margin %.1f mm"
          % (c, r["ball_distance_mm"], r["ball_margin_deg"], r["ball_visible"], r["rays_clipped"], r["rays"], r["clipped_by"], r["shield_edge_margin_mm"]))
        L("         down-range coverage (mm) by launch angle: %s" % r["downrange_mm_at_launch_deg"])
    L("  both cameras: %s (target %d mm met for launch angles %s) -> %s" % (
        f["both_cameras_downrange_mm_at_launch_deg"], f["target_down_range_mm"], f["launch_angles_meeting_target_deg"], "OK" if f["ok"] else "FAIL"))
    r = rep["led"]
    L("== LED -> lens direct lines (emission half angle %.0f deg) ==" % r["emission_half_angle_deg"])
    for c, v in r["cams"].items():
        L("  as built, cam %s: %d segments to the filter aperture, direct %d, blocked %d (by %s), nearest %.1f deg off LED axis"
          % (c, v["segments"], v["direct_lines"], v["blocked_inside_cone"], v["blocked_by"], v["min_angle_off_led_axis_deg"]))
    for c, v in r["led_mount_alone_bare_lens"].items():
        L("  LED mounts alone (bare lens, camera mounts removed), cam %s: direct %d, blocked %d" % (c, v["direct_lines"], v["blocked_inside_cone"]))
    for c, v in r["control_no_led_mount"].items():
        L("  control without LED mounts, cam %s: direct lines %d of %d" % (c, v["direct_lines"], v["segments"]))
    for c, v in r["control_no_baffle_wing"].items():
        L("  control with LED mounts but without baffle wing, cam %s: direct lines %d of %d" % (c, v["direct_lines"], v["segments"]))
    L("  emitters with a clear path to the ball: %d/%d -> %s" % (r["emitters_with_clear_path_to_ball"], r["emitters"], "OK" if r["ok"] else "FAIL"))
    c = rep["com"]
    L("== head COM and tripod torque ==")
    L("  head mass %d g, COM (BF) %s, stud (world) %s, offset %s -> %.1f mm, torque %.3f N m, stud height %.1f mm at pitch %.0f deg -> %s" % (
        c["head_mass_g"], c["com_bf_mm"], c["stud_world_mm"], c["horizontal_offset_com_to_stud_mm"], c["offset_mm"],
        c["tipping_torque_Nm"], c["stud_height_above_mat_mm"], c["pitch_deg"], "OK" if c["ok"] else "FAIL"))
    L("  recommended stud (BF x, y) for zero offset: %s" % (rep["stud_recommended"],))
    L("  sensitivity: a 20 mm error in the real COM position gives %.3f N m on the tripod head" % (c["head_mass_g"] / 1000 * 9.81 * 0.02))
    L("== extras ==")
    L("  FPC reach: %s -> %s" % (rep["fpc"]["paths_mm"], "OK" if rep["fpc"]["ok"] else "FAIL"))
    L("== printability (unsupported overhangs steeper than 45 deg on the bed-oriented STL files) ==")
    for n, o in rep["overhang"].items():
        L("  %-6s sloped overhang %6.1f mm2, widest bridge %5.1f mm -> %s" % (n, o["sloped_overhang_mm2"], o["max_bridge_span_mm"],
                                                                      "REVIEW" if o["needs_support_review"] else "no supports needed"))
    L("== beam stiffness (hand calculation) ==")
    L("  %s" % (rep["beam"],))
    if "wall_scan" in rep:
        L("== wall thickness scan (rays inward, rule >= %.1f mm; raised text, tongue, ticks and sharp edges show up as thin) ==" % P.MIN_WALL)
        for n, w in rep["wall_scan"].items():
            L("  %-6s thin area %6.1f mm2 (%.1f %%)  worst cells %s" % (n, w["thin_area_mm2"], 100 * w["thin_fraction"],
                                                                 [(c["cell_xyz_mm"], c["min_mm"]) for c in w["worst_cells"][:2]]))
    return "\n".join(lines)


def run_checks(bodies, meshes, parts_info):
    import checks as K
    rep = {}
    rep["stl"] = K.check_stl_files(STL, list(parts_info.keys()))
    rep["interference"] = K.check_interference(bodies, meshes)
    rep["fov"] = K.check_fov(bodies, meshes)
    rep["led"] = K.check_led_los(bodies, meshes)
    rep["com"] = K.check_com(bodies, meshes)
    rep["stud_recommended"] = K.recommend_stud(bodies, meshes)
    rep["fpc"] = K.check_fpc_reach()
    rep["overhang"] = {n: K.overhang_scan(os.path.join(STL, n + ".stl")) for n in parts_info}
    rep["beam"] = K.beam_stiffness(bodies, meshes)
    return rep


def wall_scans(parts):
    import checks as K
    from meshing import to_mesh
    return {n: K.wall_scan(to_mesh(p.wp, 0.05, 0.2)) for n, p in parts.items()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--variant-check", action="store_true", help="print one JSON line (used by sweep.py)")
    ap.add_argument("--quick", action="store_true", help="skip STEP assembly + wall scans")
    a = ap.parse_args()
    t0 = time.time()
    import assembly as A
    import checks as K
    import params as P
    if a.variant_check:
        bodies = A.build_assembly()
        meshes = K.build_meshes(bodies)
        r = dict(interference=K.check_interference(bodies, meshes)["overlaps"], fov=K.check_fov(bodies, meshes),
                 led=K.check_led_los(bodies, meshes), com=K.check_com(bodies, meshes), fpc=K.check_fpc_reach(),
                 beam_len=P.BEAM_LEN)
        print("VARIANT_JSON " + json.dumps(r, default=jdefault))
        return
    os.makedirs(STL, exist_ok=True)
    os.makedirs(STEP, exist_ok=True)
    os.makedirs(DOCS, exist_ok=True)
    print("building parts ...", flush=True)
    parts = A.build_parts()
    info = export_parts(parts)
    print("  %d part designs exported (%.0f s)" % (len(parts), time.time() - t0), flush=True)
    bodies = A.build_assembly(parts=parts)
    meshes = K.build_meshes(bodies)
    if not a.quick:
        export_assembly(bodies)
    print("running checks ...", flush=True)
    rep = run_checks(bodies, meshes, info)
    if not a.quick:
        rep["wall_scan"] = wall_scans(parts)
    rep["parts"] = info
    if not a.no_sweep:
        import sweep
        rep["sweep"] = sweep.run_all()
    with open(os.path.join(EXPORT, "check_report.json"), "w") as f:
        json.dump(rep, f, indent=1, default=jdefault)
    txt = summarize(rep)
    if "sweep" in rep:
        txt += "\n" + sweep.summarize(rep["sweep"])
    with open(os.path.join(EXPORT, "check_report.txt"), "w") as f:
        f.write(txt + "\n")
    print(txt)
    if not a.no_render:
        import previews
        previews.make_all(bodies, meshes, parts, rep)
    print("done in %.0f s" % (time.time() - t0))


if __name__ == "__main__":
    main()
