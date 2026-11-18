"""Automated design checks (run by build.py).  Every function returns a dict with
`ok` (bool) and the numbers behind it; build.py prints and stores them."""
import itertools
import math
import os
import numpy as np
import trimesh
import manifold3d as m3d

import params as P
import pose
import parts_cam as C
import parts_led as L
import assembly as A
from lib import rot_x, rot_z
from meshing import to_mesh

HU = math.atan(P.SENSOR_W / 2 / P.FOCAL)          # half field of view, horizontal (rad)
HV = math.atan(P.SENSOR_H / 2 / P.FOCAL)          # vertical
MARGIN = math.radians(P.FOV_MARGIN_DEG)
OWN_KINDS = ("PCB", "HOLDER", "FPCCONN", "LENS", "FILTER")


# ---------------------------------------------------------------------------
# 1 + 2: printed part files
# ---------------------------------------------------------------------------
def check_stl_files(stl_dir, names):
    out = {"ok": True, "parts": {}}
    vx, vy, vz = P.BUILD_VOLUME
    for n in names:
        m = trimesh.load(os.path.join(stl_dir, n + ".stl"), process=True)
        ext = m.extents
        fit = bool(ext[0] <= vx and ext[1] <= vy and ext[2] <= vz)
        wt = bool(m.is_watertight)
        wc = bool(m.is_winding_consistent)
        vol_ok = bool(m.volume > 0)
        bodies = len(m.split(only_watertight=False))
        z0 = bool(abs(m.bounds[0][2]) < 1e-3)
        ok = fit and wt and wc and vol_ok and z0
        out["parts"][n] = dict(extents=[round(float(e), 1) for e in ext], fits=fit, watertight=wt,
                               winding=wc, volume_mm3=round(float(m.volume)), bodies=bodies, on_bed=z0, ok=ok)
        out["ok"] &= ok
    return out


# ---------------------------------------------------------------------------
# 3: interference between parts and proxies in the nominal pose
# ---------------------------------------------------------------------------
def _manifold(m):
    mesh = m3d.Mesh(vert_properties=np.asarray(m.vertices, dtype=np.float32),
                    tri_verts=np.asarray(m.faces, dtype=np.uint32))
    return m3d.Manifold(mesh)


def check_interference(bodies, meshes, threshold=0.5):
    mans, pairs, hits, skipped = {}, 0, [], []
    for i, j in itertools.combinations(range(len(bodies)), 2):
        a, b = bodies[i], bodies[j]
        if a.name == "MAT" or b.name == "MAT":
            continue
        ma, mb = meshes[i], meshes[j]
        if np.any(ma.bounds[1] < mb.bounds[0] - 0.01) or np.any(mb.bounds[1] < ma.bounds[0] - 0.01):
            continue
        for k in (i, j):
            if k not in mans:
                try:
                    mans[k] = _manifold(meshes[k])
                except Exception:
                    mans[k] = None
                    skipped.append(bodies[k].name)
        if mans[i] is None or mans[j] is None:
            continue
        pairs += 1
        v = (mans[i] ^ mans[j]).volume()
        if v > threshold:
            hits.append((round(float(v), 2), a.name, b.name))
    hits.sort(reverse=True)
    return dict(ok=len(hits) == 0, bodies=len(bodies), pairs_tested=pairs, threshold_mm3=threshold,
                overlaps=hits, skipped=sorted(set(skipped)))


# ---------------------------------------------------------------------------
# ray casting helpers
# ---------------------------------------------------------------------------
class Scene:
    """All opaque bodies merged into one mesh for embree ray casting."""

    def __init__(self, bodies, meshes, exclude=lambda b: False):
        ms, owner = [], []
        for k, (b, m) in enumerate(zip(bodies, meshes)):
            if b.transparent or b.ignore_clip or exclude(b):
                continue
            ms.append(m)
            owner += [k] * len(m.faces)
        self.mesh = trimesh.util.concatenate(ms)
        self.owner = np.array(owner)
        self.bodies = bodies

    def hits(self, origins, dirs):
        """Distance of the first hit per ray (inf if none) and the hit body name."""
        origins = np.asarray(origins, float)
        dirs = np.asarray(dirs, float)
        dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
        loc, idx_ray, idx_tri = self.mesh.ray.intersects_location(origins, dirs, multiple_hits=False)
        dist = np.full(len(origins), np.inf)
        who = [None] * len(origins)
        for l, r, t in zip(loc, idx_ray, idx_tri):
            d = np.linalg.norm(l - origins[r])
            if d < dist[r]:
                dist[r] = d
                who[r] = self.bodies[self.owner[t]].name
        return dist, who


def cam_frame(side, theta=None):
    M = pose.bf2w_matrix(theta) @ C.kp2bf(side)
    R, t = M[:3, :3], M[:3, 3]
    pupil = R @ np.array([0, P.PUPIL_Y, P.Z_AX]) + t
    return dict(pupil=pupil, right=R[:, 0], fwd=R[:, 1], up=R[:, 2], M=M)


def cam_coords(cam, pts):
    v = np.asarray(pts, float) - cam["pupil"]
    x, y, z = v @ cam["right"], v @ cam["fwd"], v @ cam["up"]
    az = np.arctan2(x, y)
    el = np.arctan2(z, y)
    return az, el, np.linalg.norm(v, axis=1)


def frustum_rays(cam, margin=MARGIN, nu=17, nv=13):
    au, av = HU + margin, HV + margin
    us = np.tan(np.linspace(-au, au, nu))
    vs = np.tan(np.linspace(-av, av, nv))
    d = []
    for u in us:
        for v in vs:
            d.append(u * cam["right"] + cam["fwd"] + v * cam["up"])
    d = np.array(d)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def own_exclude(side):
    tag = "#" + C.side_tag(side)
    return lambda b: b.name.endswith(tag) and b.name.split("#")[0] in OWN_KINDS


# ---------------------------------------------------------------------------
# 4: field of view
# ---------------------------------------------------------------------------
def check_fov(bodies, meshes, theta=None, launch=(0, 15, 30, 45), step=2.0):
    ball = np.array([0.0, 0.0, P.BALL_R])
    res = {"ok": True, "half_fov_deg": [round(math.degrees(HU), 2), round(math.degrees(HV), 2)],
           "margin_deg": P.FOV_MARGIN_DEG, "cams": {}}
    sheet_body = next(b for b in bodies if b.name == "SHIELD")
    Mbf = pose.bf2w_matrix(theta)
    Minv = np.linalg.inv(Mbf)
    ysh = pose.shield_y0()
    z0s = P.Z_BEAM_TOP + 3.0
    covered = {a: [] for a in launch}
    per_cam_cov = {}
    for side in (1, -1):
        cam = cam_frame(side, theta)
        scene = Scene(bodies, meshes, own_exclude(side))
        # --- rays across the whole field of view
        d = frustum_rays(cam)
        tmat = np.where(d[:, 2] < 0, cam["pupil"][2] / np.maximum(-d[:, 2], 1e-9), 1500.0)
        limit = np.minimum(tmat, 1500.0)
        dist, who = scene.hits(np.tile(cam["pupil"], (len(d), 1)), d)
        clipped = dist < limit - 1e-6
        names = sorted(set(w for w, c in zip(who, clipped) if c))
        # --- shield coverage: where does each ray cross the sheet plane (BF)
        pb = (Minv @ np.append(cam["pupil"], 1.0))[:3]
        db = (Minv[:3, :3] @ d.T).T
        t = (ysh + P.SHIELD_T / 2 - pb[1]) / db[:, 1]
        q = pb + db * t[:, None]
        mx = P.SHIELD_W / 2 - np.abs(q[:, 0])
        mz = np.minimum(q[:, 2] - z0s, z0s + P.SHIELD_H - q[:, 2])
        edge = float(min(mx.min(), mz.min()))
        # --- ball at rest: centre + ring of surface points
        pts = [ball]
        for ax in np.eye(3):
            pts += [ball + ax * P.BALL_R, ball - ax * P.BALL_R]
        pts = np.array(pts)
        az, el, dd = cam_coords(cam, pts)
        mball = min(HU - np.abs(az).max(), HV - np.abs(el).max())
        dball, _ = scene.hits(np.tile(cam["pupil"], (len(pts), 1)), pts - cam["pupil"])
        ball_vis = bool(np.all(dball > dd - 1e-6))
        cov = {}
        for a in launch:
            best = -1.0
            for dx in np.arange(0, 600, step):
                c = ball + np.array([dx, 0, dx * math.tan(math.radians(a))])
                # ball-sized target: use angular radius of the ball
                az_, el_, dd_ = cam_coords(cam, [c])
                r = math.atan(P.BALL_R / dd_[0])
                inside = abs(az_[0]) <= HU - MARGIN - r and abs(el_[0]) <= HV - MARGIN - r
                if not inside:
                    break
                dh, _ = scene.hits(cam["pupil"][None], (c - cam["pupil"])[None])
                if dh[0] < dd_[0] - 1e-6:
                    break
                best = float(dx)
            cov[a] = best
        per_cam_cov[side] = cov
        res["cams"][C.side_tag(side)] = dict(
            pupil_world=[round(float(x), 1) for x in cam["pupil"]],
            ball_distance_mm=round(float(dd[0])),
            ball_margin_deg=round(math.degrees(mball), 2),
            ball_visible=ball_vis,
            rays=len(d), rays_clipped=int(clipped.sum()), clipped_by=names,
            shield_edge_margin_mm=round(edge, 1),
            shield_covers_all_rays=bool(edge > 0),
            downrange_mm_at_launch_deg={str(a): cov[a] for a in launch})
        res["ok"] &= bool(ball_vis and mball > 0 and clipped.sum() == 0 and edge > 5.0)
    res["both_cameras_downrange_mm_at_launch_deg"] = {
        str(a): min(per_cam_cov[1][a], per_cam_cov[-1][a]) for a in launch}
    res["target_down_range_mm"] = P.FLIGHT_COVER
    res["launch_angles_meeting_target_deg"] = [a for a in launch
                                               if res["both_cameras_downrange_mm_at_launch_deg"][str(a)] >= P.FLIGHT_COVER]
    return res


# ---------------------------------------------------------------------------
# 5: LED -> lens direct line of sight
# ---------------------------------------------------------------------------
def led_emitters(bodies):
    """Emitter points (world, on the lens tip plane, both boards) and the common LED axis direction."""
    M = pose.bf2w_matrix()
    pts = []
    for side in (1, -1):
        for i in range(P.LED_GRID[0]):
            for j in range(P.LED_GRID[1]):
                if (i, j) in P.LED_SKIP:
                    continue
                x, z = pose.led_star_xz(i, j, side)
                pts.append(M @ np.array([x, pose.led_tip_y() + 0.05, z, 1.0]))
    normal = M[:3, :3] @ np.array([0, 1.0, 0])
    return np.array(pts)[:, :3], normal


def _led_direct(bodies, meshes, em, n, target, drop=lambda b: False):
    """Count LED -> lens segments that are unobstructed and inside the emission cone.
    target 'filter': clear aperture of the filter holder; 'lens': front glass of the bare lens."""
    out = {}
    for side in (1, -1):
        cam = cam_frame(side)
        yf = P.FH_FRONT_Y - 0.2 if target == "filter" else P.LENS_FRONT_Y + 0.1
        rr = (5.0, 10.5) if target == "filter" else (6.0, 12.0)
        samples = [np.array([0, yf, P.Z_AX])]
        for r_ in rr:
            for k in range(8):
                a = 2 * math.pi * k / 8
                samples.append(np.array([r_ * math.cos(a), yf, P.Z_AX + r_ * math.sin(a)]))
        S = np.array([(cam["M"] @ np.append(s, 1.0))[:3] for s in samples])
        own = own_exclude(side)
        ledown = lambda b: b.name.startswith(("LEDLENS", "LEDSTARS", "LEDPLATE"))   # emitters see through their own lens bodies
        scene = Scene(bodies, meshes, lambda b: own(b) or drop(b) or ledown(b))
        O, D, ang = [], [], []
        for e in em:
            for s in S:
                v = s - e
                O.append(e)
                D.append(v)
                ang.append(math.degrees(math.acos(np.clip(np.dot(v, n) / np.linalg.norm(v), -1, 1))))
        O, D, ang = np.array(O), np.array(D), np.array(ang)
        dist, who = scene.hits(O, D)
        free = dist > np.linalg.norm(D, axis=1) - 1e-3
        emitting = ang <= P.LED_HALF_ANGLE
        blockers = {}
        for w, f, e in zip(who, free, emitting):
            if (not f) and e and w:
                blockers[w] = blockers.get(w, 0) + 1
        out[C.side_tag(side)] = dict(
            segments=len(D), direct_lines=int((free & emitting).sum()),
            outside_emission_cone=int((~emitting).sum()), blocked_inside_cone=int((emitting & ~free).sum()),
            min_angle_off_led_axis_deg=round(float(ang.min()), 1),
            blocked_by=dict(sorted(blockers.items(), key=lambda kv: -kv[1])))
    return out


def check_led_los(bodies, meshes):
    em, n = led_emitters(bodies)
    res = {"ok": True, "emitters": len(em)}
    # (A) as built: every opaque part counts; target = clear aperture of each filter holder
    res["cams"] = _led_direct(bodies, meshes, em, n, "filter")
    res["ok"] &= all(v["direct_lines"] == 0 for v in res["cams"].values())
    # (B) the baffle on its own: lens front fully exposed (no filter housing / ring / shield arms)
    hood = lambda b: b.name.split("#")[0] in ("FH-1", "FR-1") or b.name.startswith("SA-1")
    baffle = lambda b: b.name.startswith("LD-2")
    res["baffle_alone_bare_lens"] = _led_direct(bodies, meshes, em, n, "lens", hood)
    res["ok"] &= all(v["direct_lines"] == 0 for v in res["baffle_alone_bare_lens"].values())
    # (C) control: same bare-lens test without the baffle must find direct lines (the test can fail)
    res["control_bare_lens_no_baffle"] = _led_direct(bodies, meshes, em, n, "lens", lambda b: hood(b) or baffle(b))
    res["control_finds_lines"] = any(v["direct_lines"] > 0 for v in res["control_bare_lens_no_baffle"].values())
    # nothing opaque between the LEDs and the ball (illumination not shadowed)
    ball = np.array([0, 0, P.BALL_R])
    scene = Scene(bodies, meshes)
    dist, who = scene.hits(em, np.tile(ball, (len(em), 1)) - em)
    dd = np.linalg.norm(ball - em, axis=1)
    lit = dist > dd - P.BALL_R - 1e-3
    res["emitters_with_clear_path_to_ball"] = int(lit.sum())
    res["ball_illuminated"] = bool(lit.sum() >= len(em) // 2)
    res["ok"] &= res["ball_illuminated"] and res["control_finds_lines"]
    return res


# ---------------------------------------------------------------------------
# 6: centre of mass versus the foot triangle
# ---------------------------------------------------------------------------
def com_at(bodies, meshes, theta, dh=0.0):
    """Centre of mass of the rig (no ball / mat / template) with the head pitched to `theta`.
    dh raises the head by dh mm (taller mast: head and pivot parts rise, the mast gets longer)."""
    Mn = pose.bf2w_matrix()
    Mt = pose.bf2w_matrix(theta)
    T = Mt @ np.linalg.inv(Mn)
    tot, c = 0.0, np.zeros(3)
    for b, m in zip(bodies, meshes):
        if b.group in ("tmpl", "ball", "mat") or b.name in ("EXT-STICK", "TPL-PIN") or b.mass <= 0:
            continue
        p = m.centroid if m.is_watertight else m.vertices.mean(axis=0)
        p = np.append(p, 1.0)
        mass = b.mass
        if b.moves:
            p = T @ p
        if dh:
            if b.moves or b.name in ("PV-1", "MC-1"):
                p[2] += dh
            elif b.name == "EXT-MAST":                   # longer mast: more mass, centroid rises by dh / 2
                mass = b.mass + dh * P.EXT_MASS_PER_M / 1000.0
                p[2] += dh / 2.0
        tot += mass
        c += mass * p[:3]
    return c / tot, tot


def foot_margin(com, feet):
    f = np.array(feet)
    best = 1e9
    for i in range(3):
        a, b = f[i], f[(i + 1) % 3]
        e = b - a
        nrm = np.array([-e[1], e[0]]) / np.linalg.norm(e)
        # make normal point inside
        cen = f.mean(axis=0)
        if np.dot(nrm, cen - a) < 0:
            nrm = -nrm
        best = min(best, float(np.dot(nrm, com[:2] - a)))
    return best


def check_stability(bodies, meshes):
    feet = A.FOOT_POS
    out = {"ok": True, "feet_xy": [[round(float(x), 1), round(float(y), 1)] for x, y in feet], "cases": {}}
    for th in (0.0, P.PITCH_NOM, P.PITCH_MAX):
        com, mass = com_at(bodies, meshes, th)
        mg = foot_margin(com, feet)
        out["cases"][str(th)] = dict(com=[round(float(x), 1) for x in com], total_mass_g=round(mass),
                                      margin_mm=round(mg, 1))
        out["ok"] &= mg >= P.STABILITY_MARGIN
    com, _ = com_at(bodies, meshes, P.PITCH_MAX)
    mg = foot_margin(com, feet)
    out["tip_angle_deg_at_max_pitch"] = round(math.degrees(math.atan2(mg, com[2])), 1)
    # tallest set-up: lens axis 300 mm at the steepest pitch (foot triangle unchanged)
    com, mass = com_at(bodies, meshes, P.PITCH_MAX, 300.0 - P.LENS_HEIGHT)
    mg = foot_margin(com, feet)
    out["cases"]["lens 300 mm, pitch %.0f" % P.PITCH_MAX] = dict(
        com=[round(float(x), 1) for x in com], total_mass_g=round(mass), margin_mm=round(mg, 1))
    out["tip_angle_deg_at_300mm_max_pitch"] = round(math.degrees(math.atan2(mg, com[2])), 1)
    out["ok"] &= mg >= P.STABILITY_MARGIN
    return out


# ---------------------------------------------------------------------------
# 7: extras
# ---------------------------------------------------------------------------
def check_fpc_reach():
    """Length of a ribbon routed Pi CSI connector -> over the camera bridge -> camera connector."""
    out = {"ok": True, "paths_mm": {}}
    xc, zc = A.PI_POS
    y_b = P.REAR_PLATE_Y - P.PLATE_T - P.BOARD_STANDOFF - P.PI_PCB[2] - 2.5
    for k, (u, v) in enumerate(P.PI_CSI):
        pc = np.array([xc - (u - P.PI_PCB[0] / 2), y_b, zc - (v - P.PI_PCB[1] / 2)])
        for side in (1, -1):
            kp = C.kp2bf(side)
            bridge = (kp @ np.array([0, 14.0, P.Z_AX + 31.0 + 4.0, 1.0]))[:3]
            top = (kp @ np.array([0, P.PCB_BACK_Y - P.CAM_REAR_H, P.Z_AX + P.CAM_PCB / 2 + 6.0, 1.0]))[:3]
            conn = (kp @ np.array([0, P.PCB_BACK_Y - P.CAM_REAR_H / 2, P.Z_AX + P.CAM_PCB / 2 - 2.0, 1.0]))[:3]
            pts = [pc, pc + np.array([0, 0, 12.0]), bridge, top, conn]
            ln = sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1)) + 20.0   # + slack/bends
            out["paths_mm"]["CSI%d->%s" % (k, C.side_tag(side))] = round(float(ln))
    # best assignment: CSI connector 0 / 1 to the two cameras
    ps = out["paths_mm"]
    a = max(ps["CSI0->R"], ps["CSI1->L"])
    b = max(ps["CSI0->L"], ps["CSI1->R"])
    out["best_assignment_longest_mm"] = int(min(a, b))
    out["limit_mm"] = P.CAM_FPC_LEN
    out["ok"] = out["best_assignment_longest_mm"] <= P.CAM_FPC_LEN - 10
    return out


def check_glare(bodies):
    """Specular reflection of the LEDs on the shield sheet into the lenses."""
    em, _ = led_emitters(bodies)
    Mbf = pose.bf2w_matrix()
    Minv = np.linalg.inv(Mbf)
    ysh = pose.shield_y0()
    res = {"ok": True, "cams": {}}
    for side in (1, -1):
        cam = cam_frame(side)
        worst = 1e9
        risk = 0
        for e in em:
            eb = (Minv @ np.append(e, 1.0))[:3]
            for yplane in (ysh, ysh + P.SHIELD_T):
                mirror = np.array([eb[0], 2 * yplane - eb[1], eb[2]])
                mw = (Mbf @ np.append(mirror, 1.0))[:3]
                az, el, _ = cam_coords(cam, [mw])
                off = max(abs(az[0]) - HU, abs(el[0]) - HV)
                worst = min(worst, off)
                if off < 0:
                    pb = (Minv @ np.append(cam["pupil"], 1.0))[:3]
                    t = (yplane - pb[1]) / (mirror[1] - pb[1])
                    q = pb + (mirror - pb) * t
                    if abs(q[0]) < P.SHIELD_W / 2 and P.Z_BEAM_TOP + 3 < q[2] < P.Z_BEAM_TOP + 3 + P.SHIELD_H:
                        risk += 1
        res["cams"][C.side_tag(side)] = dict(reflections_inside_fov_on_sheet=risk,
                                             closest_to_fov_edge_deg=round(math.degrees(worst), 1))
        res["ok"] &= risk == 0
    return res


def lens_height_table():
    """Which lens heights / pitch angles the printed hub + collar geometry can reach."""
    rows = []
    tower_top = P.Z_PAD_TOP + P.TOWER_H + 2.0
    for th in (P.PITCH_MIN, P.PITCH_NOM, P.PITCH_MAX):
        off = pose.pitch_pupil_offset(th)[1]
        # collar bottom = pivot + MC_FLANGE_BOTTOM_PF - collar height must clear the hub tower
        pz_min = tower_top + P.MC_COLLAR_H - P.MC_FLANGE_BOTTOM_PF
        rows.append(dict(pitch_deg=th, min_lens_height_mm_feet_retracted=round(pz_min + off),
                         min_lens_height_mm_nominal_feet=round(pz_min + P.FOOT_GAP_NOM + off)))
    return dict(rows=rows, cut_mast_mm=round(pose.MAST_LEN, 1),
                height_with_this_mast_mm={str(th): [round(pose.lens_height_for(th, g)) for g in (0, P.FOOT_GAP_NOM, 30)]
                                           for th in (P.PITCH_MIN, P.PITCH_NOM, P.PITCH_MAX)})


def build_meshes(bodies, tol=0.03):
    out = []
    for b in bodies:
        if b.name == "BALL":      # OCC spheres tessellate with a seam; use a clean icosphere
            m = trimesh.creation.icosphere(subdivisions=4, radius=P.BALL_R)
            m.apply_translation([0, 0, P.BALL_R])
            out.append(m)
        else:
            out.append(to_mesh(b.wp, tol, 0.15))
    return out


# ---------------------------------------------------------------------------
# wall thickness scan (local thickness measured by shooting rays inwards)
# ---------------------------------------------------------------------------
def wall_scan(mesh, n=6000, min_wall=None, seed=1):
    """Fraction of the surface (by area) whose local thickness is below the design rule,
    plus the cells (10 mm grid) where it concentrates."""
    min_wall = P.MIN_WALL if min_wall is None else min_wall
    rng = np.random.default_rng(seed)
    pts, fidx = trimesh.sample.sample_surface(mesh, n, seed=seed)
    nrm = mesh.face_normals[fidx]
    org = pts - nrm * 0.02
    loc, ir, it = mesh.ray.intersects_location(org, -nrm, multiple_hits=False)
    th = np.full(n, np.inf)
    th[ir] = np.linalg.norm(loc - pts[ir], axis=1)
    thin = th < min_wall - 0.05
    area = mesh.area
    cells = {}
    for p, t in zip(pts[thin], th[thin]):
        k = tuple((p // 10).astype(int))
        c = cells.setdefault(k, [0, 9e9])
        c[0] += 1
        c[1] = min(c[1], t)
    top = sorted(cells.items(), key=lambda kv: -kv[1][0])[:4]
    return dict(thin_area_mm2=round(float(thin.mean() * area), 1), thin_fraction=round(float(thin.mean()), 4),
                min_thickness_mm=round(float(th.min()), 2),
                worst_cells=[dict(cell_xyz_mm=[int(a) * 10 for a in k], samples=v[0], min_mm=round(float(v[1]), 2)) for k, v in top])


# ---------------------------------------------------------------------------
# motion ranges: adjustment joints swept through their limits
# ---------------------------------------------------------------------------
def _rot(mesh, angle_deg, axis_point, axis_dir):
    m = mesh.copy()
    m.apply_transform(trimesh.transformations.rotation_matrix(math.radians(angle_deg), axis_dir, axis_point))
    return m


def _overlap_pairs(group, others, threshold=0.5):
    """group / others: lists of (name, mesh).  Returns overlaps above threshold (mm3)."""
    hits = []
    man_o = {}
    for gn, gm in group:
        try:
            mg = _manifold(gm)
        except Exception:
            continue
        for on, om in others:
            if np.any(gm.bounds[1] < om.bounds[0] - 0.01) or np.any(om.bounds[1] < gm.bounds[0] - 0.01):
                continue
            if on not in man_o:
                try:
                    man_o[on] = _manifold(om)
                except Exception:
                    man_o[on] = None
            if man_o[on] is None:
                continue
            v = (mg ^ man_o[on]).volume()
            if v > threshold:
                hits.append((round(float(v), 2), gn, on))
    return sorted(hits, reverse=True)


def check_motion(bodies, meshes):
    """Sweep the adjustment joints through their limits and look for new overlaps:
    head pitch (-3..30 deg), camera yaw (+-YAW_RANGE), camera tilt (+-TILT_RANGE)."""
    named = {b.name: (b, m) for b, m in zip(bodies, meshes)}
    skip = lambda b: b.name in ("MAT", "BALL", "EXT-STICK", "TPL-PIN") or b.group == "tmpl"
    res = {"ok": True, "cases": {}}
    # --- head pitch about the pitch axis
    Mn = pose.bf2w_matrix()
    moving = [(b.name, m) for b, m in zip(bodies, meshes) if b.moves and not skip(b)]
    fixed = [(b.name, m) for b, m in zip(bodies, meshes) if (not b.moves) and not skip(b)]
    for th in (PIV_MIN, 0.0, P.PITCH_MIN, P.PITCH_MAX, PIV_MAX):
        d = -(th - P.PITCH_NOM)           # rotation about +X (nose down is negative)
        g = [(n, _rot(m, d, pose.PIVOT_W, (1, 0, 0))) for n, m in moving]
        h = _overlap_pairs(g, fixed)
        res["cases"]["head pitch %.0f deg" % th] = h
        res["ok"] &= not h
    # --- camera stage joints
    for side in (1, -1):
        tag = C.side_tag(side)
        M = pose.bf2w_matrix() @ C.kp2bf(side)
        R, t = M[:3, :3], M[:3, 3]
        yaw_names = ["YS-1#", "CP-1#", "FH-1#", "FR-1#", "PCB#", "HOLDER#", "FPCCONN#", "LENS#", "FILTER#"]
        tilt_names = yaw_names[1:]
        yh = (P.CP_BACK_Y + P.CP_FRONT_Y) / 2
        for kind, names, p0, dirv, rng in (
                ("yaw", yaw_names, t, R @ np.array([0, 0, 1.0]), P.YAW_RANGE),
                ("tilt", tilt_names, M @ np.array([0, yh, P.Z_AX, 1.0]), R @ np.array([1.0, 0, 0]), P.TILT_RANGE)):
            grp = [(n + tag, named[n + tag][1]) for n in names]
            gset = set(n for n, _ in grp)
            others = [(b.name, m) for b, m in zip(bodies, meshes) if b.name not in gset and not skip(b)]
            for sgn in (-1, 1):
                g = [(n, _rot(m, sgn * rng, p0[:3], dirv)) for n, m in grp]
                h = _overlap_pairs(g, others)
                res["cases"]["camera %s %s %+.0f deg" % (tag, kind, sgn * rng)] = h
                res["ok"] &= not h
    return res


PIV_MIN, PIV_MAX = P.PIV_RANGE


# ---------------------------------------------------------------------------
# hand-calculation of the beam stiffness (the beam, not the plastic, sets the baseline)
# ---------------------------------------------------------------------------
def beam_stiffness(bodies, meshes):
    from lib import ext_proxy
    # second moment of area of the 2020 profile from a 1 mm slice of the proxy (about the horizontal bending axis)
    sl = to_mesh(ext_proxy((0, 0, 0), (1, 0, 0), 1.0), 0.005, 0.05)
    A_sec = sl.volume / 1.0
    Iyy = sl.moment_inertia[1][1] / 1.0 - A_sec * 1.0 ** 2 / 12.0        # mm4 (x^2 term of the slice removed)
    E = 70000.0                                                           # N/mm2 aluminium
    g = 9.81 / 1000.0
    Mn = pose.bf2w_matrix()
    loads = []
    for b, m in zip(bodies, meshes):
        if not b.moves or b.mass <= 0 or b.name == "EXT-BEAM" or b.group == "pivot":
            continue
        c = (np.linalg.inv(Mn) @ np.append(m.centroid, 1.0))[:3]
        loads.append((c[0], b.mass * g))
    xp = pose.PIVOT_X
    out = {}
    for name, xq in (("right camera", P.BASELINE / 2.0), ("left camera", -P.BASELINE / 2.0),
                     ("LED board (right)", P.LED_X)):
        d = 0.0
        for xl, f in loads:
            if (xl - xp) * (xq - xp) <= 0:
                continue                                   # other side of the clamp
            a, b_ = abs(xl - xp), abs(xq - xp)
            d += f * (a ** 2 * (3 * b_ - a) if a <= b_ else b_ ** 2 * (3 * a - b_)) / (6 * E * Iyy)
        out[name.replace(" ", "_") + "_sag_um"] = round(d * 1000, 1)
    return dict(area_mm2=round(float(A_sec), 1), I_mm4=round(float(Iyy)), E_N_per_mm2=E,
                head_load_N=round(sum(f for _, f in loads), 1), **out,
                note="hand calculation: beam treated as clamped at the pitch drum with every head mass as a point "
                     "load; printed camera stages are NOT included (they set the pointing, so the stereo "
                     "extrinsics must be calibrated in software)")


# ---------------------------------------------------------------------------
# printability: unsupported overhang scan on the exported (print-orientation) STL files
# ---------------------------------------------------------------------------
def overhang_scan(stl_path, bridge_limit_mm=30.0, sloped_limit_mm2=400.0):
    """Faces looking downwards steeper than 45 deg (n_z < -0.72) that do not touch the bed.
    Horizontal ones are bridges; their span is estimated by the hydraulic diameter 4A/P of each
    patch (a slit 99 x 1.9 mm bridges 3.7 mm, a round hole bridges its diameter)."""
    m = trimesh.load(stl_path, process=True)
    nz = m.face_normals[:, 2]
    zc = m.triangles_center[:, 2]
    above = zc > 0.15
    slope = above & (nz < -0.72) & (nz >= -0.98)
    flat = np.where(above & (nz < -0.98))[0]
    out = dict(sloped_overhang_mm2=round(float(m.area_faces[slope].sum()), 1),
               horizontal_mm2=round(float(m.area_faces[flat].sum()), 1))
    span, worst = 0.0, None
    if len(flat):
        adj = m.face_adjacency
        sel = np.isin(adj[:, 0], flat) & np.isin(adj[:, 1], flat)
        import networkx as nx
        g = nx.Graph()
        g.add_nodes_from(flat.tolist())
        g.add_edges_from(adj[sel].tolist())
        for comp in nx.connected_components(g):
            comp = np.array(sorted(comp))
            area = m.area_faces[comp].sum()
            ed = m.edges[np.concatenate([np.arange(3) + 3 * f for f in comp])] if False else None
            tri = m.faces[comp]
            e = np.sort(np.vstack([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]]), axis=1)
            u, cnt = np.unique(e, axis=0, return_counts=True)
            b = u[cnt == 1]
            per = np.linalg.norm(m.vertices[b[:, 0]] - m.vertices[b[:, 1]], axis=1).sum()
            dh = 4 * area / max(per, 1e-9)
            if dh > span:
                span, worst = dh, (round(float(area), 1), np.round(m.triangles_center[comp].mean(axis=0), 1).tolist())
    out["max_bridge_span_mm"] = round(float(span), 1)
    out["widest_bridge_patch"] = worst
    out["needs_support_review"] = bool(out["sloped_overhang_mm2"] > sloped_limit_mm2 or span > bridge_limit_mm)
    return out
