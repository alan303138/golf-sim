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
import parts as PT
import assembly as A
from lib import rot_x, rot_z
from meshing import to_mesh

HU = math.atan(P.SENSOR_W / 2 / P.FOCAL)          # half field of view, horizontal (rad)
HV = math.atan(P.SENSOR_H / 2 / P.FOCAL)          # vertical
MARGIN = math.radians(P.FOV_MARGIN_DEG)
OWN_KINDS = ("PCB", "HOLDER", "FPCCONN", "LENS", "FILTER")
tag = PT.tag



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
    M = pose.bf2w_matrix(theta) @ pose.k2bf(side)
    R, t = M[:3, :3], M[:3, 3]
    pupil = R @ np.array([0, P.PUPIL_K, 0]) + t
    return dict(pupil=pupil, right=R[:, 0], fwd=R[:, 1], up=R[:, 2], M=M)


def cam_coords(cam, pts):
    v = np.asarray(pts, float) - cam["pupil"]
    x, y, z = v @ cam["right"], v @ cam["fwd"], v @ cam["up"]
    return np.arctan2(x, y), np.arctan2(z, y), np.linalg.norm(v, axis=1)


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
    t = "#" + tag(side)
    return lambda b: b.name.endswith(t) and b.name.split("#")[0] in OWN_KINDS




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
        mz = np.minimum(q[:, 2] - P.SHIELD_Z[0], P.SHIELD_Z[1] - q[:, 2])
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
        res["cams"][tag(side)] = dict(
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
# LED -> lens direct line of sight
# ---------------------------------------------------------------------------
def led_emitters():
    M = pose.bf2w_matrix()
    pts = []
    for side in (1, -1):
        for i in range(P.LED_GRID[0]):
            for j in range(P.LED_GRID[1]):
                if (i, j) in P.LED_SKIP:
                    continue
                x, z = pose.led_star_xz(i, j, side)
                pts.append(M @ np.array([x, pose.led_tip_y() + 0.05, z, 1.0]))
    return np.array(pts)[:, :3], M[:3, :3] @ np.array([0, 1.0, 0])


def _led_direct(bodies, meshes, em, n, target, drop=lambda b: False):
    out = {}
    for side in (1, -1):
        cam = cam_frame(side)
        yf = P.HOOD_FRONT_K - 0.2 if target == "filter" else P.LENS_FRONT_K + 0.1
        rr = (5.0, 10.5) if target == "filter" else (6.0, 12.0)
        samples = [np.array([0, yf, 0.0])]
        for r_ in rr:
            for k in range(8):
                a = 2 * math.pi * k / 8
                samples.append(np.array([r_ * math.cos(a), yf, r_ * math.sin(a)]))
        S = np.array([(cam["M"] @ np.append(s, 1.0))[:3] for s in samples])
        own = own_exclude(side)
        ledown = lambda b: b.name.startswith(("LEDLENS", "LEDSTARS", "LEDPLATE"))
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
        out[tag(side)] = dict(segments=len(D), direct_lines=int((free & emitting).sum()),
                              outside_emission_cone=int((~emitting).sum()), blocked_inside_cone=int((emitting & ~free).sum()),
                              min_angle_off_led_axis_deg=round(float(ang.min()), 1),
                              blocked_by=dict(sorted(blockers.items(), key=lambda kv: -kv[1])))
    return out


def check_led_los(bodies, meshes):
    em, n = led_emitters()
    res = {"ok": True, "emitters": len(em), "emission_half_angle_deg": P.LED_HALF_ANGLE}
    # (A) as built, target = clear aperture of each filter pocket
    res["cams"] = _led_direct(bodies, meshes, em, n, "filter")
    res["ok"] &= all(v["direct_lines"] == 0 for v in res["cams"].values())
    # (B) the LED mount alone: camera mounts removed, target = bare front glass of the lens
    hood = lambda b: b.name.startswith("CM-1")
    res["led_mount_alone_bare_lens"] = _led_direct(bodies, meshes, em, n, "lens", hood)
    res["ok"] &= all(v["direct_lines"] == 0 for v in res["led_mount_alone_bare_lens"].values())
    # (C) control: same test with the baffle wing removed from both LED mounts must find direct lines
    keep = [k for k, b in enumerate(bodies) if not b.name.startswith("LT-1")]
    # (C2) control: LED mounts removed altogether
    res["control_no_led_mount"] = _led_direct([bodies[k] for k in keep], [meshes[k] for k in keep], em, n, "lens", hood)
    res["control_finds_lines"] = any(v["direct_lines"] > 0 for v in res["control_no_led_mount"].values())
    nowing = []
    for s in (1, -1):
        p = PT.led_tray(s, wing=False)
        wp = A.apply(p.wp, pose.bf2w_matrix())
        nowing.append((A.Body("LT-1" + tag(s) + "-nowing", wp, "printed", "led"), to_mesh(wp, 0.03, 0.15)))
    b2 = [bodies[k] for k in keep] + [x[0] for x in nowing]
    m2 = [meshes[k] for k in keep] + [x[1] for x in nowing]
    res["control_no_baffle_wing"] = _led_direct(b2, m2, em, n, "lens", hood)
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
# centre of mass and tripod-head torque
# ---------------------------------------------------------------------------
def com_info(bodies, meshes, theta=None):
    Mbf = pose.bf2w_matrix(theta)
    Minv = np.linalg.inv(Mbf)
    m_tot, acc = 0.0, np.zeros(3)
    for b, m in zip(bodies, meshes):
        if not b.moves or b.mass <= 0 or b.name.startswith("TRIPOD"):
            continue
        m_tot += b.mass
        acc += b.mass * m.centroid
    cw = acc / m_tot
    cb = (Minv @ np.append(cw, 1.0))[:3]
    S = np.array([P.TA_X, P.TA_Y, P.Z_BEAM_BOT - P.TA_T, 1.0])
    sw = (Mbf @ S)[:3]
    return m_tot, cw, cb, sw


def check_com(bodies, meshes):
    m, cw, cb, sw = com_info(bodies, meshes)
    off = cw[:2] - sw[:2]
    d = float(np.linalg.norm(off))
    g = 9.81
    torque = m / 1000.0 * g * d / 1000.0               # N m
    out = dict(head_mass_g=round(m), com_bf_mm=[round(float(x), 1) for x in cb], com_world_mm=[round(float(x), 1) for x in cw],
               stud_world_mm=[round(float(x), 1) for x in sw], horizontal_offset_com_to_stud_mm=[round(float(x), 1) for x in off],
               offset_mm=round(d, 1), tipping_torque_Nm=round(torque, 3),
               stud_height_above_mat_mm=round(float(sw[2]), 1), pitch_deg=P.PITCH_NOM)
    # pitch sweep (torque with the stud fixed in the beam)
    sweep = {}
    for th in (10, 17, 25):
        m_, cw_, cb_, sw_ = com_info(bodies, meshes, th) if False else (None, None, None, None)
    out["ok"] = bool(torque < 0.15 and d < 25.0)
    out["note"] = "COM from meshes of printed parts and proxies (printed mass at %d%% infill estimate); " \
                  "recommended stud position in BF for zero offset is reported by recommend_stud()" % round(P.INFILL_EFF * 100)
    return out


def recommend_stud(bodies, meshes):
    """BF (x, y) of the stud for which the COM sits above it at nominal pitch (linear in y)."""
    m, cw, cb, sw = com_info(bodies, meshes)
    Mbf = pose.bf2w_matrix()
    zb = P.Z_BEAM_BOT - P.TA_T
    # world y of a stud at BF (x, y, zb): row 1 of Mbf
    r = Mbf[1]
    y_bf = (cw[1] - r[2] * zb - r[3]) / r[1]
    return float(cb[0]), float(y_bf)


# ---------------------------------------------------------------------------
# FPC routes
# ---------------------------------------------------------------------------
def check_fpc_reach():
    out = {"ok": True, "paths_mm": {}}
    g = PT.box_geom()
    zs = P.PI_Z + P.PI_PCB[1] / 2 + 4.0 + 1.75
    y_conn = g["yb"] - P.BOARD_STANDOFF - P.PI_PCB[2] - 2.0
    tie = []
    for side in (1, -1):
        K = pose.k2bf(side)
        ear_x = -side * (P.HOOD_HALF + 4.5)
        tie.append((K @ np.array([ear_x, -12.0, 10.0, 1.0]))[:3])
    for k, (u, v) in enumerate(P.PI_CSI):
        x, z = PT.pi_xz(u, v)
        pc = np.array([x, y_conn, z])
        for si, side in enumerate((1, -1)):
            K = pose.k2bf(side)
            conn = (K @ np.array([0, -P.CAM_PCB_T - 2.5, P.CAM_PCB / 2 - 2.0, 1.0]))[:3]
            top = (K @ np.array([0, -P.CAM_PCB_T - P.CAM_REAR_H - 4.0, P.CAM_PCB / 2 + 5.0, 1.0]))[:3]
            up = np.array([x, y_conn, zs])
            out_front = np.array([x, P.Y_BEAM_REAR + 6.0, zs])
            above = np.array([tie[si][0], tie[si][1] - 4.0, zs - 10.0])
            pts = [pc, up, out_front, above, tie[si], top, conn]
            ln = sum(np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1)) + 20.0
            out["paths_mm"]["CSI%d->%s" % (k, tag(side))] = round(float(ln))
    ps = out["paths_mm"]
    a = max(ps["CSI0->R"], ps["CSI1->L"])
    b = max(ps["CSI0->L"], ps["CSI1->R"])
    out["best_assignment_longest_mm"] = int(min(a, b))
    out["limit_mm"] = P.CAM_FPC_LEN
    out["ok"] = out["best_assignment_longest_mm"] <= P.CAM_FPC_LEN - 10
    return out


def build_meshes(bodies, tol=0.03):
    out = []
    for b in bodies:
        if b.name == "BALL":
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


def beam_stiffness(bodies, meshes):
    from lib import ext_proxy
    sl = to_mesh(ext_proxy((0, 0, 0), (1, 0, 0), 1.0), 0.005, 0.05)
    A_sec = sl.volume
    Iyy = sl.moment_inertia[1][1] - A_sec / 12.0
    E = 70000.0
    g = 9.81 / 1000.0
    Mn = pose.bf2w_matrix()
    loads = []
    for b, m in zip(bodies, meshes):
        if not b.moves or b.mass <= 0 or b.name in ("EXT-BEAM",) or b.name.startswith("TRIPOD"):
            continue
        c = (np.linalg.inv(Mn) @ np.append(m.centroid, 1.0))[:3]
        loads.append((c[0], b.mass * g))
    xp = P.TA_X
    out = {}
    for name, xq in (("right camera", P.BASELINE / 2.0), ("left camera", -P.BASELINE / 2.0), ("right LED board", P.LED_X)):
        d = 0.0
        for xl, f in loads:
            if (xl - xp) * (xq - xp) <= 0:
                continue
            a, b_ = abs(xl - xp), abs(xq - xp)
            d += f * (a ** 2 * (3 * b_ - a) if a <= b_ else b_ ** 2 * (3 * a - b_)) / (6 * E * Iyy)
        out[name.replace(" ", "_") + "_sag_um"] = round(d * 1000, 1)
    return dict(area_mm2=round(float(A_sec), 1), I_mm4=round(float(Iyy)), E_N_per_mm2=E,
                head_load_N=round(sum(f for _, f in loads), 1), **out,
                note="hand calculation: beam clamped at the tripod stud, head masses as point loads; "
                     "the cameras' pointing is set by the printed mounts, extrinsics must be calibrated in software")
