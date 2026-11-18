"""Tessellation helpers (cadquery shape -> trimesh)."""
import numpy as np
import trimesh
import cadquery as cq


def to_mesh(wp, tol=0.05, ang=0.2):
    s = wp.val() if isinstance(wp, cq.Workplane) else wp
    v, f = s.tessellate(tol, ang)
    V = np.array([[p.x, p.y, p.z] for p in v], float)
    F = np.array(f, int)
    m = trimesh.Trimesh(V, F, process=True)
    m.merge_vertices()
    return m
