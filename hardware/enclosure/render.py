"""Headless previews with VTK off-screen rendering (EGL)."""
import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray


def _polydata(mesh):
    pts = vtk.vtkPoints()
    pts.SetData(numpy_to_vtk(np.ascontiguousarray(mesh.vertices, dtype=float), deep=True))
    cells = np.hstack([np.full((len(mesh.faces), 1), 3), mesh.faces]).astype(np.int64).ravel()
    ca = vtk.vtkCellArray()
    ca.SetCells(len(mesh.faces), numpy_to_vtkIdTypeArray(cells, deep=True))
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetPolys(ca)
    n = vtk.vtkPolyDataNormals()
    n.SetInputData(pd)
    n.SetFeatureAngle(35.0)
    n.SplittingOn()
    n.Update()
    return n.GetOutput()


def render(items, path, eye, focal, up=(0, 0, 1), size=(1400, 900), parallel_scale=None,
           bg=(1, 1, 1), view_angle=30.0, clip=None, title=None):
    """items: list of (trimesh, rgb, opacity).  eye/focal in world coordinates."""
    rw = vtk.vtkRenderWindow()
    rw.SetOffScreenRendering(1)
    rw.SetSize(*size)
    rw.SetMultiSamples(4)
    r = vtk.vtkRenderer()
    r.SetBackground(*bg)
    rw.AddRenderer(r)
    for m, col, op in items:
        mp = vtk.vtkPolyDataMapper()
        mp.SetInputData(_polydata(m))
        a = vtk.vtkActor()
        a.SetMapper(mp)
        pr = a.GetProperty()
        pr.SetColor(*col)
        pr.SetOpacity(op)
        pr.SetAmbient(0.25)
        pr.SetDiffuse(0.75)
        pr.SetSpecular(0.15)
        r.AddActor(a)
    cam = r.GetActiveCamera()
    cam.SetPosition(*eye)
    cam.SetFocalPoint(*focal)
    cam.SetViewUp(*up)
    if parallel_scale:
        cam.ParallelProjectionOn()
        cam.SetParallelScale(parallel_scale)
    else:
        cam.SetViewAngle(view_angle)
    r.ResetCameraClippingRange()
    lk = vtk.vtkLightKit()
    lk.AddLightsToRenderer(r)
    rw.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(rw)
    w2i.ReadFrontBufferOff()
    w2i.Update()
    wr = vtk.vtkPNGWriter()
    wr.SetFileName(path)
    wr.SetInputConnection(w2i.GetOutputPort())
    wr.Write()
