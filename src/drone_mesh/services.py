"""High-level meshing services: STEP import, quality reporting, and file export.

These services wrap the Gmsh Python API so the pipeline core stays focused on
orchestration. Each service is a thin, testable unit.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from drone_mesh.exceptions import MeshGenerationError, StepImportError

if TYPE_CHECKING:  # pragma: no cover - typing only
    import gmsh  # type: ignore


def import_step(gmsh: "gmsh", step_path: str | Path) -> int:
    """Import a STEP file and return the number of solid volumes found."""
    step_path = Path(step_path)
    if not step_path.exists():
        raise StepImportError(f"STEP file not found: {step_path}")
    gmsh.model.occ.importShapes(str(step_path))
    gmsh.model.occ.synchronize()
    vols = [t for _, t in gmsh.model.getEntities(3)]
    if not vols:
        raise StepImportError(f"STEP file {step_path} contains no solid volumes")
    return len(vols)


def fuse_volumes(gmsh: "gmsh") -> None:
    """Fuse multiple volumes into a single solid (idempotent when already one)."""
    vols = gmsh.model.getEntities(3)
    if len(vols) > 1:
        gmsh.model.occ.fuse(vols[:1], vols[1:])
        gmsh.model.occ.synchronize()


def classify_boundary_surfaces(
    gmsh: "gmsh",
    inlet_x: float,
    outlet_x: float,
    far_bounds: tuple[float, float, float, float, float, float],
    tol: float,
) -> dict[str, list[int]]:
    """Classify surface tags into inlet/outlet/farfield/body by bounding box."""
    x0, y0, z0, x1, y1, z1 = far_bounds
    faces: dict[str, list[int]] = {"inlet": [], "outlet": [], "farfield": [], "body": []}
    for _, t in gmsh.model.getEntities(2):
        b = gmsh.model.getBoundingBox(2, t)
        if abs(b[0] - inlet_x) < tol and abs(b[3] - inlet_x) < tol:
            faces["inlet"].append(t)
        elif abs(b[0] - outlet_x) < tol and abs(b[3] - outlet_x) < tol:
            faces["outlet"].append(t)
        elif (
            (abs(b[1] - y0) < tol and abs(b[4] - y0) < tol)
            or (abs(b[1] - y1) < tol and abs(b[4] - y1) < tol)
            or (abs(b[2] - z0) < tol and abs(b[5] - z0) < tol)
            or (abs(b[2] - z1) < tol and abs(b[5] - z1) < tol)
        ):
            faces["farfield"].append(t)
        else:
            faces["body"].append(t)
    return faces


def quality_report(
    gmsh: "gmsh",
    name: str,
    out_dir: str | Path = "mesh",
    *,
    min_sicn_threshold: float = 0.1,
) -> dict:
    """Compute element-quality metrics and write a JSON report.

    Returns the report dict so callers can assert on quality programmatically.
    """
    _, tags, _ = gmsh.model.mesh.getElements(3)
    all_tags = np.concatenate([np.asarray(t) for t in tags])
    rep: dict = {"elements": int(len(all_tags))}
    for q in ("minSICN", "gamma", "minSJ"):
        v = np.array(gmsh.model.mesh.getElementQualities(all_tags, q))
        rep[q] = {
            "min": float(v.min()),
            "mean": float(v.mean()),
            "p1": float(np.percentile(v, 1)),
        }
    g = np.array(gmsh.model.mesh.getElementQualities(all_tags, "gamma"))
    rep["aspect_ratio_proxy_max"] = float(1.0 / max(g.min(), 1e-12))
    rep["bad_elements_minSICN_lt_0.1"] = int(
        (np.array(gmsh.model.mesh.getElementQualities(all_tags, "minSICN")) < min_sicn_threshold).sum()
    )
    os.makedirs(out_dir, exist_ok=True)
    out_path = Path(out_dir) / f"quality_{name}.json"
    with open(out_path, "w") as fh:
        json.dump(rep, fh, indent=2)
    print(f"[{name}]", json.dumps(rep, indent=2))
    return rep


def export_mesh(
    gmsh: "gmsh",
    path: str | Path,
    *,
    version: float = 4.1,
    binary: bool = False,
) -> None:
    """Write the mesh to disk, configuring format options first."""
    gmsh.option.setNumber("Mesh.MshFileVersion", version)
    gmsh.option.setNumber("Mesh.Binary", 1 if binary else 0)
    gmsh.write(str(path))


def run_mesh_generation(gmsh: "gmsh", dim: int = 3) -> None:
    """Generate the mesh; raise on failure."""
    ok = gmsh.model.mesh.generate(dim)
    if not ok:
        raise MeshGenerationError(f"gmsh.model.mesh.generate({dim}) failed")


def optimize_mesh(gmsh: "gmsh", method: str = "Netgen") -> None:
    """Run a mesh optimizer. Non-fatal if the method is unavailable."""
    try:
        gmsh.model.mesh.optimize(method)
    except Exception as exc:  # pragma: no cover - gmsh-specific
        print(f"[warn] mesh optimize '{method}' skipped: {exc}")