"""Pipeline core: configuration and orchestration for the drone mesh pipeline.

This module ties together the low-level Gmsh services in :mod:`drone_mesh.services`
and the mesh-size field helpers in :mod:`drone_mesh.algorithms` into a single
configurable pipeline. It never imports ``gmsh`` directly; the Gmsh session is
passed in by the caller (see :func:`drone_mesh.cli.main`), which keeps the core
importable on machines without Gmsh installed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

from drone_mesh.algorithms import (
    box_field,
    compose_background,
    distance_field,
    threshold_field,
)
from drone_mesh.exceptions import MeshError, MeshQualityError
from drone_mesh.services import (
    classify_boundary_surfaces,
    export_mesh,
    fuse_volumes,
    import_step,
    optimize_mesh,
    quality_report,
    run_mesh_generation,
)

if TYPE_CHECKING:  # pragma: no cover - typing only
    import gmsh  # type: ignore


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------


@dataclass
class MeshConfig:
    """All tunable parameters of a single meshing run.

    Every mesh size is expressed as a multiple of a base size, so a single
    ``refine`` factor scales the whole mesh uniformly for convergence studies.
    """

    refine: float = 1.0
    """Global mesh-size scale factor (0.5 = finer, 2.0 = coarser)."""

    out_dir: str | Path = "mesh"
    """Directory for output meshes and quality reports."""

    # --- structural (CalculiX) parameters ---
    struct_base_size: float = 2.0
    """Base element size for the structural mesh, before ``refine`` scaling."""

    struct_min_size: float = 0.05
    """Minimum element size allowed in the structural mesh."""

    struct_max_size: float = 5.0
    """Maximum element size allowed in the structural mesh."""

    # --- flow (OpenFOAM) parameters ---
    flow_base_size: float = 1.0
    """Base element size for the flow mesh, before ``refine`` scaling."""

    flow_boundary_thickness: float = 0.2
    """Boundary-layer thickness for the flow mesh refinement box."""

    flow_farfield_size: float = 4.0
    """Element size in the far-field region of the flow mesh."""

    # --- boundary classification ---
    inlet_x: float = 0.0
    """X-coordinate used to identify inlet surfaces."""

    outlet_x: float = 1.0
    """X-coordinate used to identify outlet surfaces."""

    far_bounds: tuple[float, float, float, float, float, float] = (-5.0, -5.0, -5.0, 5.0, 5.0, 5.0)
    """Bounding box of the far-field region (x0, y0, z0, x1, y1, z1)."""

    far_tol: float = 1e-3
    """Tolerance for classifying a surface as far-field."""

    # --- quality thresholds ---
    min_sicn_threshold: float = 0.1
    """Elements with SICN below this are flagged as bad in the quality report."""

    msh_version: float = 4.1
    """Msh file version written by :func:`drone_mesh.services.export_mesh`."""

    msh_binary: bool = False
    """Whether meshes are written in binary Msh format."""

    optimize_method: str | None = "Netgen"
    """Mesh optimizer to run after generation; ``None`` skips optimization."""

    def __post_init__(self) -> None:
        """Validate configuration invariants after construction."""
        if self.refine <= 0:
            raise MeshError(f"refine must be positive, got {self.refine}")
        if self.struct_min_size > self.struct_max_size:
            raise MeshError(
                "struct_min_size must be <= struct_max_size, got "
                f"{self.struct_min_size} > {self.struct_max_size}"
            )
        if self.flow_boundary_thickness <= 0:
            raise MeshError(
                "flow_boundary_thickness must be positive, got "
                f"{self.flow_boundary_thickness}"
            )

    @property
    def scaled_struct_base(self) -> float:
        """Structural base size after applying ``refine``."""
        return self.struct_base_size * self.refine

    @property
    def scaled_flow_base(self) -> float:
        """Flow base size after applying ``refine``."""
        return self.flow_base_size * self.refine

    @classmethod
    def from_args(cls, refine: float, out_dir: str | Path) -> "MeshConfig":
        """Build a config from CLI arguments, applying sensible defaults."""
        return cls(refine=refine, out_dir=out_dir)

    def out_dir_path(self) -> Path:
        """Return ``out_dir`` as a :class:`pathlib.Path`."""
        return Path(self.out_dir)# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------


class MeshPipeline:
    """Orchestrates STEP import, mesh sizing, generation, and export.

    The pipeline is configured via a :class:`MeshConfig` and driven by a Gmsh
    session passed to :meth:`run_structural` / :meth:`run_flow`. Each method is
    a single, testable unit so failures are isolated and easy to reproduce.
    """

    def __init__(self, config: MeshConfig) -> None:
        self.cfg = config
        self.out_dir = config.out_dir_path()
        self.out_dir.mkdir(parents=True, exist_ok=True)

    # -- structural mesh ---------------------------------------------------

    def run_structural(self, gmsh: "gmsh", step_path: str | Path) -> dict:
        """Generate the structural (CalculiX) mesh and write its quality report."""
        step_path = Path(step_path)
        n_volumes = import_step(gmsh, step_path)
        fuse_volumes(gmsh)

        base = self.cfg.scaled_struct_base
        field_ids: list[int] = []

        # Distance field over all surfaces drives the sizing field.
        d = distance_field(gmsh, sampling=50)
        field_ids.append(d)

        t = threshold_field(
            gmsh,
            d,
            h_min=self.cfg.struct_min_size,
            h_max=self.cfg.struct_max_size,
            d_min=0.0,
            d_max=self.cfg.struct_max_size,
        )
        field_ids.append(t)

        compose_background(gmsh, field_ids)
        run_mesh_generation(gmsh, dim=3)

        if self.cfg.optimize_method:
            optimize_mesh(gmsh, self.cfg.optimize_method)

        rep = quality_report(
            gmsh,
            "structural",
            self.out_dir,
            min_sicn_threshold=self.cfg.min_sicn_threshold,
        )
        self._raise_on_quality(rep, "structural")

        export_mesh(
            gmsh,
            self.out_dir / "structural.msh",
            version=self.cfg.msh_version,
            binary=self.cfg.msh_binary,
        )
        return {"mesh": "structural", "volumes": n_volumes, "quality": rep}

    # -- flow mesh ---------------------------------------------------------

    def run_flow(self, gmsh: "gmsh", step_path: str | Path) -> dict:
        """Generate the flow (OpenFOAM) mesh with a boundary-layer refinement box."""
        step_path = Path(step_path)
        n_volumes = import_step(gmsh, step_path)
        fuse_volumes(gmsh)

        faces = classify_boundary_surfaces(
            gmsh,
            self.cfg.inlet_x,
            self.cfg.outlet_x,
            self.cfg.far_bounds,
            self.cfg.far_tol,
        )

        base = self.cfg.scaled_flow_base
        field_ids: list[int] = []

        # Boundary-layer refinement box around the body.
        box = (
            self.cfg.far_bounds[0] + self.cfg.flow_boundary_thickness,
            self.cfg.far_bounds[1] + self.cfg.flow_boundary_thickness,
            self.cfg.far_bounds[2] + self.cfg.flow_boundary_thickness,
            self.cfg.far_bounds[3] - self.cfg.flow_boundary_thickness,
            self.cfg.far_bounds[4] - self.cfg.flow_boundary_thickness,
            self.cfg.far_bounds[5] - self.cfg.flow_boundary_thickness,
        )
        b = box_field(
            gmsh,
            box,
            v_in=base,
            v_out=self.cfg.flow_farfield_size,
            thickness=self.cfg.flow_boundary_thickness,
        )
        field_ids.append(b)

        # Distance field over body surfaces for sizing near geometry.
        d = distance_field(gmsh, surfaces=faces["body"], sampling=50)
        field_ids.append(d)

        t = threshold_field(
            gmsh,
            d,
            h_min=base * 0.5,
            h_max=self.cfg.flow_farfield_size,
            d_min=0.0,
            d_max=self.cfg.flow_boundary_thickness,
        )
        field_ids.append(t)

        compose_background(gmsh, field_ids)
        run_mesh_generation(gmsh, dim=3)

        if self.cfg.optimize_method:
            optimize_mesh(gmsh, self.cfg.optimize_method)

        rep = quality_report(
            gmsh,
            "flow",
            self.out_dir,
            min_sicn_threshold=self.cfg.min_sicn_threshold,
        )
        self._raise_on_quality(rep, "flow")

        export_mesh(
            gmsh,
            self.out_dir / "flow.msh",
            version=self.cfg.msh_version,
            binary=self.cfg.msh_binary,
        )
        return {"mesh": "flow", "volumes": n_volumes, "faces": faces, "quality": rep}

    # -- helpers -----------------------------------------------------------

    @staticmethod
    def _raise_on_quality(rep: dict, name: str) -> None:
        """Raise :class:`MeshQualityError` if the report flags bad elements."""
        bad = int(rep.get("bad_elements_minSICN_lt_0.1", 0))
        if bad:
            raise MeshQualityError(
                f"{name} mesh produced {bad} element(s) below the SICN threshold.",
                violations=rep,
            )