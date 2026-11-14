"""drone_mesh: Deterministic Gmsh meshing pipeline for multi-physics drone simulation.

Rebuilds structural (CalculiX) and flow (OpenFOAM) meshes from a STEP CAD file.
All mesh sizes are parameterized so the same script drives mesh convergence
studies by scaling a single ``--refine`` factor.
"""

from __future__ import annotations

from drone_mesh.core import MeshConfig, MeshPipeline
from drone_mesh.exceptions import MeshError, MeshQualityError, StepImportError

__version__ = "1.0.0"
__all__ = ["MeshConfig", "MeshPipeline", "MeshError", "MeshQualityError", "StepImportError"]
