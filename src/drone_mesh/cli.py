"""Command-line interface for the drone mesh pipeline.

Usage:
    python -m drone_mesh.cli --step cad/drone.step --mode both [--refine 1.0] [--gui]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from drone_mesh.core import MeshConfig, MeshPipeline


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="drone-mesh",
        description="Deterministic Gmsh meshing pipeline for drone multi-physics simulation.",
    )
    ap.add_argument("--step", default="cad/drone.step", help="Path to the STEP CAD file.")
    ap.add_argument(
        "--mode",
        choices=["structural", "flow", "both"],
        default="both",
        help="Which meshes to generate.",
    )
    ap.add_argument(
        "--refine",
        type=float,
        default=1.0,
        help="Scale all mesh sizes (0.5 = finer, 2.0 = coarser).",
    )
    ap.add_argument("--gui", action="store_true", help="Open the Gmsh GUI after meshing.")
    ap.add_argument(
        "--out-dir",
        default="mesh",
        help="Directory for output meshes and quality reports.",
    )
    return ap


def main(argv: list[str] | None = None) -> int:
    ap = build_parser()
    a = ap.parse_args(argv)

    step_path = Path(a.step)
    if not step_path.exists():
        ap.error(f"STEP file not found: {step_path}")

    cfg = MeshConfig.from_args(a.refine, a.out_dir)
    pipeline = MeshPipeline(cfg)

    import gmsh  # local import: keeps CLI importable without gmsh installed

    gmsh.initialize()
    try:
        if a.mode in ("structural", "both"):
            pipeline.run_structural(gmsh, step_path)
        if a.mode in ("flow", "both"):
            pipeline.run_flow(gmsh, step_path)
    finally:
        gmsh.finalize()

    print(f"\nDone. Meshes written to {a.out_dir}/")
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())