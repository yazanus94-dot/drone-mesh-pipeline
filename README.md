# drone-mesh-pipeline

[![Python](https://img.shields.io/badge/python-3.11+-blue.svg)](https://www.python.org/)
[![Gmsh](https://img.shields.io/badge/gmsh-4.11-green.svg)](https://gmsh.info/)
[![License](https://img.shields.io/badge/license-MIT-lightgrey.svg)](./LICENSE)
[![Tests](https://img.shields.io/badge/tests-pending-yellow.svg)](./tests)

Deterministic Gmsh meshing pipeline for multi-physics drone simulation. Rebuilds
structural (CalculiX) and flow (OpenFOAM) meshes from a single STEP CAD file. All
mesh sizes are parameterized so the same script drives mesh convergence studies by
scaling a single `--refine` factor.

## Features

- **STEP import & volume fusion** — imports a STEP file, fuses multiple volumes into
  a single solid, and validates that at least one volume exists.
- **Boundary classification** — automatically tags surfaces as `inlet`, `outlet`,
  `farfield`, or `body` using bounding-box heuristics, ready for CFD boundary
  conditions.
- **Parameterized mesh sizing** — composable Gmsh mesh-size fields
  (`Threshold`, `Distance`, `Box`) combined into a `Min` background field.
- **Deterministic output** — the same STEP file and `--refine` factor always
  produce the same mesh, enabling reproducible convergence studies.
- **Quality reporting** — emits a JSON quality report per mesh (min/mean/p1 of
  `minSICN`, `gamma`, `minSJ`) and flags elements below a configurable threshold.
- **Multi-physics export** — writes structural and flow meshes to disk in the
  configured Msh file version.

## Installation

```bash
pip install -e .
```

Gmsh with Python bindings is required at runtime:

```bash
# Debian/Ubuntu
sudo apt install gmsh

# macOS
brew install gmsh

# or via pip (includes the Python API)
pip install gmsh
```

> **Note:** `gmsh` is imported lazily inside the CLI so the package itself can be
> imported without Gmsh installed.

## Usage

```bash
python -m drone_mesh.cli \
  --step cad/drone.step \
  --mode both \
  --refine 1.0 \
  --out-dir mesh
```

### Arguments

| Argument       | Choices          | Default            | Description                                            |
| -------------- | ---------------- | ------------------ | ------------------------------------------------------ |
| `--step`       | —                | `cad/drone.step`   | Path to the STEP CAD file.                             |
| `--mode`       | `structural`     | `both`             | Which meshes to generate.                              |
|                | `flow`           |                    |                                                        |
|                | `both`           |                    |                                                        |
| `--refine`     | —                | `1.0`              | Scale all mesh sizes (`0.5` = finer, `2.0` = coarser). |
| `--gui`        | —                | (flag)             | Open the Gmsh GUI after meshing.                       |
| `--out-dir`    | —                | `mesh`             | Directory for output meshes and quality reports.      |

## Package layout

```
drone-mesh-pipeline/
├── src/drone_mesh/
│   ├── __init__.py     # Public API: MeshConfig, MeshPipeline, exceptions
│   ├── algorithms.py   # Mesh-size field construction (Threshold/Distance/Box)
│   ├── cli.py          # Command-line interface
│   ├── core.py         # MeshConfig + MeshPipeline orchestration
│   ├── exceptions.py   # Exception hierarchy (MeshError and subclasses)
│   └── services.py     # STEP import, boundary classification, quality, export
├── tests/              # Unit, integration, and property-based tests
├── benchmarks/         # Mesh convergence benchmarks
├── cad/                # STEP source geometry
├── docs/               # Documentation
├── scripts/            # Helper scripts
└── mesh/               # Generated mesh outputs (git-ignored)
```

## Testing

```bash
pip install -r requirements.txt
pytest -q
```

## License

MIT — see [LICENSE](./LICENSE).