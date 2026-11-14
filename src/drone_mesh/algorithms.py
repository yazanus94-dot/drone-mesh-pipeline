"""Mesh-size field construction algorithms.

Each helper wraps a Gmsh mesh field and returns the field tag so the caller can
compose fields into a background-field tree. Keeping these as pure functions
makes the field logic unit-testable without a running Gmsh session.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - typing only
    import gmsh  # type: ignore


def threshold_field(
    gmsh: "gmsh",
    in_field: int,
    *,
    h_min: float,
    h_max: float,
    d_min: float,
    d_max: float,
) -> int:
    """Create a Gmsh ``Threshold`` field that resizes elements between two distances."""
    f = gmsh.model.mesh.field
    t = f.add("Threshold")
    f.setNumber(t, "InField", in_field)
    f.setNumber(t, "SizeMin", h_min)
    f.setNumber(t, "SizeMax", h_max)
    f.setNumber(t, "DistMin", d_min)
    f.setNumber(t, "DistMax", d_max)
    return t


def distance_field(
    gmsh: "gmsh",
    *,
    curves: list[int] | None = None,
    surfaces: list[int] | None = None,
    sampling: int = 50,
) -> int:
    """Create a Gmsh ``Distance`` field over the given curves or surfaces."""
    f = gmsh.model.mesh.field
    d = f.add("Distance")
    if curves:
        f.setNumbers(d, "CurvesList", curves)
    if surfaces:
        f.setNumbers(d, "SurfacesList", surfaces)
    f.setNumber(d, "Sampling", sampling)
    return d


def box_field(
    gmsh: "gmsh",
    box: tuple[float, float, float, float, float, float],
    *,
    v_in: float,
    v_out: float,
    thickness: float,
) -> int:
    """Create a Gmsh ``Box`` field for localized refinement."""
    f = gmsh.model.mesh.field
    b = f.add("Box")
    f.setNumber(b, "VIn", v_in)
    f.setNumber(b, "VOut", v_out)
    for key, value in zip(
        ("XMin", "YMin", "ZMin", "XMax", "YMax", "ZMax"), box
    ):
        f.setNumber(b, key, value)
    f.setNumber(b, "Thickness", thickness)
    return b


def compose_background(gmsh: "gmsh", field_ids: list[int]) -> int:
    """Combine multiple fields with a ``Min`` background field."""
    f = gmsh.model.mesh.field
    m = f.add("Min")
    f.setNumbers(m, "FieldsList", field_ids)
    f.setAsBackgroundField(m)
    return m