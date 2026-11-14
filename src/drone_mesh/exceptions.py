"""Exception hierarchy for the drone mesh pipeline.

All failures raised by the pipeline inherit from :class:`MeshError` so callers
can catch pipeline-wide failures without importing private modules.
"""

from __future__ import annotations


class MeshError(Exception):
    """Base class for every failure raised by the drone_mesh pipeline."""


class StepImportError(MeshError):
    """Raised when the STEP file cannot be imported or contains no solids."""


class MeshGenerationError(MeshError):
    """Raised when Gmsh fails to produce a mesh (e.g. geometry is invalid)."""


class MeshQualityError(MeshError):
    """Raised when the produced mesh violates a configured quality threshold."""

    def __init__(self, message: str, *, violations: dict | None = None) -> None:
        super().__init__(message)
        self.violations = violations or {}


class FieldError(MeshError):
    """Raised when a mesh size field cannot be constructed or applied."""
