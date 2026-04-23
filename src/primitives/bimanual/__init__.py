"""Bimanual primitives. See README.md for scope and conventions.

Same ``PrimitiveResult`` return type as single-arm primitives, but the
signature takes two arm names (``left_arm``, ``right_arm``) and
coordinates them internally.
"""

from .carry_tray import carry_tray
from .pour_stabilized import pour_stabilized

__all__ = ["carry_tray", "pour_stabilized"]
