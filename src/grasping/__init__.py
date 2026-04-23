"""Grasp candidates + (future) grasp generation from perception.

Today this package is just a hand-edited table of per-object grasp poses.
The interface is designed so a live grasp generator (primitive fitting,
learned detector) can be dropped in behind ``get_grasp_candidates`` later
without any caller changes.
"""

from .candidates import get_grasp_candidates

__all__ = ["get_grasp_candidates"]
