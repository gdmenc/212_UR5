"""Cup grasps (the receiving cup — what gets poured into).

Object-frame convention (matches plate / bowl): origin at the cup's resting
point on the table, +z up out of the opening, rotational symmetry about +z.

Cup shape (measured at the 212 lab):
    - Top (rim) radius:    4.4 cm   (rim diameter 8.8 cm)
    - Base radius:         3.35 cm  (base diameter 6.7 cm)
    - Total height:        15.4 cm
    - Profile: gentle taper — flares OUTWARD from base to rim, opposite
      direction from the bowl. Outward inset of the outer wall ≈
      (4.4 − 3.35) / 15.4 ≈ 0.068 cm per cm of height. Effectively a
      slight truncated cone; for collision purposes a cylinder of radius
      4.4 cm gives a conservative bound.

Grasp strategy: TBD. Two natural options once decided:
    1. Top-down rim pinch with the 2F-85 (parallel to bowl_rim_grasp).
       Rim diameter 8.8 cm is right at the edge of the 2F-85's working
       spread; verify before committing.
    2. Side body grasp — pinch across the body at some height. Forces the
       hand to come in horizontally rather than top-down, which may
       conflict with whatever's above the cup at pour time.
"""

from __future__ import annotations


CUP_RIM_OUTER_RADIUS_M = 0.044
"""Radial distance from cup center to the outer edge of the rim. Measured
at the lab — top diameter 8.8 cm."""

CUP_BASE_RADIUS_M = 0.0335
"""Radial distance from cup center to the outer edge of the base. Measured
6.7 cm diameter. Cup flares outward from base to rim."""

CUP_HEIGHT_M = 0.154
"""Total cup height (resting surface to top of rim). When the cup sits on
the table, this is also the rim's task-z."""
