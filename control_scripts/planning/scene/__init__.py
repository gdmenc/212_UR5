"""Scene fragments. Each module adds bodies/joints to a MultibodyPlant.

Order matters: ``vention`` first (provides the top-plate frame the arms
weld onto), then ``arms``, then static fixtures (``tables``,
``microwave``). Movable objects (cup, bowl, ...) come last so they
appear on top of the table in Meshcat.
"""

from __future__ import annotations
