"""Shared workspace landmarks: named task-frame locations that multiple
tasks must agree on.

Scope
-----
This file owns task-frame xy/xyz locations that are referenced by 2+
tasks. The motivating example is the *pour station*: the cup gets
placed there by ``pick_place_cup``, the bottle pours into it from
``pour_bottle_hook``, and a future cup-pickup task reads it too. All
three need to agree on a single coordinate; if it lives in any one
task, the others drift.

Out of scope
------------
- Object rest poses → ``planning/scene/objects.py:*_DEFAULT_TASK_XYZ``
- Tray pose / on-tray slots → ``util/tray_layout.py``
- Microwave geometry → ``microwave.py``
- Robot/TCP calibration → ``calibration.py``
- Single-task literals (carry midpoints, hover offsets) — keep those
  in the task file. Only promote here when a second task starts
  reading the same value.
"""

from __future__ import annotations

import numpy as np


CUP_POUR_STATION_XY_TASK = np.array([-0.1, 0.15])
"""Task-frame xy where the cup sits to receive a pour. Shared by:
  - ``tasks/pick_place_cup.py``       (places the cup here)
  - ``tasks/pour_bottle_hook.py``     (aims the pour here)
  - (future) cup pickup task          (picks the cup from here)
"""

BOTTLE_MICROWAVE_TOP_XYZ_TASK = np.array([-0.30, 0.42, 0.28])
"""Task-frame xyz where the bottle is stashed on top of the microwave
after pouring. Z=0.28 is the microwave outer top. Shared by:
  - ``tasks/pour_bottle_hook.py``     (places the bottle here)
  - (future) bottle pickup task       (picks the bottle from here)

Layout note: front-left of the microwave top so the LEFT arm reaches
without overextending. Bottle xy footprint
``[-0.34, -0.26] × [0.38, 0.46]`` (radius ~4 cm) sits fully inside the
microwave outer-top footprint with ~9 cm side / ~6 cm front margin."""

CUP_MICROWAVE_TOP_XYZ_TASK = np.array([-0.014, 0.449, 0.280])
"""Task-frame xyz where the cup-with-stick is placed on top of the
microwave. Z=0.280 is the microwave outer top. Shared by:
  - ``planning/visualize_cup_microwave_place.py``  (preview rendering)
  - (future) cup-with-stick pick/place tasks

Layout note: 3 cm wall-clearance reading — cup outer wall sits 3 cm
clear of each microwave edge (no overhang). Centred-on-corner variant
``(0.030, 0.405, 0.280)`` would overhang ~1.4 cm; not used."""
