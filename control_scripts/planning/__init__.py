"""Drake-side motion planning for the 212 bimanual rig.

Mirrors the structure of ``control_scripts/tasks/`` (the RTDE-side
execution) but operates on a ``MultibodyPlant``: kinematic queries,
collision checks, trajectory optimization.

Anchoring
---------
**Drake's world frame is the task frame** used everywhere else in this
package. That makes constants from ``control_scripts/calibration.py``
(``X_LEFT_BASE_TASK``, ``HINGE_POSITION_TASK``, ``HOME_Q_RAD_LEFT``)
drop straight into Drake without any frame conversion.

  - origin: top-centre of the heavy-duty table
  - +x:     toward the right arm
  - +y:     toward the microwave
  - +z:     up

Layout
------
::

    planning/
    ├── scene/
    │   ├── vention.py     — programmatic stand from boxes (one per extrusion)
    │   ├── arms.py        — load UR5e URDFs, weld at calibration poses
    │   ├── tables.py      — heavy-duty workspace tables
    │   └── microwave.py   — microwave housing + hinged door
    ├── build_scene.py     — compose the above into one MultibodyPlant
    └── visualize.py       — CLI: ``python -m control_scripts.planning.visualize``
"""

from __future__ import annotations

from typing import Dict, Optional

import numpy as np
from pydrake.multibody.plant import MultibodyPlant

# Per-arm SIMULATION HOME joint configurations.
#
# Deliberately decoupled from ``calibration.HOME_Q_RAD_*`` (which drive
# the real arm via ``Session.move_to_home``) so changing the sim default
# can never command an unintended motion on the physical UR. The
# real-arm HOME stays whatever the team has lab-verified; this is just
# what ``plant.CreateDefaultContext()`` lands at, and what tasks pass
# in via ``plan_transit(current_q=...)`` when running offline.
#
# Pose chosen by FK+collision sweep against the planning scene
# (table + vention + microwave + grippers): forearms parallel to the
# operator-edge of the table, wrists tucked away from the operator
# (task +y), TCPs beyond the ±0.45 m table x-edges and ~27 cm lower
# than the previous left HOME. Both arms now mirror in y too — left
# uses shoulder_pan = +70° (NOT just -sign of right's +130°) because
# the asymmetric base R matrices mean naive sign-flip doesn't mirror.
SIM_HOME_Q_LEFT = np.radians([+70.0, -90.0, -90.0, -90.0, -90.0, 0.0])
SIM_HOME_Q_RIGHT = np.radians([130.0, -90.0, -90.0, -90.0, -90.0, 0.0])

HOME_Q: Dict[str, Optional[np.ndarray]] = {
    "ur_left": SIM_HOME_Q_LEFT,
    "ur_right": SIM_HOME_Q_RIGHT,
}


def default_home_q(plant: MultibodyPlant) -> np.ndarray:
    """Full-plant position vector with each arm at its sim HOME pose.

    Drives ``plant.CreateDefaultContext()`` and is also the natural
    seed for offline ``plan_transit(current_q=...)`` calls. Decoupled
    from the lab-verified real-arm HOME in ``calibration.py`` — see
    ``HOME_Q`` above for the rationale.
    """
    # Local import to avoid a circular dep at package-load time.
    from .transit import _arm_model_instance, _arm_position_indices

    q = np.zeros(plant.num_positions(), dtype=float)
    for arm_name, q_home in HOME_Q.items():
        if q_home is None:
            continue
        try:
            inst = _arm_model_instance(plant, arm_name)
        except KeyError:
            continue
        idx = _arm_position_indices(plant, inst)
        if q_home.shape != (len(idx),):
            raise ValueError(
                f"HOME_Q[{arm_name!r}] has shape {q_home.shape}, "
                f"expected ({len(idx)},)."
            )
        q[idx] = q_home
    return q
