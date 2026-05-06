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
# (table + vention + microwave + grippers, with the URDF welded at the
# UR controller's ``base`` frame — see scene/arms.py): forearms parallel
# to the operator-edge of the table, wrists tucked away from the
# operator (task +y), TCPs at task xyz ≈ (±0.93, *, ~0.4) — beyond the
# ±0.45 m table x-edges and well above the floor. Mirror-symmetric in x.
#
# Previous values [+70, ...] / [+130, ...] were tuned against the old
# (incorrect) base_link weld and put the right TCP below the floor
# (z ≈ -0.014 m) once the welding was fixed. These new shoulder_pans
# are the 180°-shifted equivalents under the corrected base orientation.
SIM_HOME_Q_LEFT = np.array([
    0.8135381937026978,
    -1.4808495801738282,
    1.7939346472369593,
    -1.3749521386674424,
    -1.0451715628253382,
    -2.5229952971087855 + 2.0 * np.pi,
])
SIM_HOME_Q_RIGHT = np.array([
    -0.8470800558673304,
    -1.5827747784056605,
    -1.7470908164978027,
    -1.9068347416319789,
    1.00191330909729,
    # Wrapped from the rig-recorded +5.4352 rad (+311.4°, ~49° below the
    # +2π upper limit) to its kinematically-equivalent value on the other
    # side of zero. Same physical pose; ~5.4 rad / 309° headroom in both
    # directions. Planning-only — does NOT command the rig.
    5.435201168060303 - 2.0 * np.pi,
])

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
