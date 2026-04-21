"""Planner protocol.

All trajectory planners in this package share a single signature:

    plan(plant, q_start, q_goal, duration=None) -> BsplineTrajectory | None

A ``BsplineTrajectory`` is a Drake representation of a smooth joint-space
path parameterized by time. Returning ``None`` means the planner failed to
find a feasible path (collision, joint limit, timeout). Callers are expected
to fall back to a slower-but-more-complete planner when that happens — e.g.
``plan_trajopt`` -> ``plan_rrt_connect``.

The protocol is intentionally narrow: take a plant for collision / kinematics
queries, take start and goal joint configs, produce a time-parameterized
path. Anything fancier (Cartesian constraints, via-points, multi-arm
coordination) is the responsibility of the caller to bake into ``q_goal`` or
into a higher-level wrapper.
"""

from typing import Optional, Protocol

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.trajectories import BsplineTrajectory


class Planner(Protocol):
    def plan(
        self,
        plant: MultibodyPlant,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        duration: Optional[float] = None,
    ) -> Optional[BsplineTrajectory]:
        ...
