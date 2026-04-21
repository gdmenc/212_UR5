"""Warm-start cache for trajectory optimization.

Trajopt benefits significantly from a good initial guess — a B-spline that
was feasible for a similar (q_start, q_goal, scene) pair is usually close
enough to feasible in the new problem that the NLP converges in a handful
of iterations instead of hundreds.

Cache key = (rounded q_start, rounded q_goal, scene_hash). Rounding groups
near-duplicate queries together; scene_hash keys off the poses of movable
objects so a solution computed for one plate position isn't reused after
the plate has moved.

Kept fully in-memory and small. Skip entirely unless solve time becomes a
bottleneck — class-project scale probably doesn't need this, but the slot
is here so we don't have to refactor later.
"""

from collections import OrderedDict
from typing import Optional, Tuple

import numpy as np


_MAX_ENTRIES = 256
_CACHE: "OrderedDict[Tuple, np.ndarray]" = OrderedDict()


def _make_key(
    q_start: np.ndarray, q_goal: np.ndarray, scene_hash: str
) -> Tuple:
    return (
        tuple(np.round(q_start, 3)),
        tuple(np.round(q_goal, 3)),
        scene_hash,
    )


def lookup(
    q_start: np.ndarray, q_goal: np.ndarray, scene_hash: str
) -> Optional[np.ndarray]:
    """Return cached control points for a matching problem, or None."""
    key = _make_key(q_start, q_goal, scene_hash)
    if key in _CACHE:
        _CACHE.move_to_end(key)  # LRU: mark as recently used
        return _CACHE[key]
    return None


def store(
    q_start: np.ndarray,
    q_goal: np.ndarray,
    scene_hash: str,
    control_points: np.ndarray,
) -> None:
    """Cache a successful solution. Evicts the oldest entry when full."""
    key = _make_key(q_start, q_goal, scene_hash)
    _CACHE[key] = control_points
    _CACHE.move_to_end(key)
    if len(_CACHE) > _MAX_ENTRIES:
        _CACHE.popitem(last=False)


def clear() -> None:
    """Wipe the cache. Useful across unrelated tasks or during debugging."""
    _CACHE.clear()
