"""Per-object grasp candidates.

Each object has a small module here (``plate.py``, ``bowl.py``, ...) that
exports factory functions returning ``Grasp`` instances **in the task
frame**. The factories consume the object pose in task frame and any
object-specific parameters (e.g. which rim angle to grasp at).

Why factories, not static tables: most of our target objects have
rotational symmetry (plate, bowl, cup body), so the right grasp depends
on the current object pose and on which candidate angle reaches best
from the current TCP. A function captures that.

Primitives should NOT import from individual grasp files; they take a
``Grasp`` and execute it. Callers (task sequencer) build the Grasp.
"""

from .base import Grasp

__all__ = ["Grasp"]
