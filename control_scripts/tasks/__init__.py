"""Task-specific primitives.

Two flavours live here:

  - **Workspace-fixture primitives** (``microwave_place``, ``open_microwave``)
    encode the geometry of a single physical fixture and expose one or
    more functions that take an ``ArmHandle`` plus a fixture spec and
    drive the motion. They take per-call parameters that don't have
    sensible defaults, so they're meant to be CALLED FROM ROUTINES, not
    invoked from the CLI.

  - **End-to-end single-arm tasks** (``pick_place_plate``) wrap a
    self-contained operation behind ``main(dry=False) -> int`` so they
    can run standalone OR be dispatched from ``run.py task <name>``.
    Module-level constants pin the workspace coordinates; edit the
    constants, don't pass them in.

The ``TASKS`` registry below holds only the second flavour — entries
that are invocable from the unified CLI. Add new ones as you write them.

Parameters that need physical measurement are marked TODO in each file
and will raise a clear error until set.
"""

from . import pick_place_plate

# Maps CLI task name → main(dry: bool = False) -> int.
TASKS = {
    "pick_place_plate": pick_place_plate.main,
}
