"""End-to-end (and bimanual) competition routines.

A routine composes one or more tasks and primitives into a complete
sequence — for example, "open the microwave with the left arm, then
pick a plate with the right arm and place it inside, then close the
door." Routines own bimanual sequencing logic; tasks stay single-arm.

Each routine module exposes:

    main(dry: bool = False) -> int

so it can be invoked from the unified CLI:

    python -m control_scripts.run routine <name> [--dry]

Routines own the ``Session`` (typically ``default_session(left=True,
right=True)``) and call into:

  - per-arm primitives in ``tasks/`` (e.g. ``microwave_place.place_in_microwave``)
  - end-to-end task ``main()`` functions when those happen to do exactly
    what one phase of the routine needs (rare; usually you call
    ``run_on_arm`` or the underlying primitive directly so the routine
    can pick its own poses / order).

Parallel execution across the two arms isn't wired in yet — start with
sequential calls and add concurrency only when something actually needs it.
"""

# Maps CLI routine name → main(dry: bool = False) -> int.
# Add new routines here as they're written.
ROUTINES: dict = {}
