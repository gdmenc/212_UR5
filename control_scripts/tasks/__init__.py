"""Task-specific primitives.

Unlike ``control_scripts/pick.py`` and ``place.py`` (which are generic —
they work for any object with a ``Grasp``), modules here target ONE
specific workspace fixture and encode its geometry: the microwave
cavity, the tray handles, etc.

Each module exposes:
  - a dataclass describing the fixture's measurable properties
    (``MicrowaveSpec``, ``TraySpec``, ...)
  - one or more primitive functions that take the fixture spec and an
    ArmHandle and drive the motion.

Parameters that need physical measurement are marked TODO in each file
and will raise a clear error until set.
"""
