"""ArmHandle: bundles RTDE handles + per-arm calibration for one UR arm.

Constructed once at program start and reused. Carries:
    - the two RTDE interfaces (control + receive)
    - the attached gripper driver (optional — not every call path needs one)
    - X_base_task — pose of the shared task frame in this arm's base frame
      (see calibration.py and util/frames.py)
    - tcp_offset — 6-vector for rtde_c.setTcp, must be applied once per
      connection via setup()

Primitives and moves always speak poses in the task frame. Conversion to
base frame happens inside ``to_base()``, just before the RTDE call.
"""

from dataclasses import dataclass, field
from typing import Any, List, Optional

from .grippers.base import Gripper
from .util.frames import base_to_task, task_to_base, velocity_task_to_base
from .util.poses import Pose


@dataclass
class ArmHandle:
    name: str
    control: Any          # rtde_control.RTDEControlInterface
    receive: Any          # rtde_receive.RTDEReceiveInterface
    gripper: Optional[Gripper] = None

    X_base_task: Pose = field(default_factory=Pose)
    """Calibration: pose of the task frame expressed in this arm's base frame.
    Default is identity (equivalent to 'task frame == base frame'), which is
    fine for single-arm bring-up but must be set from calibration.py before
    running any task-frame code on the real rig."""

    tcp_offset: Optional[List[float]] = None
    """6-vector [x, y, z, rx, ry, rz]. Applied by ``setup()`` — leaving this
    None means the controller uses whatever TCP was last set on the pendant
    or by a previous script. Always set it explicitly for reproducibility."""

    def setup(self) -> None:
        """Apply TCP offset. Call once after connection is confirmed."""
        if self.tcp_offset is None:
            raise ValueError(
                f"{self.name}: tcp_offset is not set — pick a value from "
                f"calibration.py (e.g. TCP_OFFSET_ROBOTIQ_2F85) before setup()."
            )
        self.control.setTcp(list(self.tcp_offset))

    # --- Frame conversion helpers ---
    def to_base(self, pose_task: Pose) -> Pose:
        """Convert a task-frame pose to this arm's base frame."""
        return task_to_base(self.X_base_task, pose_task)

    def to_task(self, pose_base: Pose) -> Pose:
        """Convert a base-frame pose to task frame."""
        return base_to_task(self.X_base_task, pose_base)

    def task_velocity_to_base(self, v_task) -> List[float]:
        """Rotate a 6-DOF Cartesian twist from task to base frame."""
        return velocity_task_to_base(self.X_base_task, v_task)
