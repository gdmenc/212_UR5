---
name: Joint mapping RTDE to Drake
description: RTDE getActualQ() 6-vector maps directly to Drake plant positions for each arm
type: reference
---

Confirmed in `planning/scene/arms.py` line 22: "The arm's six joint angles (SetPositions) line up directly with what RTDE reports as getActualQ."

Arms are welded at the `base` frame (UR controller convention), NOT `base_link` (REP-103). This was a past bug — welding `base_link` put shoulder_pan off by 180°.

Joint order: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3] — same in RTDE and Drake.

In the 12-DOF bimanual plant, ur_left occupies indices 0:6 and ur_right occupies indices 6:12 (from add_both_arms call order in build_scene.py).
