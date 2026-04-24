# Real-Arm Control

Main entrypoint: [`run.py`](/Users/gdmen/MIT/sp26/2.12/212_UR5/run.py)

Useful commands:

```bash
python3 run.py list-arms
python3 run.py list-trials
python3 run.py state --arms ur_right
python3 run.py trial --trial right_task_frame_probe
python3 run.py trial --trial right_task_frame_probe --live --arms ur_right
python3 run.py manual --arms ur_right --arm ur_right --input-source auto
```

Where to edit things:

- Available live arms, IPs, calibration, TCP readiness, gripper type:
  [`control_scripts/runtime.py`](/Users/gdmen/MIT/sp26/2.12/212_UR5/control_scripts/runtime.py)
- Hard-coded autonomous waypoint paths and keep-out boxes:
  [`control_scripts/trials/definitions.py`](/Users/gdmen/MIT/sp26/2.12/212_UR5/control_scripts/trials/definitions.py)
- Manual teleop loop, joystick mapping, CSV logging:
  [`control_scripts/manual.py`](/Users/gdmen/MIT/sp26/2.12/212_UR5/control_scripts/manual.py)

Notes:

- `state` can connect any defined arm even if its TCP offset is still unset.
- `trial` and `manual` require the controlled arm to be motion-ready.
- Manual mode logs each cycle to `logs/manual/*.csv` by default.
- The default teleop safety box comes from `--safety-trial`, so manual probing can reuse the same workspace and keep-out geometry as the autonomous trial.
