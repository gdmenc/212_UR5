# Camera Calibration (C++)

Hand-eye calibration for a wrist-mounted Intel RealSense camera (eye-in-hand).

Solves **T_ee_camera** — the fixed rigid transform from the camera frame to the robot's TCP (end-effector) frame.

## Dependencies

Installed via Homebrew:

- **librealsense** — Intel RealSense SDK (`brew install librealsense`)
- **OpenCV** — with aruco contrib module (`brew install opencv`)
- **CMake** ≥ 3.16

## Build

```bash
cd calibration
mkdir -p build && cd build
cmake ..
make -j$(sysctl -n hw.ncpu)
```

## Usage

### 1. Generate the calibration board

```bash
./calibrate_camera --print-board
```

Produces `charuco_board.png`. Print at **100% scale** (no "fit to page"). Verify that each square measures exactly **25 mm** on the printout. Tape the board flat to a rigid surface.

### 2. Run calibration

```bash
# Manual EE pose entry (no robot C++ SDK needed)
./calibrate_camera --no-robot

# With robot IP (requires ur_rtde C++ SDK — not yet integrated)
./calibrate_camera --arm left --ip 192.168.1.101
```

### 3. Interactive capture

- **SPACE** — capture a sample (board must be visible in the live view)
- **Q** — finish capturing and run calibration
- **ESC** — abort without calibrating

For each capture, you'll be prompted to enter the TCP pose `x y z rx ry rz` (metres, axis-angle) from the teach pendant or a companion script.

Collect **≥ 12 samples** with diverse poses (vary tilt, pan, and standoff distance). The diversity score in the overlay should stay above 0.5.

### 4. Output

The result is saved to `control_scripts/T_ee_camera.json` (or the path given via `--output`). The JSON format is compatible with the Python `load_T_ee_camera()` function.

## Options

| Flag | Default | Description |
|---|---|---|
| `--print-board` | — | Save `charuco_board.png` and exit |
| `--arm NAME` | `left` | Arm name (metadata only) |
| `--ip IP` | `192.168.1.101` | Robot IP (for future ur_rtde integration) |
| `--no-robot` | off | Skip robot connection; enter EE poses manually |
| `--min-samples N` | `12` | Minimum captures before Q is accepted |
| `--output PATH` | `../control_scripts/T_ee_camera.json` | Output JSON path |
