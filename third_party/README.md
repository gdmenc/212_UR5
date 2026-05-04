# third_party/

Vendored external code we depend on directly. Each subdirectory is a
snapshot of upstream source — not a git submodule — so a fresh clone
of this repo doesn't need any extra `git submodule init` step.

## ikfastpy/

Closed-form analytic IK for the UR5e arm. Used by
`control_scripts/planning/ikfast.py` and the planning stack to
enumerate all 8 IK branches at any TCP pose in microseconds (vs.
SNOPT's 0.4 ms on success / 60 ms on failure).

### Provenance

- Build harness (Cython wrapper, header, setup.py): vendored from
  [`andyzeng/ikfastpy`](https://github.com/andyzeng/ikfastpy) (MIT).
- Compiled-in kinematics (`ikfast61.cpp`): the **UR5e** variant from
  [`cambel/ur_ikfast`](https://github.com/cambel/ur_ikfast)
  (`ur5e/ur5e_ikfast61.cpp`, Apache 2.0). The original `andyzeng`
  cpp was UR5; we swapped it for the UR5e version because UR5
  link lengths produce ~80 mm position error against our UR5e plant.
  Both license headers are preserved in the source files.

The single edit to `cambel`'s file was changing
`#include "../include/ikfast.h"` to `#include "ikfast.h"` so the
header resolves locally.

### Build (one-time, after clone)

The `.so` is platform-specific (gitignored at repo root); rebuild it
once after cloning the repo:

```sh
cd third_party/ikfastpy
python3.11 setup.py build_ext --inplace
```

Requires `Cython`, a C++ compiler, and LAPACK (Apple Silicon: comes
via the Accelerate framework — no install needed; Linux: `apt install
liblapack-dev`).

Verify with:

```sh
python3.11 -m control_scripts.planning.verify_ikfast --n-random 200
```

Expected results: ~98% FK→IK round-trip rate, ~1 mm position error and
~1.4° rotation error against Drake URDF FK (the constants in
`cambel`'s upstream URDF are rounded to 3 decimals — the residual
rotation is that rounding, not a kinematics bug).

### What's gitignored

The repo's root `.gitignore` skips:
- `build/` (CMake/distutils output)
- `*.so` (compiled extension, platform-specific)
- `ikfastpy.cpp` (Cython-generated C++ wrapper, regenerated from `.pyx`)
- `ikfast61_ur5.cpp.bak` (local backup of the original UR5 cpp,
  kept around in case we want to switch back; not worth committing)

Everything else in `third_party/ikfastpy/` is committed source.
