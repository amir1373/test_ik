# Test IK

Forward and inverse kinematics experiments for a robot arm, written mainly in C++ with a supporting Python script.

## What This Repository Contains

- `fk.cpp`, `fk1.cpp` - forward kinematics experiments.
- `ik.cpp`, `ik1.cpp`, `ik_last1.cpp`, `iktest.cpp` - inverse kinematics experiments and test variants.
- `rotmat2rpy.cpp` - rotation-matrix to roll/pitch/yaw conversion utility.
- `FK_symbolic.py` - symbolic forward-kinematics helper script.
- `Eigen/` - Eigen math library files used by the C++ examples.
- `sample robot1/` - sample robot-related data or reference files.

## Build

The C++ files are standalone experiments. Build the file you want to test with a C++ compiler and include the Eigen headers if needed:

```bash
g++ ik.cpp -I Eigen -o ik
```

Adjust the include path and source file depending on the experiment being built.

## Notes

This repository is an experimental kinematics workspace. Some generated Windows `.exe` files are currently committed; for a cleaner source repository, move generated binaries to releases or rebuild them locally from source.