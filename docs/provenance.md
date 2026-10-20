# Source and environment provenance

## Coursework basis

The Pure Pursuit geometric idea and path-distance interpolation were informed by
the team's previous MP2 controller at:

`/home/xinweiyu/Workspace/ECE484_Program/fa26-mp2-release/code/release/src/mp2/src/controller.py`

- Local MP2 repository HEAD: `f2af4b3bd0892e7868609b2c62fff41e6000fa48`.
- Inspected controller SHA-256:
  `f518d007199ac92ec3257178a6f6fc3001f7b19a517704f70ef47b72b4e625d3`.
- That HEAD alone does not identify the locally edited assignment solution;
  the file hash records the exact inspected reference.
- The new core is a dependency-free, local-path implementation with conservative
  simulator defaults. The old controller's 5–10 m/s schedule, global waypoint
  traversal, ROS publishers and coursework datasets are not imported.
- No original assignment or historical experiment file was edited.

MP1 ENet segmentation, bird's-eye transformation and lane fitting are planned
sources for later porting, not working components of this starter. Existing MP2
logs can support a **prior path-tracking baseline** only; they do not demonstrate
vision-based lane following, stop behavior or a two-lap final-project result.

## Existing environment

`environment.lock.json` records the inspected local environment and source hashes.
The existing image uses Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / Python 3.12.
It has a single system-site-packages virtual environment and ROS/Ubuntu OpenCV.
Reuse it without replacing `cv2` with a PyPI wheel or altering the MP environment.

The simulator checkout is pinned at
`190e21da4394d4ce028d42d2fadf95a9b8eb1f47` in
[UIUC-Robotics/gem_simulator](https://github.com/UIUC-Robotics/gem_simulator).
The original README's older ROS/Gazebo instructions must not override the
verified local Jazzy/Harmonic setup.

The checked-in patch reproduces the local CMake compatibility change: removing
two `find_package` calls for `ros2_control`/`ros2_controllers` metapackages. It does
not remove the installed runtime dependencies. In a new checkout at the recorded
commit, inspect and apply it with:

```bash
git apply --check /path/to/SLY/code/patches/gem-simulator-jazzy-metapackages.patch
git apply /path/to/SLY/code/patches/gem-simulator-jazzy-metapackages.patch
```

The existing local simulator already has this change; do not apply it twice.
No model weights, simulator clone, Docker image, rosbag or build/install/log
tree is bundled here. An image digest and complete installed-package snapshot
have not been captured, so this is source/configuration provenance rather than
a bit-for-bit reconstruction guarantee.
