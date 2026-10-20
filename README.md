# SLY GEM — project code

**ECE484 · Fall 2026 · Team SLY · Pitch starter**

The project goal is camera-based lane following, stop-sign and obstacle stops,
automatic recovery, and two complete laps on the assigned closed course.
This repository currently provides a **simulation-only control foundation**.
It is not an end-to-end autonomous-driving implementation.

[Project website](https://safeautonomy-illinois-students.github.io/fa26_ece484_final_project_site_team_SLY/)
· [Course GEM task](https://safeautonomy-illinois.github.io/ece484-site/fall-2026/projects/robotaxi/)

## Current evidence and limits

| Component | Status |
|---|---|
| Low-speed Pure Pursuit and curvature-based speed cap | Implemented; pure-Python unit tests |
| Stop supervisor: freshness, obstacle hold, sign dwell/rearm, manual latch | Implemented; pure-Python unit tests |
| ROS 2 `nav_msgs/Path` → `AckermannDrive` simulation adapter | Colcon build and 8 synthetic ROS smoke checks passed |
| MP1 perception and MP2 controller | Prior coursework; references recorded, old assignments not modified |
| Camera segmentation, YOLO detection, depth/LiDAR fusion | Planned; not implemented here |
| Visual closed loop, two-lap project validation | Not yet demonstrated |
| PACMod integration, real-vehicle joystick demo | Not implemented/verified here |

The node starts **disabled**, requires a simulation clock, and publishes to
`/sly/sim/ackermann_cmd`, which is disconnected from GEM by default. It cannot
publish PACMod commands. Do not connect this starter to a physical vehicle.
Zero Ackermann speed is a simulator target, not a verified physical brake.

## Run core tests on the host

From this repository's root, with Python 3.10 or newer:

```bash
PYTHONPATH=src/sly_autonomy python3 -m unittest discover -s tests -v
```

There are no pip dependencies for the core tests. Do not run the container's
`/workspace/.venv` interpreter on the host.

After building and sourcing the workspace in the ROS container, the synthetic
adapter test can be repeated with `ROS_DOMAIN_ID=84 python3 tests/ros_smoke.py`.
It refuses an already-used test domain and shuts down the node it starts.
This is message-level verification, not a Gazebo driving demonstration.

## Build in the existing ECE484 container

The existing environment is Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic;
its simulator source is pinned in [`environment.lock.json`](environment.lock.json).
The workspace is independent of all MP build directories.

1. Stop any active simulator/MP experiment before changing container mounts.
2. From the code repository root on the host, add this checkout as `/sly`:

```bash
export SLY_CODE_DIR="$PWD"
docker compose \
  -f /home/xinweiyu/Workspace/ECE484_Program/compose.yaml \
  -f "$SLY_CODE_DIR/compose.sly.yaml" up -d --no-build
docker compose \
  -f /home/xinweiyu/Workspace/ECE484_Program/compose.yaml \
  -f "$SLY_CODE_DIR/compose.sly.yaml" exec -w /sly ece484 bash
```

3. Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select sly_autonomy
source install/setup.bash
ros2 launch sly_autonomy sim.launch.py
```

The idle controller needs `/clock`; while the simulator is paused, ROS-time
control ticks also pause. A running clock alone cannot enable motion.
Connect a tested local-path producer, odometry, and a perception status producer
using the exact contracts in [`docs/interfaces.md`](docs/interfaces.md).
Those producers are not supplied as working perception in this starter.

For an isolated simulator only, the output can be explicitly remapped using
`command_topic:=/ackermann_cmd` and `odom_topic:=<verified odometry topic>`.
Ensure that the MP2 controller and joystick publisher are not competing on that
command topic. Only then request enable:

```bash
ros2 service call /sly/enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /sly/state
```

Enable is rejected without fresh, valid inputs. To disable:

```bash
ros2 service call /sly/enable std_srvs/srv/SetBool '{data: false}'
```

## Project documents

- [Interfaces and state behavior](docs/interfaces.md)
- [Validation record and acceptance cases](docs/validation.md)
- [Milestones, team proposals and remaining work](docs/roadmap.md)
- [MP2 attribution and simulator/environment provenance](docs/provenance.md)
- [Hardware interface checklist — unimplemented](docs/hardware-interface.md)

The simulator numbers in the configuration are initial development choices.
They are not measured vehicle limits or final course acceptance thresholds.
