# Hardware adapter — interface proposal only

**No PACMod publisher, joystick demo, hardware launch or physical-vehicle
validation is implemented in this repository.** The simulator node must stay
isolated from the vehicle network. A successful synthetic test is not hardware
evidence, and the three-second simulator dwell is not a course-approved rule.

Before implementation, record the assigned vehicle model, wheelbase, firmware,
ROS distribution, computer, camera/depth availability, LiDAR extrinsics, IMU/GNSS,
topic names, message types, units, QoS, coordinate frames and update rates.
Do not assume that all equipment mentioned in the GEM e2/e4 documentation is
installed on the assigned vehicle.

The future adapter consumes a timestamped desired front-wheel angle, desired
speed, stop reason and enable state. It must produce measured steering and
speed feedback, actuator faults and manual-control state. It must explicitly
convert front-wheel angle to the vehicle's verified steering-wheel/PACMod
convention and implement closed-loop throttle/brake control. Throttle/brake pedal
percentages are not equivalent to speed commands.

One arbiter owns final commands. Prove mutually exclusive throttle/brake use,
bounded commands, input timeout, process-death watchdog, steering limit,
continuous stopping behavior and manual takeover independently. Releasing
manual control must not re-enable autonomy. Establish speed, braking distance
and dwell requirements through course staff and supervised low-speed tests.
Turning off drive-by-wire must not be treated as an automatic brake.

The first hardware milestone remains the supervised joystick demo, then sensor
inspection and rosbag collection, then offline perception validation, before
autonomous motion trials. If hardware uses ROS Humble, build a separate matching
environment; do not copy Jazzy compiled outputs or virtual environments.

Reference entry points:

- [GEM resource page](https://publish.illinois.edu/robotics-autonomy-resources/gem-e2/)
- [Hardware setup](https://github.com/UIUC-Robotics/gem_ws/wiki/Hardware-Setup)
- [Software setup](https://github.com/UIUC-Robotics/gem_ws/wiki/Software-Setup)
- [Drive-by-wire instructions](https://github.com/UIUC-Robotics/gem_ws/blob/main/src/basic_launch/docs/dbw_instructions.md)
