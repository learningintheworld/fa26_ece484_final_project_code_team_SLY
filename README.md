# Team SLY — ECE484 GEM Final Project

This repository is at the **Pitch and setup stage**. The project will implement camera-based lane following, stop-sign and obstacle stopping, and automatic resumption on the GEM. There is no autonomous-driving implementation in this repository yet.

[Project website](https://safeautonomy-illinois-students.github.io/fa26_ece484_final_project_site_team_SLY/) · [Course requirements](https://safeautonomy-illinois.github.io/ece484-site/fall-2026/projects/robotaxi/)

## Setup plan

1. Use the existing Docker environment at `/home/xinweiyu/Workspace/ECE484_Program` as the candidate simulation environment: Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. Reuse the environment only; develop the final project independently in this repository.
2. Verify the official [GEM simulator](https://github.com/UIUC-Robotics/gem_simulator) and its dependencies against the installed versions before adding project code. The simulator README and checked-out revision may describe different ROS/Gazebo versions; record the working revision once project setup is verified.
3. Review the [GEM getting-started guide](https://github.com/UIUC-Robotics/gem_ws/wiki#getting-started), [hardware setup](https://github.com/UIUC-Robotics/gem_ws/wiki/Hardware-Setup) and [software setup](https://github.com/UIUC-Robotics/gem_ws/wiki/Software-Setup).
4. Confirm training and supervised vehicle access, inspect the actual sensors/ROS version, and complete the required Joystick controller demo. Build separately if the hardware uses a different ROS distribution.
5. Add a minimal project ROS package only after the simulator, sensor topics and coordinate conventions are confirmed. Keep simulation and PACMod hardware interfaces separate.

## Initial checks to record

- Simulator launches and the camera and vehicle-state topics publish correctly.
- Topic names, message types, QoS, timestamps and coordinate frames are documented.
- A dated project-specific simulator recording and a supervised hardware-demo recording are available.
- Vehicle stop, manual takeover and disengagement procedures are verified with a mentor. Disabling PACMod alone does not guarantee braking.

These checks are pending; this README is a setup plan, not a completed validation report. Proposed team responsibilities and milestones are on the project website.
