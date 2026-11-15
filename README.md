# Team SLY — ECE484 GEM Final Project

Current stage: **Milestone 3 image-based lane-following development**. The controller uses normalized image features, not a metric path or Pure Pursuit. Missing recorded camera-to-body TF does not block this mode. Vehicle installation/reference positions still require field verification.

**Milestone 3 is not complete.** Camera-driven simulation and PACMod adapter code are implemented locally; continuous-turn and recorded-data acceptance are still under evaluation. No physical autonomous lane-following test has been performed in this work.

[Project website](https://safeautonomy-illinois-students.github.io/fa26_ece484_final_project_site_team_SLY/) · [Course requirement](https://safeautonomy-illinois.github.io/ece484-site/fall-2026/projects/robotaxi/) · [中文启动与上车指南](MILESTONE3.md) · [本地试验报告](results/milestone3/report.md)

## 最新 Highbay 结果（2026-11-13）

**未通过完整急弯及三次重复验收。** 3 个速度对照、9 组图像方向增益及 2 次超时重跑均保留；有效入弯试验在约 11.7–13.0 m 进度处丢失右边界或车体包络越界，规定路线为 49.07 m。降至 0.1 m/s 未消除问题。相机、检测与停车条件未放宽；默认仿真参数及 e4 配置保持原样。

控制器新增可选 `direction_mode: tangent` 与诊断字段，所有实验参数只保存在对应结果目录。完整录像、轨迹、静止视野对照和复现命令见 [Highbay 实验报告](results/highbay_control/report.md)。

## 当前实现

- One package: `src/sly_lane_following`. `core.py` handles paint candidates, left/right identities, optical-flow-assisted association and corridor selection; `control.py` converts near/far image errors to speed/front-wheel steering requests.
- `node.py` shares the detector with offline analysis. `adapter.py` is the single final command outlet, separately enabled and latched off after faults. It provides Gazebo and PACMod modes; hardware defaults to **observe-only** with unverified configuration.
- RGB and CameraInfo are the perception inputs. No MP workspace import, vehicle TF, LiDAR/GNSS/IMU fusion or stored-waypoint steering is used.
- Both boundaries must have current evidence. Identity memory does not count as an observed centreline. Ambiguity, stale input or missing feedback stops control; fresh input alone does not re-enable motion.
- Simulator and e4 configuration are separate. The current simulator is the official **e2 model**, not an e4 vehicle-parameter validation.

| Topic | Meaning |
|---|---|
| `/sly/lane/debug_image` | Timestamped candidates, selected boundaries, pixel centreline, control/reference points |
| `/sly/lane/status` | Visual validity and image-space control diagnostics; score is heuristic, not probability |
| `/sly/lane/local_path` | Always empty; `NOT_USED_IMAGE_SERVO`. Pixels are never presented as metres |
| `/sly/control/request` | `AckermannDriveStamped`: requested m/s and front-wheel radians |
| `/sly/control/status` | Final outlet state and stop reason |
| `/sly/control/enable` | `SetBool`; explicit enable / latched stop |
| `/sly/lane/reset` | `Trigger`; perception reset only, not actuator enable |

## 构建、回放与离线回归

在宿主机执行：

```bash
cd /home/xinweiyu/Workspace/ECE484_Program/fa26_ece484_final_project_code_team_SLY
./scripts/dev.sh up
./scripts/dev.sh build
./scripts/dev.sh lane-offline --output results/milestone3/my_review
```

每次使用新的结果目录。分别在三个终端运行 `./scripts/dev.sh replay`、`./scripts/dev.sh lane`、`./scripts/dev.sh lane-view`；回放终端按空格开始/暂停，各终端 Ctrl+C 结束。回放感知不加载控制配置，不发布执行器命令。不要同时运行回放和仿真（它们共用项目通信域）。

`--start 108 --end 120` 可建立显式重启的片段试验；这不表示连续运行自动恢复。完整视频、逐帧 JSONL、每秒图片和源码配置快照均保存在被忽略的结果目录。

早期基线见 [Stage 2A 报告](results/stage2/report.md)。这些数据已反复用于发现问题，现在全部属于**已知数据回归**，不再称为独立验证。每秒参考标注来自 Codex 对原图的粗略视觉审阅（约 ±10 px），待人工复核；不是实测几何真值。覆盖率和错误走廊应一起报告，拒绝所有结果不算验收通过。

测试：`./scripts/dev.sh exec python -m pytest src/sly_lane_following/test -q`。ROS 故障测试和 PACMod 模拟消息测试见本地报告。

算法参考 MP1 的滑窗/拟合思想，但源码不依赖 MP 包或旧标定。相机模型参考 [OpenCV](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)，拟合参考 [RANSAC](https://www.sri.com/wp-content/uploads/2021/12/ransac-publication.pdf)。PACMod 转向映射来源为本地官方 `gem_ws/src/vehicle_drivers/gem_odometry_control/gem_odometry_control/pure_pursuit_odom.py` 的 `front2steer`；必须现场验证。

## 第一阶段：在现有 Docker 中查看采集数据

以下命令均在**宿主机终端**执行；脚本会自动进入现有 Jazzy Docker 服务。不要在已进入容器的终端再次运行此入口。

### 终端 1：启动并回放

```bash
cd /home/xinweiyu/Workspace/ECE484_Program/fa26_ece484_final_project_code_team_SLY
./scripts/dev.sh up
./scripts/dev.sh replay
```

默认从头开始、**暂停、0.5 倍速**。先打开下面两个查看终端，再回到终端 1 按 **空格**开始。首次暂停时还没有图像属于正常情况；LiDAR / IMU 要到录制起点后约 2.7 秒才出现。

### 终端 2：RGB 与 LiDAR

```bash
cd /home/xinweiyu/Workspace/ECE484_Program/fa26_ece484_final_project_code_team_SLY
./scripts/dev.sh view
```

RViz 中 `Front OAK RGB` 显示前置相机；主三维窗口显示 `/ouster/points`。固定坐标系为 `os_lidar`，滚轮缩放、拖动旋转。相机和点云分别显示；本 bag 没有 TF，不把两者叠加成未经标定的融合画面。

### 终端 3：IMU 曲线

```bash
cd /home/xinweiyu/Workspace/ECE484_Program/fa26_ece484_final_project_code_team_SLY
./scripts/dev.sh imu
```

单窗口上下两图：角速度 xyz（rad/s）、加速度 xyz（m/s²）。横轴使用消息 header 时间，展示最近 10 秒；暂停时保留曲线，重播时间回跳时清空。此 IMU 的姿态无效，不显示航向；加速度包含重力，z 轴接近 9.8 m/s² 不代表车辆向上加速。

### 暂停、慢放、重播与停止

- 在**终端 1**按空格：暂停 / 恢复；↑ / ↓：速度增加 / 降低 10%；暂停时 →：前进一条消息。
- 各终端 `Ctrl+C`：结束各自程序；结束回放后可保留查看窗口，重新运行 `replay`。不需要停止整个容器。
- 原速、从第 75 秒开始：先停止旧 player，再执行下面命令，仍从暂停开始。

```bash
./scripts/dev.sh replay --rate 1.0 --start-offset 75
```

也可以在仓库目录的另一个宿主机终端执行 `./scripts/dev.sh pause` 或 `./scripts/dev.sh resume`。全长约 129 秒，0.5 倍速播放约需 258 秒。不要同时开两个 player；脚本会拒绝检测到的重复回放。

### 已完成的数据检查

全量检查读取并反序列化 **24,801 条**消息，全部 RGB 可解码：

| Topic | 数量 | 说明 |
|---|---:|---|
| `/oak/rgb/image_raw` | 3,870 | 768 × 480，bgr8，约 30 Hz |
| `/oak/rgb/camera_info` | 3,870 | 与图像时间戳全部匹配 |
| `/ouster/points` | 1,264 | 约 10 Hz，逐条检查字段与布局 |
| `/ouster/imu` | 15,797 | 约 125 Hz，角速度与加速度有效，姿态无效 |

未发现 header 时间逆序或重复。发现图像 / CameraInfo 同一处约 **133 ms** 间隙，以及 IMU 一处约 **34 ms** 间隙；不能仅据此断言原因。未录制 TF、GNSS、深度图和车速反馈，不影响当前原始数据查看，但不能据此宣称完成标定或闭环验证。

本地报告：[report.md](results/stage1/report.md)、[详细时间统计](results/stage1/report.json)、[RGB 抽样](results/stage1/rgb_contact.jpg)、[点云 XY 抽样](results/stage1/pointcloud_xy_preview.png)、[IMU 曲线](results/stage1/imu_preview.png)、[回放验证](results/stage1/playback_verification.md)。这些文件被 Git 忽略。重新检查：

```bash
./scripts/dev.sh inspect
```

请完整查看直线、急弯、逆光和多条标线片段，确认画面方向、颜色、连续性及场地内容。原始数据已由你确认；第二阶段入口见上方。

### 环境与文件约定

- 继续使用 Compose 项目 `ece484` / 服务 `ece484`，Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。没有新建 Humble 环境或独立项目容器。
- `compose.sly.yaml` 只为现有服务增加只读挂载：宿主机 `/mnt/Data/01_Projects/ECE484_GEMe4/data` → 容器 `/data/gem`。路径或 bag 不存在时直接报错，不自动创建空数据目录。
- 新终端绕开 shell 启动文件，只 source `/opt/ros/jazzy/setup.bash` 和存在时的本项目 `install/local_setup.bash`，不加载 MP 或 simulator。通信域为 `84`、仅本机发现。原作业启动文件不变。
- 项目 shell：`./scripts/dev.sh shell`。已有 Python 环境中的 OpenCV 4.6、NumPy 1.26.4、cv_bridge、rosbag2_py、RViz2 和 Matplotlib 足以完成本阶段；无需额外安装。
- 本 bag 来自 Humble，已在 Jazzy 完整解码。旧元数据的 `ros_distro` 可能显示 `unknown`，SQLite schema 中记录为 `humble`。
- RViz 回放配置使用 Reliable 订阅，匹配该 bag 的发布 QoS；本机实测 Best Effort 会丢失大图像 / 点云。这是**回放查看配置**，实车 QoS 需要按真实驱动另行配置。
- 新挂载生效前已保存原容器的本地镜像快照，由被忽略的 `.sly-local.env` 选择。回退说明在 [results/environment_backup/README.md](results/environment_backup/README.md)，避免用基础镜像覆盖原容器内修改。
- 原始数据只读；报告与实验输出放 `results/`。嵌套官方 `gem_ws`、构建产物和大数据不纳入本项目 Git。第二阶段仅增加 `src/sly_lane_following` 一个包，构建范围限定为本项目 `src`。

---

## 历史 Pitch / simulator setup 记录

以下保留先前记录，不代表当前硬件或算法进度。后续车道跟随使用 `highbay_testbed.world`；Sonoma 记录只证明旧的环境启动与命令响应。

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

The simulator launch, RGB/depth streams and response to speed commands were checked on October 19, 2026, as documented below. Hardware setup, the physical Joystick demo and autonomous-driving checks remain pending. Proposed responsibilities and milestones are on the project website.

## Local simulator check — October 19, 2026

The current setup recording was made on Sonoma Raceway using the official GEM simulator at
`190e21da4394d4ce028d42d2fadf95a9b8eb1f47`, ROS 2 Jazzy and Gazebo Harmonic
8.15.0 in the existing container. The upstream checkout retains its previously
documented CMake compatibility change and the two-line Sonoma asset-path fix below;
no new environment installation was needed.

- World: `sonoma_raceway.world`; launch pose `(x=0, y=0, yaw=-0.5)`.
- Observed: RGB `rgb8` and depth `32FC1`, both 800 × 600, plus odometry and
  Ackermann command response.
- Recording: 28.05 seconds, 1280 × 800, H.264. Predefined straight-driving commands
  requested 0, 0.6, 0, 0.4, then 0 m/s. Observed displacement was approximately
  3.52 m; maximum measured speed 0.60 m/s; final measured speed 0 m/s.
  The video follows wall-clock time; odometry spans 11.84 seconds of simulation
  time. This run does not establish real-time performance.
- Scope: simulator/setup evidence only. No lane controller, stop-sign detector,
  perception-triggered stop or physical vehicle test was performed.
- The external view comes from a temporary static observer camera. RGB and depth
  are real simulator sensor outputs; depth is displayed on a fixed 0–30 m scale
  with invalid values black.

The local capture bundle, including the headless launch copy, observer SDF,
capture script, timestamps/odometry log and video, is saved outside the repos at:

`/home/xinweiyu/Workspace/ECE484_Program/SLY_local_review/sonoma-setup-2026-10-19/`

### Repeat the setup with the official interactive launch

From `/home/xinweiyu/Workspace/ECE484_Program` on the host:

```bash
docker compose exec -e ROS_DOMAIN_ID=84 -e GZ_PARTITION=sly_sonoma_fixed ece484 bash --noprofile --norc
```

Inside that shell, source only ROS and the official simulator:

```bash
source /opt/ros/jazzy/setup.bash
source /workspace/gem_simulator/install/setup.bash
ros2 launch gem_launch gem_init.launch.py world_name:=sonoma_raceway.world x:=0 y:=0 yaw:=-0.5 start_rviz:=false
```

Open a second shell with the same Docker command and source the same setup files.
Inspect `/camera/image_raw`, `/depth_camera/image_raw` and `/odom`. The official
Gazebo GUI can display the two image topics; the recording used headless sensor
rendering and a separate observer camera instead of desktop screen capture.

For a short **simulation-only** forward-command check:

```bash
timeout --signal=INT 8s ros2 topic pub --rate 10 /ackermann_cmd ackermann_msgs/msg/AckermannDrive '{steering_angle: 0.0, steering_angle_velocity: 0.5, speed: 0.5, acceleration: 0.5}'
ros2 topic pub --once /ackermann_cmd ackermann_msgs/msg/AckermannDrive '{steering_angle: 0.0, speed: 0.0}'
```

Observe speed feedback, then stop the launch with Ctrl+C. These are simulator
commands, not PACMod hardware instructions.

### Setup findings to carry into implementation

- RGB horizontal FOV is approximately 80°; depth is 60° in this revision. Equal
  image dimensions and frame labels do not imply pixel alignment. Register range
  into RGB coordinates and validate timing before using detection-box distances.
- Sonoma loaded successfully after the local asset-path fix described below.
  The earlier `smaller_track` recording remains in the previous local audit bundle.
- The upstream README mentions Humble/Fortress, but the pinned launch uses Gazebo
  8. Keep this working environment; verify the real vehicle's ROS version separately.


### Sonoma asset-path fix

The world file is present in the upstream repository. Its model also includes
`meshes/Raceway.obj`, `meshes/Raceway.mtl` and the texture files locally, but
`gem_gazebo/models/sonoma_raceway/model.sdf` originally references the remote
Fuel mesh in both the collision and visual blocks. During the October 19 check,
Gazebo's model-archive request failed with:

```text
Failed to download model.
Server: https://fuel.ignitionrobotics.org
Route: openrobotics/models/sonoma raceway/3/sonoma raceway.zip
REST response code: 500
```

In the **official simulator checkout**, replace both mesh `<uri>` values with:

```xml
<uri>model://sonoma_raceway/meshes/Raceway.obj</uri>
```

Keep the surrounding model, scale and world file unchanged. This workspace uses
a symlink install, so the installed model resolves to the edited source. For a
copied install, rebuild the simulator workspace before relaunching. No dependency
reinstallation is needed for this fix.

The local audit bundle preserves the original model, failed-load log, successful
launch log, capture script and sensor/odometry timestamps. The new recording uses
the original GEM model and Sonoma geometry; the added fixed observer camera is
only for visualization. The recording and the new website remain local pending
team review.
