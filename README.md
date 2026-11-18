# Team SLY — ECE484 GEM Final Project

Current stage: **Milestone 3 monocular lane-following development**. An optional visual Pure Pursuit controller now projects observed lane centres into rear-axle ground coordinates using model-based camera geometry. The original image controller remains available. Official e4 installation geometry is the accepted nominal default; simulation uses its own e2 geometry.

**Milestone 3 is not complete.** The Highbay straight-and-combination-bend route has passed three independent simulation runs. Recorded-data acceptance and physical vehicle testing remain outstanding. No physical autonomous lane-following test has been performed in this work.

[Project website](https://safeautonomy-illinois-students.github.io/fa26_ece484_final_project_site_team_SLY/) · [Course requirement](https://safeautonomy-illinois.github.io/ece484-site/fall-2026/projects/robotaxi/) · [中文启动与上车指南](MILESTONE3.md) · [本地试验报告](results/single_boundary/report.md)

当前代码包含ENet车道识别、视觉Pure Pursuit、Highbay曲率预瞄调速及e4 PACMod启停准备。最新Highbay版本完成一次独立整圈，到终点882.418秒；实车尚未验收。实车命令见 [E4_QUICKSTART_ZH.md](E4_QUICKSTART_ZH.md)，网页版AI排障背景见 [E4_AI_HANDOFF_ZH.md](E4_AI_HANDOFF_ZH.md)。v3及可选night扩容权重均随仓库提供，默认仍为v3。以下历史报告、原始数据和视频在Git忽略的 `results/`，不随代码上传。

## 最新发布状态（2026-11-18）

发布前完整单元测试：**295通过、1跳过**；跳过项要求真实Torch1.13环境，既有兼容报告单独覆盖。两套权重、官方e4投影、Pure Pursuit公式、实车配置和硬件验证门槛未改。

| 入口 | 本次代码及验证 | 使用范围 |
|---|---|---|
| 实车 `e4.sh` | 保留v3/night、GPU/CPU选择，启停命令不变 | 新交接与有限缓存尚未接入实车；先观察和现场验收 |
| Highbay实时节点 | 可靠图像订阅、交汇处身份保持、实测里程计补偿的有限缓存 | 一次独立整圈通过；不代表实车验收 |
| 离线 `--handoff` | 分段质量记录，联合判断当前交接与冲突，同步已确认边界身份 | 真实RGB候选缓存回归，非实车轨迹 |

**离线结果按三层区分：**

- **识别：**沿用两套冻结网络结果，没有重新训练。近场、预瞄和远场分别记录支持长度、完整度及拟合质量；这些是几何证据，不是经标定的网络置信概率。
- **中心线：**修复已接受新边界但走廊锚点仍保留旧身份的状态不同步；先评估可信当前交接，再检查当前近场冲突。110–114秒night候选有效帧88/120→120/120；15–17秒直线、20–23秒弯道、65–67秒多线对照均无有效性或转向变化。
- **控制请求：**冻结网络输出的3,870帧完整下游回归中，v3有效3310/3870（85.53%），原有有效转向不变；night候选2827→3312/3870（73.05%→85.58%），恢复112.935秒至结束的485帧，无原有效帧变无效。仍有约0–11.2秒和38.9–46.3秒无效区间，不能称全段通过。79个原先有效的night帧在交接附近转向发生变化，最大差0.1713 rad；这不是对所有画面的零误选证明。

本轮离线缓存最长1.467秒、目标最远6.994米，保留1.5秒与7米限制。回放运动是假设量，不是实测反馈；短时缓存未因此接入实车。图像、分段质量、转向对比和完整统计见本地 `results/near_priority_v1/`。`--physical-identity`仍只记录物理身份元数据，不自动授权车道角色。

## Highbay：可靠订阅与实时缓存整圈验证（2026-11-18）

`highbay_enet.yaml`启用交汇区外的可见曲线身份匹配；`sim_pure_pursuit.yaml`启用`runtime_path_cache`。实时缓存使用实测`/odom`线速度与角速度，最多1.5秒、0.3米，续行速度上限0.1 m/s；预测不刷新观测年龄。当前观测恢复后重新生成路径。真实近场冲突、明确歧义、反馈过期或缓存到期仍停车；缓存不掩盖图像停帧或控制请求超时。此入口只允许仿真时间和模拟车辆几何，e4入口未启用。

最新一次独立整圈：要求137.531米，进度137.649米，到终点882.418秒，最小车身/轮胎余量0.116米，驾驶期间零失效控制帧，终点停稳。缓存参与2帧，最长0.066秒，随后恢复当前右侧观测。请求时延中位61 ms、p95 116 ms。先前52–60米局部验证中缓存参与14帧，最长0.165秒。录像及验收记录保留于本地 `results/highbay_cache_lap_20261118/`；此次发布没有重跑整圈。

复现当前ENet、自适应速度整圈（先关闭其他仿真/感知/适配器，输出目录须未存在）：

```bash
./scripts/dev.sh exec python scripts/highbay_repeat.py --route full-lap --repeats 1 --speed 0.2 --lookahead 3.5 --adaptive-speed --perception-config src/sly_lane_following/config/highbay_enet.yaml --output results/my_current_enet_lap
```

`--adaptive-speed`启用直线上限1 m/s的曲率预瞄调速；`--speed 0.2`是基础控制配置，不是该模式的整段速度上限。赛道真值只用于独立评估和终点停车。下面按日期保留旧实验记录；其中“本地／未上传”描述当时状态，当前功能范围以上表为准。

## ENet Highbay 整圈验证（2026-11-15）

**现有 ENet v3 权重已完成一次独立整圈：137.55 m，1074.0 s，驾驶期间零失效控制帧，终点停稳。** 最小车身/轮胎包络余量 0.117 m；出弯仍有摆动，不能视为实车验收。

新仿真配置 `src/sly_lane_following/config/highbay_enet.yaml` 使用 ENet 与当前 RGB 标线支持、近期观测身份关联和近场片段连接优先级。原 ENet 权重、Pure Pursuit、速度上限 0.2 m/s、请求前视 3.5 m，以及原 Highbay / e4 控制配置均未改变。两侧消失或真正选道歧义仍停车。141 项单元测试及隔离域故障停车检查通过。

[本地完整报告](results/highbay_enet_v1/report.md) · [整圈录像](results/highbay_enet_v1/row_identity_attempt_16/repeat_1/closed_loop_h264.mp4) · [独立验收](results/highbay_enet_v1/row_identity_attempt_16/qualification.json) · [轨迹与余量](results/highbay_enet_v1/row_identity_attempt_16/evaluation.png)

```bash
./scripts/dev.sh exec python scripts/highbay_repeat.py --route full-lap --repeats 1 --speed 0.2 --lookahead 3.5 --perception-config src/sly_lane_following/config/highbay_enet.yaml --output results/my_enet_full_lap
```

输出目录需未存在；本入口会独立启动并停止仿真。运行配置使用随仓库提供的 `models/enet_v3.pth`；从仓库根目录执行，无需另行复制权重。先用故障 PNG 窗口和局部闭环定位问题，再执行完整验证；所有失败记录保留。此次仅本地修改。

## 整圈与提速验证（OpenCV 基线，2026-11-15）

**0.2 m/s、请求前视 3.5 m：一次独立整圈通过。** 近场边界身份、片段几何和历史车道宽度联合过滤已修复原 55.69 m 选道问题。连续完成 137.55 m，耗时 1076.6 s，最小车身/轮胎余量 0.0985 m，无无效控制帧或中途重新启用，终点停稳。

同一冻结版本的 **0.3 m/s 提速尝试未通过**：在 42.87 m 因可靠单侧片段支持不足停车，未越界。默认仍为 0.2 m/s，不降低可靠性门槛。88 项单元测试与 ROS 故障停车检查通过。详细失败记录、录像和复现命令见 [本地报告](results/nearfield/report.md) · [整圈录像](results/nearfield/full_speed_020_v2/repeat_1/closed_loop_h264.mp4) · [轨迹图](results/nearfield/full_speed_020_v2/full_lap.png)。这不是实车验收。

## 此前 49 m 组合弯三次验证

**固定源码和参数，三次独立重启全部通过完整路线。** 采用可靠的当前单侧标线、历史双边界测得的宽度及 Pure Pursuit；速度上限 0.2 m/s，前视距离 3.5 m。路线约 49.07 m，包含完整右侧组合弯及出口直行至少 3 m。

| 独立运行 | 进度 m | 耗时 s | 最小车身余量 m | 无效控制帧 |
|---|---:|---:|---:|---:|
| 1 | 49.09 | 437.8 | 0.098 | 0 |
| 2 | 49.09 | 437.8 | 0.098 | 0 |
| 3 | 49.09 | 437.8 | 0.097 | 0 |

各次均在终点停稳，无中途重新启用。车身与轮胎包络均在车道内；最小余量接近 0.10 m，不能据此推断实车裕量。两侧都不可见、近处候选矛盾或输入过期仍停车锁定。

实现还修复了官方仿真驱动启动时轮 TF 尚未到达、轮距误读为零的问题，采用等待真实 TF 的薄包装；上游模型、相机与物理参数未改变。

[完整本地报告](results/single_boundary/report.md) · [三次验证记录](results/single_boundary/qualification_v1/qualification.json) · [轨迹与余量图](results/single_boundary/qualification_v1/qualification.png) · [第 1 次完整录像](results/single_boundary/qualification_v1/repeat_1/closed_loop_h264.mp4)

单包构建和普通分终端运行见 [操作指南](MILESTONE3.md)。独立重复脚本会自行启动和停止各次仿真；先关闭现有项目仿真、感知和适配器，使用新输出目录：

```bash
./scripts/dev.sh build
./scripts/dev.sh exec python scripts/highbay_repeat.py --lookahead 3.5 --output results/single_boundary/my_repeats
```

76 项单元测试与 ROS 故障联调通过。单侧功能仅在 Highbay 配置中启用；真实数据回归和实车闭环仍未验收，e4 执行器继续保持观察模式。全部实验产物均在 Git 忽略的 `results` 中。

## 此前双边界 Pure Pursuit 结果（2026-11-14）

**该旧版本当时未通过完整路线或三次重复验收。** 0.2 m/s 上限、4.0/4.5/5.0 m 前视测试在约 9.70/10.02/10.41 m 处因双边界支持不足停车；对应最小车身余量为 0.62/0.55/0.46 m。第三组包含一个已修复的标线片段合并问题，原失败记录保留。完整路线仍为 49.07 m，不能把这三组失败试验称作三次通过。

新代码、全段 3,870 帧真实数据回归、259 张未标注照片与实验录像见 [本地 Pure Pursuit 报告](results/pure_pursuit/report.md)。真实数据连续跟踪约 20.5 s 后锁定失效，总覆盖率仅 7.21%；没有达到验收要求。该阶段照片只做了固定划分；后续真实 RGB 网络标签、训练和回归见文末。

- 仿真感知/控制入口：`./scripts/dev.sh lane-sim-pp`（替换 `lane-sim`，其他终端不变）。
- bag 中仅观察 e4 名义投影：`./scripts/dev.sh lane-pp`；不启动执行器适配器。
- 重跑全段：`./scripts/dev.sh lane-offline --output results/pure_pursuit/my_review --control-config src/sly_lane_following/config/e4_pure_pursuit.yaml --export-dataset`。
历史仿真对应各结果目录中的源码快照；当前完整路线使用本页顶部的新命令。

以上为双边界旧版本的失败基线。最新单侧仿真结果见本页顶部；原始记录保留。

## 此前 Highbay 结果（2026-11-13）

**该旧版本当时未通过完整急弯及三次重复验收。** 3 个速度对照、9 组图像方向增益及 2 次超时重跑均保留；有效入弯试验在约 11.7–13.0 m 进度处丢失右边界或车体包络越界，规定路线为 49.07 m。降至 0.1 m/s 未消除问题。相机、检测与停车条件未放宽；默认仿真参数及 e4 配置保持原样。

控制器新增可选 `direction_mode: tangent` 与诊断字段，所有实验参数只保存在对应结果目录。完整录像、轨迹、静止视野对照和复现命令见 [Highbay 实验报告](results/highbay_control/report.md)。

## 当前实现

- One package: `src/sly_lane_following`. `core.py` handles paint candidates, left/right identities, optical-flow-assisted association and corridor selection; `control.py` retains image-error control; `geometry.py` provides ground projection and visual Pure Pursuit without importing MP workspaces.
- `node.py` shares the detector with offline analysis. `adapter.py` is the single final command outlet, separately enabled and latched off after faults. It provides Gazebo and PACMod modes; hardware defaults to **observe-only** with unverified configuration.
- RGB and CameraInfo are the perception inputs. No MP workspace import, recorded vehicle TF, LiDAR/GNSS/IMU fusion or stored-waypoint steering is used; Pure Pursuit uses configured nominal camera geometry.
- Highbay may use one reliably observed boundary and a previously verified lane width. With the opt-in simulation cache, temporary missing geometry can use a separately labelled, bounded motion-compensated path. Confirmed ambiguity still invalidates control; predicted points never become fresh observations. Ambiguity, stale input or missing feedback stops control; fresh input alone does not re-enable motion.
- Simulator and e4 configuration are separate. The current simulator is the official **e2 model**, not an e4 vehicle-parameter validation.

| Topic | Meaning |
|---|---|
| `/sly/lane/debug_image` | Timestamped candidates, selected boundaries, pixel centreline, control/reference points |
| `/sly/lane/status` | Visual validity and image-space control diagnostics; score is heuristic, not probability |
| `/sly/lane/local_path` | Pure Pursuit: metric centreline from current boundaries (single-side centres use historical width) in `sly_rear_axle_ground`; invalid input or legacy image mode: empty |
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

## 真实 RGB 急弯回归（2026-11-15）

真实配置已启用逐列补充扫描、分段拟合及保守的 e4 单侧推算；Pure Pursuit 和已通过的 Highbay 配置保持不变。3,870 帧完整回放中，明确双边界参考帧的候选匹配由 63/75 提升到 68/75；约 23.935 秒触发单侧 3 秒时限并锁定停止，**尚未通过全段连续跟随**。单侧误差只验证名义模型下一部分可对照的目标，不代表实车精度。

[本地报告与复现命令](results/real_transfer_v1/report.md) · [完整前后对比视频](results/real_transfer_v1/before_after.mp4) · [单侧检查](results/real_transfer_v1/masked_side_checks.json)。结果目录不提交 Git；本轮未微调 MP1，实车仍仅观察。


## 真实 RGB 神经网络主研究入口（本地实验）

现用ENet v3：修复训练标签与推理ROI不一致，补充12张人行道、路缘等局部背景照片。共126张训练、20张验证、25张评估；沿用MP1结构和v2权重微调。留出评估IoU为85.12%（v2：83.78%），远场IoU为67.15%（v2：62.71%）；近场略降。模型只生成掩膜，当前车道仍由几何与身份关联选择。

ENet v3原基线在23.935秒触发原单侧时限，候选双边界匹配69/75（v2：70/75）。本次仅改进历史宽度和边界重新关联，**全段连续跟随仍未完成**；最新结果见下节。Pure Pursuit、e4原有控制参数及Highbay配置保持不变，两侧不可见或真实歧义时继续停车。

- [ENet v3训练报告](results/enet_real_v3/report.md) · [129秒前后对照录像](results/enet_real_v3/full_comparison.mp4) · [23.935秒掩膜对比](results/enet_real_v3/compare_23.935_masks.jpg) · [旧版报告](results/enet_real_v2/report.md)。
- `./scripts/dev.sh lane-offline-enet --reference results/stage2/reference/reference.json --output results/enet_real_v3/replay_again`：全段离线分析，输出目录须为新目录。
- 回放三个终端：`./scripts/dev.sh replay`、`./scripts/dev.sh lane-enet`、`./scripts/dev.sh lane-view`；空格播放/暂停，分别Ctrl+C停止。避免同时启动其他感知节点。
- 原OpenCV比较入口和仿真入口不变；网络入口使用已有venv和CUDA PyTorch，运行不导入MP工作空间。源码、训练和评估来源见报告。
- 历史标签、训练产物、逐帧记录及视频保留在Git忽略的 `results`；当前部署权重另存为仓库中的 `models/enet_v3.pth`。

历史诊断：[23.935秒停止根因](results/single_timeout_v1/report.md)。新模型消除了该帧的远处人行道误检，但左边界退出视野及历史宽度有效性仍是独立问题；没有把该误检当作单侧超时的原因。


### 历史宽度与单侧续行更新（2026-11-15，本地预览）

保持ENet v3、Pure Pursuit及Highbay配置不变，仅在e4几何配置启用 `quality_width_history`：短残线不刷新历史宽度；保持当前右边界身份，返回左边界连续8帧确认后重新关联；单侧续行同时检查当前观测与名义误差预算。原3秒/0.6米阈值保留给旧策略，新策略并非单独延长时限。

完整3,870帧回放：23.935秒不再停止，31.702秒接回左边界，31.4–33秒连续有效；有效路径预览由382帧增至764帧（9.87%→19.74%）。**36.668秒误差预算超过0.25米后停止锁定，尚未完成全段连续跟随。** 预算是名义模型下的工程敏感度估计，不能视为实车误差保证。连续单侧试验有486个可比较目标，最大差异0.205米；另624个有效目标缺少参照，不能算作精度验证通过。

[本次报告](results/width_tracking_v1/report.md) · [129秒完整对比](results/width_tracking_v1/full_comparison.mp4) · [23.935秒对照](results/width_tracking_v1/compare_23.935.jpg)。上面的三终端回放命令仍适用；新离线输出可用 `./scripts/dev.sh lane-offline-enet --reference results/stage2/reference/reference.json --output results/width_tracking_v1/replay_again`（目录须不存在）。本轮125项测试通过、包构建通过，260帧仿真回归输出一致；未运行新的闭环整圈，未上传GitHub。


### 36.668秒预算超限修复（本地回放）

最新宽度估计仅使用最近可观测走廊的法向测量，避免远处弯开的线段污染历史宽度。预算、ENet、Pure Pursuit、e4与Highbay配置均未修改。36.668秒预算由0.2502米降至0.1496米，完整3,870帧回放已越过该点；有效预览764→778帧。**37.134秒仍因当前边界无法支持允许范围内的前视目标而停车，尚未全段通过。** 128项测试、构建和260帧仿真样本保护检查通过，未运行新闭环整圈、未上传GitHub。

[本次根因与验证报告](results/width_tracking_v2/report.md) · [完整前后录像](results/width_tracking_v2/full_comparison.mp4) · [下一停止点](results/width_tracking_v2/next_stop.json)。复现：`./scripts/dev.sh lane-offline-enet --reference results/stage2/reference/reference.json --output results/width_tracking_v2/replay_again`，输出目录须不存在。误差验证仍是名义模型和部分可比较样本，不能作为实车精度保证。


### 37.134秒目标支持诊断（未采用实验）

原图、ENet与边界ID检查表明，右线仍可见但近场左界缺失；按固定名义几何，37.134秒局部RGB参考最近中心目标约6.025–6.041米，超过当前6.0米范围，随后继续增大。试验修正了图像边缘截断截面的中点偏差，但完整回放停车反而提前至36.970秒，未达到采用条件，已撤回。**正式算法、ENet、Pure Pursuit与Highbay配置保持原样，仍在37.134秒停车，未实现全段连续跟随。** 恢复后重新读取3,870帧，候选和控制输出与上一版本逐项一致；128项测试与构建通过。

[完整诊断和未采用原因](results/target_support_v1/report.md) · [原图与掩膜](results/target_support_v1/frames/37.134_comparison.jpg) · [全段实验对照](results/target_support_v1/full_comparison.mp4)。回放的转向输出无法改变已经录制的车辆姿态，后续需用闭环验证是否能在丢视野前保持车道；本轮未扩大目标范围或上传GitHub。


### Highbay 自适应速度整圈验证（本地，2026-11-15）

保持ENet与Pure Pursuit转向不变，新增可选曲率预瞄调速及加速度/加加速度限制。直线上限1 m/s，单侧观测及急弯继续低速。局部测试后完成1次独立整圈：到终点883.033秒，停稳893.008秒，较旧版包含停稳时间缩短16.9%；最小车身/轮胎余量0.1165米，无故障停车。仅验证仿真，e4配置未变，未上传GitHub。

```bash
./scripts/dev.sh exec python scripts/highbay_repeat.py --route full-lap --repeats 1 --speed 0.2 --lookahead 3.5 --adaptive-speed --perception-config src/sly_lane_following/config/highbay_enet.yaml --output results/speed_profile_v1/reproduce_01
```

输出目录须不存在；`--speed 0.2` 保留旧弯道/单侧基准，自适应开关另行启用1 m/s直线上限。无开关时保留原行为。[完整报告及舒适性指标](results/speed_profile_v1/report.md) · [完整录像](results/speed_profile_v1/full_lap_03/repeat_1/closed_loop_h264.mp4)。


### GEM e4 实车移植准备

见 [按终端操作的精简中文指南](E4_QUICKSTART_ZH.md)。ENet与Pure Pursuit不变；补齐SLY的PACMod制动/换挡/退出确认流程，官方驱动不改。实车速度配置仍为0.2 m/s上限，未验证硬件参数仍为空；先只观察，现场验证后才闭环。

### 真实数据三层离线分析

在已有 Jazzy 开发容器中，从项目根目录执行：

```bash
python scripts/analyze_real_stages.py --output results/three_stage_review_02
```

输出目录须不存在。默认只读 `/data/gem/oak_front_image`，每帧重新执行 v3，分别记录车道候选、中心线和 Pure Pursuit 候选速度/转角。此入口不发布 ROS 控制命令；几何失败后保留当前身份重试，身份完全丢失时另开明确记录的离线会话；将历史宽度时间增长项作为标记而非预览中止原因，仍保留初始宽度不确定度、当前质量、目标支持和歧义检查。不能将预览覆盖率当作连续驾驶通过率。车辆入口及原停车/重新启用流程不变。

输出：`frames.jsonl`、`events.json`、`summary.json` 和 `preview.avi`。33–40秒的用户已确认邻道通过既有缓存几何对应排除，不能将这个人工约束视为通用自动选道成果。本次本地结果见 `results/three_stage_v1/`；前文37.134秒结论是旧版锁定回放记录，不能代表后续所有片段都无法生成中心线。

真实RGB已启用 `fragment_identity_handoff`：只有反复共同观测且唯一的远段才能接续消失近段的边界角色，不连接断口或输出外推点。可用 `--start 45 --end 70` 做局部检查；65秒对比图与完整结果见 `results/identity65_v1/report.md`。实车故障锁定/启用流程不变。


### Real-data transition regression (local, 2026-11-18)

The e4 profile now retains verified short paint fragments, checks returning boundaries with local normal widths, and separates observation timestamps/identity confirmation from path validity. ENet v3, nominal e4 projection, Pure Pursuit, the 7 m single-side target cap and Highbay profiles are unchanged. Driving latches remain active.

The final 3,870-frame **offline preview** produced 3,211 centerlines/candidate commands (previously 3,078). There are 4 newly rejected frames inside the inspected windows and remaining gaps; this is not a vehicle or continuous-driving acceptance test. Review local `results/transition_v1/index.html` and `report.md`; `final/` is the final run, while `full/` and `full_corrected/` are retained failed experiments. Artifacts are Git-ignored.

### Local finite-path-cache experiment (not enabled on the vehicle)

`results/path_cache_v1/index.html` compares two short recorded-RGB clips. The
known near-boundary remnant recovery is in `core.py`; `finite_path.py` provides
an opt-in, motion-compensated path cache and calls the unchanged Pure Pursuit.
`python scripts/analyze_path_cache.py` runs the local experiment in Jazzy using
the existing ignored baseline artifacts and read-only bag. See
`results/path_cache_v1/report.md` for required inputs and limits.

Motion feedback is **hypothetical**, not measured in this bag. No actuator
messages are published, the ROS vehicle node does not enable the cache, and
65.803 s still stops when the archived ambiguity is preserved. Neither local
preview continuity nor the new cache establishes a physical closed-loop pass.


### 统一身份与观测交接（离线实验）

```bash
python scripts/analyze_real_stages.py --handoff --output results/handoff_review_02
```

在已有 Jazzy 容器、项目根目录运行；输出目录必须不存在。保留 v3、e4 几何、Pure Pursuit 和 Highbay 配置。短时缺失或远处候选竞争不直接清空已验证近场路径；新边界须通过当前几何及连续观测交接，近场明确冲突仍停止。缓存最多 1.5 秒/0.3 米、目标最多 7 米，预测不刷新观测时间。

这是显式选择的**离线实验入口**，尚未接入实车节点。现有 bag 缺少车速/实际转角，实验用上一请求构造 20 Hz 假设反馈；它不能验证运动补偿的真实误差或闭环连续驾驶。实车接口需要实测反馈与误差界，不能照搬假设反馈。完整回放与图像对比见 `results/unified_handoff_v1/report.md`。


112.704秒的近侧交接修复：当两个已知单侧目标的近远支持分离，且较近侧通过连续身份、质量和当前冲突检查时，离线交接层可用当前观测更新路径。新增 `path_update_state` / `stop_required` 区分暂时无法更新与停车。781帧局部对比、真实近场冲突合成反例及限制见 `results/handoff112_fix_v1/report.md`；未重跑完整回放，未启用实车续行。

## Night augmentation candidates and reproducibility

The revised dark ambient / dual-headlamp preview was approved on 2026-11-18. All 500 selected originals now have assistant-reviewed labels, frozen into a grouped 350/75/75 split. Training uses 350 original + 350 online low-light examples per epoch; the original and 23%-expanded SimpleENet both start from MP1 epoch20.

See the local [experiment report](results/night_v1/report.md), [500-label audit](results/night_v1/dataset/label_audit/index.html), and [approved lighting preview](results/night_v1/dataset/review/index.html). `dataset/manifest.json` is the authoritative frozen training manifest. No real-night images were collected, so synthetic results do not establish real-night driving readiness.

Both candidates completed early stopping (small: best epoch 28; expanded: best epoch 30). The validation-selected expanded model improves synthetic light/medium IoU by 15.23 percentage points over v3, but loses 2.80 points of original-image recall and slightly increases medium-light background false positives. It remains an optional observation model, not the default. The original-image handoff regression near 112.94 s has now been repaired in the offline handoff layer (see the release results above); this does not change the segmentation metrics or establish real-night performance. The earlier dynamic-low-light replay has not been rerun with this handoff fix.

Training does not alter default v3, lane geometry, Pure Pursuit or Highbay model selection. Both deployment checkpoints and metadata are included under `models/`; see [model selection and limitations](models/README.md). Training tools `scripts/prepare_night_data.py` and `scripts/train_night_lane_net.py` are included, but require the local bag, reviewed labels, frozen manifest and original MP1 checkpoint. A fresh clone alone cannot reproduce training. `results/night_v1/compat/` contains untrained probes; `train_small/` and `train_expanded/` contain local training artifacts. Data, labels, reports, videos and environments are excluded from Git.
