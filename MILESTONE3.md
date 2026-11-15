# Milestone 3：图像车道跟随操作

目标是实车根据前置相机完成低速车道跟随。目前只有本地开发和测试结果，**尚未完成实车验收**；最新 Highbay 降速/方向反馈试验仍未通过完整急弯，见 [新报告及录像](results/highbay_control/report.md)；[前次结果](results/milestone3/report.md) 保留。缺少旧 bag 的相机→车体 TF 不阻塞此模式；本实现不生成米制路径，也不使用 Pure Pursuit。

## 1. 电脑端：已有 Jazzy Docker

所有终端在宿主机进入同一仓库：

```bash
cd /home/xinweiyu/Workspace/ECE484_Program/fa26_ece484_final_project_code_team_SLY
```

首次构建：

```bash
./scripts/dev.sh up
./scripts/dev.sh build
```

不要同时运行 bag player 和仿真。它们使用同一 ROS 域 84 和 `/clock`。

| 终端 | 直接执行 |
|---|---|
| 1：Highbay 仿真 | `./scripts/dev.sh sim` |
| 2：感知及图像控制请求 | `./scripts/dev.sh lane-sim` |
| 3：仿真执行器出口，初始禁用 | `./scripts/dev.sh adapter` |
| 4：实时调试图 | `./scripts/dev.sh lane-view` |
| 5：明确启用 | `./scripts/dev.sh enable` |
| 5：停止并锁定 | `./scripts/dev.sh stop` |

仿真默认无 GUI，终端 4 显示真实模拟相机与识别结果。需要 Gazebo GUI 时，在终端 1 使用：

```bash
./scripts/dev.sh sim 'gz_args:=-r -v1 '
```

输入无效、过期、反馈中断、出现第二个执行器发布者时禁用控制。恢复图像不会自动恢复驾驶。排除原因、车辆停止并确认仍对准正确车道后，才可运行：

```bash
./scripts/dev.sh lane-reset
./scripts/dev.sh enable
```

重置感知会重新选择当前走廊，不应在车辆移动中执行。结束顺序：先 `stop`，看到车速归零，再在终端 2、3、1 分别 Ctrl+C。无需停止整个 Docker。

### 仅仿真：复现速度与方向参数对照

这些是失败基线的复现入口，不是已通过参数。先仅启动上表终端 1（仿真）和终端 3（适配器），不要运行 `lane-sim` 或自行 `enable`；实验器负责新建感知进程、静止重置、明确启用及结束停车。

```bash
./scripts/dev.sh exec python scripts/highbay_experiment.py speed --output results/highbay_speed_review
./scripts/dev.sh exec python scripts/highbay_experiment.py angle --speed 0.2 --output results/highbay_angle_review
```

两条命令顺序执行，每次使用新输出目录。录像、逐帧日志、参数与源码快照会一起保存。赛道真值只用于诊断与验收，不参与转向。终止任意试验可在另一终端执行 `./scripts/dev.sh stop`，随后按既定顺序关闭进程。

## 2. 实车：先只观察

车辆使用已有 Ubuntu 22.04 / Humble。仅把本仓库 `src/sly_lane_following` **源码**复制到车辆 `~/sly_ws/src/sly_lane_following`；不复制 Jazzy 的 `install`、`build` 或 Python 虚拟环境。

车辆构建终端：

```bash
source /opt/ros/humble/setup.bash
source ~/gem_ws/install/local_setup.bash
cd ~/sly_ws
colcon build --base-paths src --packages-select sly_lane_following --symlink-install
```

每个车辆终端先执行以下公共环境（与车辆现有传感器/驱动使用同一通信域；以下统一为 84）：

```bash
source /opt/ros/humble/setup.bash
source ~/gem_ws/install/local_setup.bash
source ~/sly_ws/install/local_setup.bash
export ROS_DOMAIN_ID=84
export ROS_LOCALHOST_ONLY=1
```

若车辆已启动相机、PACMod 或 joy，使用现有进程，不重复启动。需要按本指南统一启动时：

**终端 1：前置 OAK。** 本地官方工作空间的前置相机入口如下；USB 相机身份以车辆现有配置为准。

```bash
ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=oak_link
```

**终端 2：PACMod 驱动，不启动 Joystick 执行器控制程序。**

```bash
ros2 launch basic_launch dbw_only.launch.py
```

**终端 3：Joystick 输入。** 此进程只发布手柄输入；本适配器使用持续按住的使能按钮（默认索引 4，必须与现场手柄对应）。

```bash
ros2 run joy joy_node
```

SLY 控制时不能同时运行 `dbw_joystick.launch.py` / `pacmod2_game_control`；它们也会向 PACMod 发布命令。检测到竞争发布者时 SLY 退出命令发布，交由人工接管，不会用“另一条零命令”与其争用。

**终端 4：实时边界和控制请求。**

```bash
ros2 run sly_lane_following lane_node --ros-args \
  -p use_sim_time:=false \
  -p reliable_input:=false \
  -p control_config:=$HOME/sly_ws/src/sly_lane_following/config/e4_control.yaml
```

输入为 `/oak/rgb/image_raw` 和 `/oak/rgb/camera_info`，要求帧 ID、尺寸和时间戳匹配。实车订阅使用 Best Effort；在车辆重新验证消息连续性。车辆实际 topic 不同可用 `image_topic`、`camera_info_topic` 参数指定。

**终端 5：PACMod 只观察。不会创建执行器发布者。**

```bash
ros2 run sly_lane_following lane_adapter --ros-args \
  -p platform:=e4 -p dry_run:=true \
  -p hardware_config:=$HOME/sly_ws/src/sly_lane_following/config/e4_adapter.yaml
```

**终端 6：显示。**

```bash
ros2 run rqt_image_view rqt_image_view /sly/lane/debug_image
```

`e4_control.yaml` 的 `reference_verified: false` 会让请求保持零，原因是 `IMAGE_REFERENCE_REQUIRED`，而不是缺 TF。将车辆沿直线车道居中、朝向正确后，测量近/远采样行的中心线横坐标除以图像宽度，填写 `reference_near`、`reference_far`。行位置由 `near_row`、`far_row` 指定；不是默认假定图像正中央正确。确认安装方向及参考后，设 `reference_verified: true` 并重启终端 4，仍保持终端 5 `dry_run:=true`，检查左右偏差对应的转向符号及候选速度。

e4 控制增益目前只是未调试的起始配置，不能照搬仿真配置。仿真使用 e2 模型；本轮没有验证其对 e4 的迁移效果。

## 3. 实车低速测试的启用与停止

**本节仅在感知/仿真达到验收条件，并按助教规程完成现场参数验证后使用。当前版本尚未达到该状态。**

`e4_adapter.yaml` 默认 `hardware_verified: false`，`stop_brake`、`max_throttle`、`speed_kp` 未填写，因此 `dry_run:=false` 会拒绝启动。现场须验证前轮→方向盘映射、正负号、实测速度、前进档反馈、制动比例和停稳判断，填写相应参数再标记验证通过。不要复制 `results` 中模拟消息测试用的 profile。

转向映射来源：`gem_ws/src/vehicle_drivers/gem_odometry_control/gem_odometry_control/pure_pursuit_odom.py` 的 `front2steer`；这里只移植其转换式，未使用 Pure Pursuit 控制。请求是前轮弧度，PACMod 命令是方向盘弧度。

按现场规程准备完毕后，停止终端 5 的只观察适配器，再启动唯一的命令出口：

```bash
ros2 run sly_lane_following lane_adapter --ros-args \
  -p platform:=e4 -p dry_run:=false \
  -p hardware_config:=$HOME/sly_ws/src/sly_lane_following/config/e4_adapter.yaml
```

驾驶员保持手柄使能按钮按下，软件操作员在另一个终端明确启用：

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: true}'
```

车辆必须停稳、请求与 PACMod 各反馈新鲜且无接管/故障。适配器先请求前进档及使能，收到对应反馈后才允许油门。首轮上车目标为转向/制动验证、直线，再逐步加入弯道，保留运行日志和录像。

正常停止：**保持适配器运行，先请求停车。**

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: false}'
```

适配器请求制动，只有有效新车速反馈确认停稳后才禁用 PACMod。驾驶员确认车已停住后，才 Ctrl+C 退出。退出程序、停发命令或禁用 PACMod 本身不能当作车辆制动。人工接管依照车辆培训规程；适配器接收到 override 后释放控制，输入恢复后不会擅自重新启用。

## 4. 当前限制

- 只要求当前图像中有充分双边界，不凭空补线；急弯单边消失会停车。
- 记忆丢失会保持禁用；旧 bag 中持续驾驶并不能模拟这种停车后的车辆画面。
- 图像中心线没有米制含义，`/sly/lane/local_path` 始终为空。
- 本地测试证明接口行为，不证明实车制动力、延迟、轮胎附着或转向映射正确。
- 所有改动和结果保留在本地，未提交 GitHub。
