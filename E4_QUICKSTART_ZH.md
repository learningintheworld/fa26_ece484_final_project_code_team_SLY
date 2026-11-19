# GEM e4 实车操作

观察模式不要求运行joy：未按手柄或缺少手柄消息仅记录为 `operator_permission`（HELD/RELEASED/INPUT_STALE），不再覆盖主要故障。`operator_permission_required: False` 表示仅观察；主动模式仍要求持续手柄许可，未提供无手柄主动驾驶模式。CAN和反馈故障检查不变。

本版 `bash scripts/e4.sh drive` 在硬件配置未验证时进入 `UNVERIFIED_OBSERVE`：进程继续接收反馈，状态显示 `dry_run: True`，不创建执行器命令发布者，`enable` 被拒绝。中心线恢复或运行中修改YAML均不会自动转为主动模式；现场验证完成后需重新启动。原生主动入口仍默认拒绝未验证配置。

更新：2026-11-18。个人公开仓库目录：`~/sly_public`；车载 Ubuntu 22.04 / ROS Humble。默认仍为 ENet v3；可选 `night` 扩容候选（仅验证合成低光，未验证真实夜间驾驶）。本版更新离线分段质量与身份交接；有限缓存已接入Highbay仿真，但下面的实车入口尚未接入这套交接与续行策略。实车自主车道跟随尚未验收。

各终端的**快捷脚本与原生命令二选一，不要同时执行**。已有相机、DBW或同名节点正在运行时不重复启动。遇到问题，把 [AI排障交接文档](E4_AI_HANDOFF_ZH.md) 和完整终端输出交给网页版ChatGPT。

## 1. 拉取与构建

首次部署（保留旧项目，不覆盖现场修改）：

```bash
git clone https://github.com/learningintheworld/fa26_ece484_final_project_code_team_SLY.git ~/sly_public
cd ~/sly_public
bash scripts/e4.sh setup
```

`setup` 检查现有PACMod消息和真实GPU前向，成功后构建；不安装或升级Torch/CUDA、不新建venv、不启动车辆。v3与night权重均随普通Git文件下载，无需Git LFS或额外复制；night的同名JSON也必须保留。

后续更新：

```bash
cd ~/sly_public
git pull --ff-only
bash scripts/e4.sh build
```

更新后继续使用下方相同短命令；不会自动切换模型或启用驾驶。有本地修改冲突时保留现场修改，不强制reset。环境变化后重跑`setup`；`build`本身不做GPU验收。

只重新构建本地代码（不拉取更新）：

```bash
cd ~/sly_public
bash scripts/e4.sh build
```

构建时无需手动执行`unset`、`source`或`colcon`。脚本会清理旧ROS工作空间变量，加载Humble与官方驱动，再只构建本项目；不加载本项目旧安装层，不启动传感器或车辆控制。

## 2. 日常启动：每个终端复制一段

以下短命令自动加载环境，无需手动`source`。官方驱动默认`~/gem_ws`；如不同，先在各终端设置`SLY_DRIVER_WS`为实际目录。使用车载已有Python环境时先激活，或设置`SLY_PYTHON`为解释器绝对路径。

### 终端1：前置相机

```bash
cd ~/sly_public && bash scripts/e4.sh camera
```

### 终端2：PACMod驱动与车辆TF

```bash
cd ~/sly_public && bash scripts/e4.sh dbw
```

### 终端3：手柄输入

```bash
cd ~/sly_public && bash scripts/e4.sh joy
```

只读取手柄，不是官方手柄驾驶程序；不能与`pacmod2_game_control`同时控制执行器。

### 终端4：GPU感知与Pure Pursuit

```bash
cd ~/sly_public && bash scripts/e4.sh vision
```

默认命令使用v3。选择夜间扩容候选时，改为下面一条（**二选一**）：

```bash
cd ~/sly_public && bash scripts/e4.sh vision-night
```

GPU预检会实际加载所选模型并做一次前向，然后启动感知，只生成控制请求。启动日志会显示`v3`或`night`。night不做在线暗化，也不改变ROI、中心线或Pure Pursuit参数。

切换模型前先执行`stop`并确认车辆停稳，结束原感知节点（Ctrl+C），再启动另一模型；不要同时运行两个感知节点。重启会清空感知历史，不会自动启用驾驶。先用`observe`和`view`查看实际夜间图像。

仅停车观察/排障时，可结束GPU节点并改用v3 CPU：

```bash
cd ~/sly_public && bash scripts/e4.sh vision-cpu
```

night CPU排障替代命令：

```bash
cd ~/sly_public && bash scripts/e4.sh vision-night-cpu
```

CPU驾驶性能未验证；若`setup`仅因GPU失败未构建，可先`build`供CPU观察。两种模型和已知限制见[权重说明](models/README.md)。

### 终端5：观察车辆反馈，不控制执行器

```bash
cd ~/sly_public && bash scripts/e4.sh observe
```

### 终端6：查看识别图

```bash
cd ~/sly_public && bash scripts/e4.sh view
```

### 终端7：录制图像、状态与控制请求

```bash
cd ~/sly_public && bash scripts/e4.sh record
```

Ctrl+C结束录制并保存到`results/e4/`。bag包含执行器命令，不在连接实车的通信域中全包回放。

## 3. 现场验证后的启用与停止

先确认感知实时性、车辆反馈、转向、制动、换挡与手柄按钮，再按实测填写`e4_adapter.yaml`。未验证参数保持为空；当前实车速度上限0.2m/s。

**终端5：**结束`observe`，改为主动适配器：

```bash
cd ~/sly_public && bash scripts/e4.sh drive
```

**终端8：**车辆停稳、空挡、PACMod禁用，驾驶员按住已确认的使能按钮后启用：

```bash
cd ~/sly_public && bash scripts/e4.sh enable
```

正常停车：

```bash
cd ~/sly_public && bash scripts/e4.sh stop
```

保持适配器运行，等车辆实际停稳并完成空挡、禁用流程，再结束录制与其他进程。Ctrl+C或断开PACMod不能代替制动。

停稳、确认车道位置后，必要时重置感知（不会自动启用驾驶）：

```bash
cd ~/sly_public && bash scripts/e4.sh reset
```

## 4. 排障与可选传感器

DBW运行时，在新终端收集诊断报告：

```bash
cd ~/sly_public && bash scripts/e4.sh diagnose
```

诊断night候选时改用：

```bash
cd ~/sly_public && bash scripts/e4.sh diagnose-night
```

提供`results/e4_diagnostics/时间/report.json`与DBW日志；报告包含选定模型、权重哈希及其CPU/GPU前向结果。采集完成不等于检查通过。

Ouster、GNSS、Livox和纯手柄采集的原生命令见下方参考。它们不参与当前RGB车道跟随，无需为运行项目额外开启全部传感器。

## 5. 原生命令参考

日常优先使用上面的短命令；以下与快捷脚本二选一。先展开并执行公共环境，再选择对应命令。

<details>
<summary>展开：公共环境、传感器、感知与控制命令</summary>

### 公共环境：每个使用原生命令的新Bash终端执行一次

```bash
cd ~/sly_public
export SLY_RUN_PYTHON="$(command -v "${SLY_PYTHON:-python3}")"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/humble/setup.bash
source "${SLY_DRIVER_WS:-$HOME/gem_ws}/install/setup.bash"
source ./install/local_setup.bash
export VEHICLE_NAME=e4
SLY_CONFIG="$PWD/src/sly_lane_following/config"
```

项目须已构建。沿用车载ROS通信域；不加载Jazzy或作业工作空间。下文用`SLY_CONFIG`缩短配置路径，不省略必需参数。

### 终端1：前置OAK相机

```bash
ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=oak_link
```

官方launch可能同时发布深度/点云，**本项目只读取RGB与CameraInfo**，不是RGB-D算法。

### 终端2：PACMod驱动与车辆模型

```bash
ros2 launch basic_launch dbw_only.launch.py
```

该launch同时启动车辆描述/TF与PACMod驱动，不必再启动`robot_description.launch.py`。

### 终端3：手柄输入

```bash
ros2 run joy joy_node
```

这里只发布`/joy`，供SLY读取使能/接管输入，**不是官方手柄驾驶控制器**。SLY主动控制时不能同时运行`dbw_joystick.launch.py`或`pacmod2_game_control`。

### 终端4：GPU感知与Pure Pursuit

先选择一组模型变量（二选一）：

```bash
SLY_MODEL=v3
SLY_VISION_CONFIG="$SLY_CONFIG/real_enet.yaml"
```

或夜间扩容候选：

```bash
SLY_MODEL=night
SLY_VISION_CONFIG="$SLY_CONFIG/real_enet_night.yaml"
```

然后GPU预检成功才启动：

```bash
"$SLY_RUN_PYTHON" scripts/e4_check.py gpu "$SLY_MODEL" && \
"$SLY_RUN_PYTHON" -m sly_lane_following.node --ros-args \
  -p use_sim_time:=false -p reliable_input:=false \
  -p config:="$SLY_VISION_CONFIG" \
  -p control_config:="$SLY_CONFIG/e4_pure_pursuit.yaml" \
  -p image_topic:=/oak/rgb/image_raw \
  -p camera_info_topic:=/oak/rgb/camera_info
```

产生识别图、路径和控制请求，不直接发布执行器命令。GPU失败时不自动转CPU。

CPU排障替代GPU节点，使用临时配置：

```bash
SLY_CPU_CONFIG="$(mktemp /tmp/sly_e4_cpu.XXXXXX.yaml)"
sed 's/^enet_device:.*/enet_device: cpu/' "$SLY_VISION_CONFIG" > "$SLY_CPU_CONFIG"
"$SLY_RUN_PYTHON" -m sly_lane_following.node --ros-args \
  -p use_sim_time:=false -p reliable_input:=false \
  -p config:="$SLY_CPU_CONFIG" \
  -p control_config:="$SLY_CONFIG/e4_pure_pursuit.yaml" \
  -p image_topic:=/oak/rgb/image_raw \
  -p camera_info_topic:=/oak/rgb/camera_info
rm -f -- "$SLY_CPU_CONFIG"
```

CPU实时驾驶性能未验证。若首次`setup`仅因GPU失败未构建，可先`build`供CPU观察。

### 终端5：只观察执行器适配器

先加载ROS/PACMod接口：

```bash
"$SLY_RUN_PYTHON" scripts/e4_check.py ros && \
"$SLY_RUN_PYTHON" -c 'from sly_lane_following.adapter import main; main()' --ros-args \
  -p platform:=e4 -p dry_run:=true \
  -p hardware_config:="$SLY_CONFIG/e4_adapter.yaml"
```

读取请求与车辆反馈，不发布执行器命令。

### 终端6：图像查看

```bash
ros2 run rqt_image_view rqt_image_view /sly/lane/debug_image
```

如需看原图，在另一个已加载环境的终端运行：

```bash
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw
```

### 终端7：项目运行录制

Ctrl+C结束并保存：

```bash
mkdir -p results/e4
ros2 bag record -o "results/e4/run_$(date +%Y%m%d_%H%M%S)" \
  /oak/rgb/image_raw /oak/rgb/camera_info \
  /sly/lane/status /sly/lane/local_path /sly/control/request /sly/control/status \
  /pacmod/vehicle_speed_rpt /pacmod/global_rpt /pacmod/steering_rpt \
  /pacmod/brake_rpt /pacmod/shift_rpt /joy \
  /pacmod/global_cmd /pacmod/accel_cmd /pacmod/brake_cmd /pacmod/steering_cmd /pacmod/shift_cmd
```

只记录实际发布且订阅成功的话题。此bag包含控制命令，不能在接通实车执行器的通信域中全包回放。

### 可选传感器：Ouster、GNSS与Livox

这些传感器**不参与当前RGB车道跟随控制**。按车辆规程上电，下面每项单独一个新终端，先执行本参考的公共环境；读取车上官方e4配置，不手填IP或标定参数。

GNSS/INS：

```bash
ros2 launch basic_launch gnss.launch.py
```

顶部Ouster LiDAR（官方launch自动完成生命周期configure/activate）：

```bash
ros2 launch basic_launch ouster_driver.launch.py
```

前部Livox LiDAR（仅需要时启动）：

```bash
ros2 launch basic_launch livox_lidar.launch.py
```

不要同时运行`ros2 launch basic_launch sensor_init.launch.py`；它会联动启动多台传感器和角部相机，可能与以上独立进程重复。

只采集前置RGB、Ouster及GNSS时，可在录制终端用下面命令**替代**上面的项目录制。默认不录深度、OAK点云、角部相机或Livox：

```bash
mkdir -p results/e4
ros2 bag record -s sqlite3 --max-bag-size 1073741824 \
  -o "results/e4/sensors_$(date +%Y%m%d_%H%M%S)" \
  -e '^(/oak/rgb/(image_raw|camera_info)|/ouster/(points.*|imu|metadata)|/tf|/tf_static|/joint_states|/joy|/(.*/)?(navsatfix|twist_ins|twist_gnss|insnavgeod|pvtgeodetic))$'
```

GNSS话题按车载配置发布，正则允许命名空间前缀；没有发布的消息不会被补录。1GiB只限制单个分片，不限制总数据量。需要Livox时，在录制正则的最末`)$`之前加入`|/livox/(lidar|imu)`。

### 可选：官方手柄驾驶（替代SLY主动控制）

只在车辆停稳并按助教规程退出SLY主动控制后切换。先结束SLY适配器、独立DBW与独立joy节点，再在已加载环境的新终端运行：

```bash
ros2 launch basic_launch dbw_joystick.launch.py
```

此官方launch已包含PACMod驱动、joy输入和手柄控制器；不再同时启动终端2、3或SLY主动适配器。使能、换挡与接管按助教培训操作。

如果需要记录模型/TF，在另一个终端启动一次（本模式的手柄launch不包含它）：

```bash
ros2 launch basic_launch robot_description.launch.py
```

### 主动适配器（替换终端5的observe）

仅在第3节现场条件满足后使用：

```bash
"$SLY_RUN_PYTHON" scripts/e4_check.py ros && \
"$SLY_RUN_PYTHON" -c 'from sly_lane_following.adapter import main; main()' --ros-args \
  -p platform:=e4 -p dry_run:=false -p hardware_config:="$SLY_CONFIG/e4_adapter.yaml"
```

### 终端8：启用、停止或重置（按需单独执行）

启用：

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: true}'
```

停车：

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: false}'
```

停稳后重置感知：

```bash
ros2 service call /sly/lane/reset std_srvs/srv/Trigger '{}'
```

启停服务由适配器处理制动和换挡；不手工发布猜测的`/pacmod/*_cmd`数值。正常停止流程见第3节。

### 诊断

```bash
"$SLY_RUN_PYTHON" scripts/e4_diagnose.py --model v3
```

night模型将上面`--model v3`改为`--model night`。环境加载失败时改用`bash scripts/e4.sh diagnose`或`diagnose-night`，它能尽量记录缺失项。

</details>

命令核对来源：[项目启动脚本](scripts/e4.sh)、[官方basic_launch固定版本](https://github.com/UIUC-Robotics/gem_ws/tree/5afbd638d182b40d5300a38bd42e48df969fb42c/src/basic_launch/launch)。已核对源码，未在本次修改中执行实车命令。
