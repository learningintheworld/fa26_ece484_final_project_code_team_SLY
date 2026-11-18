# GEM e4 实车操作

更新：2026-11-18。个人公开仓库目录：`~/sly_public`；车载 Ubuntu 22.04 / ROS Humble。默认仍为 ENet v3。最新物理白线身份层和1.5秒路径缓存属于离线试验，下面的实车入口不会启用它们；实车自主车道跟随尚未验收。

各终端的**快捷脚本与原生命令二选一，不要同时执行**。已有相机、DBW或同名节点正在运行时不重复启动。遇到问题，把 [AI排障交接文档](E4_AI_HANDOFF_ZH.md) 和完整终端输出交给网页版ChatGPT。

## 1. 拉取与构建

首次部署（保留旧项目，不覆盖现场修改）：

```bash
git clone https://github.com/learningintheworld/fa26_ece484_final_project_code_team_SLY.git ~/sly_public
cd ~/sly_public
bash scripts/e4.sh setup
```

`setup` 检查现有PACMod消息和真实GPU前向，成功后构建；不安装或升级Torch/CUDA、不新建venv、不启动车辆。v3权重已在仓库内。

后续更新：

```bash
cd ~/sly_public
git pull --ff-only
bash scripts/e4.sh build
```

有本地修改冲突时保留现场修改，不强制reset。环境变化后重跑`setup`；`build`本身不做GPU验收。

若需直接执行构建，在新的Bash终端中只加载Humble和官方驱动，不加载本项目旧安装层：

```bash
cd ~/sly_public
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/humble/setup.bash
source "${SLY_DRIVER_WS:-$HOME/gem_ws}/install/setup.bash"
/usr/bin/colcon build --base-paths src --packages-select sly_lane_following --symlink-install
```

## 2. 每个操作终端：先加载环境

**使用快捷脚本**只需`cd ~/sly_public`，脚本自动加载环境。

**使用以下原生命令**，每个新Bash终端先完整执行这一段（项目已构建）：

```bash
cd ~/sly_public
export SLY_RUN_PYTHON="$(command -v "${SLY_PYTHON:-python3}")"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source /opt/ros/humble/setup.bash
source "${SLY_DRIVER_WS:-$HOME/gem_ws}/install/setup.bash"
source ./install/local_setup.bash
export VEHICLE_NAME=e4
```

官方驱动默认`~/gem_ws`。路径不同时，在上述操作前设置`SLY_DRIVER_WS`为车上真实目录；使用已有Python环境时先激活，或设置`SLY_PYTHON`为其解释器绝对路径，必须兼容Humble/Python3.10。`SLY_RUN_PYTHON`供本指南原生命令使用。沿用车辆通信域，不套用笔记本回放的域设置；不复制笔记本venv、build或install。

## 3. 项目运行：按终端启动

### 终端1：前置OAK相机

快捷：`bash scripts/e4.sh camera`

原生命令：

```bash
ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=oak_link
```

官方launch可能同时发布深度/点云，**本项目只读取RGB与CameraInfo**，不是RGB-D算法。

### 终端2：PACMod驱动与车辆模型

快捷：`bash scripts/e4.sh dbw`

原生命令：

```bash
ros2 launch basic_launch dbw_only.launch.py
```

该launch同时启动车辆描述/TF与PACMod驱动，不必再启动`robot_description.launch.py`。

### 终端3：手柄输入

快捷：`bash scripts/e4.sh joy`

原生命令：

```bash
ros2 run joy joy_node
```

这里只发布`/joy`，供SLY读取使能/接管输入，**不是官方手柄驾驶控制器**。SLY主动控制时不能同时运行`dbw_joystick.launch.py`或`pacmod2_game_control`。

### 终端4：GPU感知与Pure Pursuit

快捷：`bash scripts/e4.sh vision`

原生命令（GPU预检成功才启动；参数与脚本一致）：

```bash
"$SLY_RUN_PYTHON" scripts/e4_check.py gpu && \
"$SLY_RUN_PYTHON" -m sly_lane_following.node --ros-args \
  -p use_sim_time:=false -p reliable_input:=false \
  -p config:="$PWD/src/sly_lane_following/config/real_enet.yaml" \
  -p control_config:="$PWD/src/sly_lane_following/config/e4_pure_pursuit.yaml" \
  -p image_topic:=/oak/rgb/image_raw \
  -p camera_info_topic:=/oak/rgb/camera_info
```

产生识别图、路径和控制请求，不直接发布执行器命令。GPU失败时不自动转CPU。

仅停车观察/排障需要CPU时，先结束GPU节点。快捷：`bash scripts/e4.sh vision-cpu`。原生命令使用临时配置，不修改仓库文件：

```bash
SLY_CPU_CONFIG="$(mktemp /tmp/sly_e4_cpu.XXXXXX.yaml)"
sed 's/^enet_device:.*/enet_device: cpu/' src/sly_lane_following/config/real_enet.yaml > "$SLY_CPU_CONFIG"
"$SLY_RUN_PYTHON" -m sly_lane_following.node --ros-args \
  -p use_sim_time:=false -p reliable_input:=false \
  -p config:="$SLY_CPU_CONFIG" \
  -p control_config:="$PWD/src/sly_lane_following/config/e4_pure_pursuit.yaml" \
  -p image_topic:=/oak/rgb/image_raw \
  -p camera_info_topic:=/oak/rgb/camera_info
rm -f -- "$SLY_CPU_CONFIG"
```

CPU实时驾驶性能未验证。若首次`setup`仅因GPU失败未构建，可先`build`供CPU观察。

### 终端5：只观察执行器适配器

快捷：`bash scripts/e4.sh observe`

原生命令（先确认Python能加载ROS/PACMod接口）：

```bash
"$SLY_RUN_PYTHON" scripts/e4_check.py ros && \
"$SLY_RUN_PYTHON" -c 'from sly_lane_following.adapter import main; main()' --ros-args \
  -p platform:=e4 -p dry_run:=true \
  -p hardware_config:="$PWD/src/sly_lane_following/config/e4_adapter.yaml"
```

读取请求与车辆反馈，不发布执行器命令。

### 终端6：图像查看

快捷：`bash scripts/e4.sh view`

```bash
ros2 run rqt_image_view rqt_image_view /sly/lane/debug_image
```

如需看原图，在另一个已加载环境的终端运行：

```bash
ros2 run rqt_image_view rqt_image_view /oak/rgb/image_raw
```

### 终端7：项目运行录制

快捷：`bash scripts/e4.sh record`

原生命令（与脚本相同；Ctrl+C正常结束并保存）：

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

## 4. 按需采集：Ouster、GNSS与Livox

这些传感器**不参与当前RGB车道跟随控制**。按车辆规程上电，下面每项单独一个新终端，先执行第2节环境加载；读取车上官方e4配置，不手填IP或标定参数。

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

只采集前置RGB、Ouster及GNSS时，可在录制终端用下面命令**替代**第3节的项目录制。默认不录深度、OAK点云、角部相机或Livox：

```bash
mkdir -p results/e4
ros2 bag record -s sqlite3 --max-bag-size 1073741824 \
  -o "results/e4/sensors_$(date +%Y%m%d_%H%M%S)" \
  -e '^(/oak/rgb/(image_raw|camera_info)|/ouster/(points.*|imu|metadata)|/tf|/tf_static|/joint_states|/joy|/(.*/)?(navsatfix|twist_ins|twist_gnss|insnavgeod|pvtgeodetic))$'
```

GNSS话题按车载配置发布，正则允许命名空间前缀；没有发布的消息不会被补录。1GiB只限制单个分片，不限制总数据量。需要Livox时，在录制正则的最末`)$`之前加入`|/livox/(lidar|imu)`。

## 5. 纯手柄驾驶采集模式（可选，替代SLY主动控制）

只在车辆停稳并按助教规程退出SLY主动控制后切换。先结束SLY适配器、独立DBW与独立joy节点，再在已加载环境的新终端运行：

```bash
ros2 launch basic_launch dbw_joystick.launch.py
```

此官方launch已包含PACMod驱动、joy输入和手柄控制器；不再同时启动终端2、3或SLY主动适配器。使能、换挡与接管按助教培训操作。

如果需要记录模型/TF，在另一个终端启动一次（本模式的手柄launch不包含它）：

```bash
ros2 launch basic_launch robot_description.launch.py
```

## 6. 现场验证后才启用SLY闭环

必须先确认感知实时性、车辆反馈、转向方向、制动、换挡及手柄按钮，按实测填写`e4_adapter.yaml`。未验证参数继续保持为空；当前实车速度上限0.2m/s。`setup`成功不等于车辆可以自动驾驶。

**终端5：**退出`observe`，换成主动适配器。快捷：`bash scripts/e4.sh drive`。原生命令：

```bash
"$SLY_RUN_PYTHON" scripts/e4_check.py ros && \
"$SLY_RUN_PYTHON" -c 'from sly_lane_following.adapter import main; main()' --ros-args \
  -p platform:=e4 -p dry_run:=false \
  -p hardware_config:="$PWD/src/sly_lane_following/config/e4_adapter.yaml"
```

**终端8：**车辆停稳、空挡、PACMod禁用，驾驶员按住已确认的使能按钮后启用。快捷：`bash scripts/e4.sh enable`。原生命令：

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: true}'
```

正常停车。快捷：`bash scripts/e4.sh stop`。原生命令：

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: false}'
```

保持主动适配器运行，直到完成停车、空挡和禁用流程；**请求发送成功不等于车辆已经停稳**。确认安全后先在录制终端Ctrl+C保存，再结束其他进程。停进程、断开PACMod或Ctrl+C均不能代替制动。

仅在停稳、确认车道位置后重置感知；不会自动恢复驾驶。快捷：`bash scripts/e4.sh reset`。原生命令：

```bash
ros2 service call /sly/lane/reset std_srvs/srv/Trigger '{}'
```

以上服务让适配器按状态机处理启停、制动与换挡；不要额外向`/pacmod/*_cmd`手工发布猜测的油门、制动或挡位数值。

## 7. 排障报告

DBW已运行时，在另一个终端执行：

```bash
cd ~/sly_public
bash scripts/e4.sh diagnose
```

已加载第2节环境时，也可直接执行：

```bash
"$SLY_RUN_PYTHON" scripts/e4_diagnose.py
```

将`results/e4_diagnostics/时间/report.json`与DBW原始日志交给排障人员。只收集信息、不发布车辆命令；“采集完成”不代表全部检查通过。环境不完整时优先使用`e4.sh diagnose`，它会尽量记录加载失败。

原生命令核对来源：[项目启动脚本](scripts/e4.sh)、[官方basic_launch固定版本](https://github.com/UIUC-Robotics/gem_ws/tree/5afbd638d182b40d5300a38bd42e48df969fb42c/src/basic_launch/launch)。这是源码核对，不是本次现场运行验证；以车上已安装的官方工作空间为准。
