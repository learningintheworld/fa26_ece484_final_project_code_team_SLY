# GEM e4 实车操作命令

使用车辆已有Ubuntu 22.04 / ROS Humble及 `~/gem_ws`。ENet权重已随仓库提供，无需单独复制。车载Python仍需兼容的PyTorch/CUDA；未验证硬件参数保持null，先只观察。

## 首次克隆与构建

以下在车载电脑执行；仓库为私有仓库，SSH密钥需获得课程组织授权。`~/sly_ws`须尚不存在；已有本仓库时使用 `git -C ~/sly_ws pull --ff-only` 更新，不覆盖其他工作空间。

```bash
git clone git@github.com:safeautonomy-illinois-students/fa26_ece484_final_project_code_team_SLY.git ~/sly_ws
source /opt/ros/humble/setup.bash
source ~/gem_ws/install/local_setup.bash
cd ~/sly_ws
colcon build --base-paths src --packages-select sly_lane_following --symlink-install
mkdir -p ~/sly_ws/results/e4
```

权重位置：`~/sly_ws/models/enet_v3.pth`。默认配置从仓库根目录读取这个相对路径，因此下面命令都先 `cd ~/sly_ws`，无需生成配置副本。

## 首次确认ENet运行环境

使用车载已有兼容PyTorch环境；若使用venv，先激活对应环境。不能复制Jazzy/Python 3.12的虚拟环境到Humble/Python 3.10。

```bash
python3 -c 'import torch, cv2, yaml; print("PyTorch:", torch.__version__, "CUDA:", torch.cuda.is_available())'
```

默认网络使用CUDA；报缺少模块或CUDA为False时，先完成车载依赖配置，再启动网络。GPU驱动尚未实查，不预设CUDA安装版本。

## 每个终端先执行

加载实车工作空间；所有终端沿用车辆一致的ROS通信域：

```bash
source /opt/ros/humble/setup.bash
source ~/gem_ws/install/local_setup.bash
source ~/sly_ws/install/local_setup.bash
export VEHICLE_NAME=e4
cd ~/sly_ws
```

## 终端1：前置OAK

已有相机进程则跳过。项目只使用RGB和CameraInfo。

```bash
ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=oak_link
```

## 终端2：PACMod驱动

已有驱动则跳过。

```bash
ros2 launch basic_launch dbw_only.launch.py
```

## 终端3：手柄输入

只发布手柄输入；不要同时运行 `dbw_joystick.launch.py` 或 `pacmod2_game_control`。默认使能按钮索引4，须现场核实。

```bash
ros2 run joy joy_node
```

## 终端4：ENet识别与Pure Pursuit请求

若PyTorch装在车载venv中，先激活对应环境，再执行：

```bash
python3 -m sly_lane_following.node --ros-args \
  -p use_sim_time:=false -p reliable_input:=false \
  -p config:="$HOME/sly_ws/src/sly_lane_following/config/real_enet.yaml" \
  -p control_config:="$HOME/sly_ws/src/sly_lane_following/config/e4_pure_pursuit.yaml" \
  -p image_topic:=/oak/rgb/image_raw \
  -p camera_info_topic:=/oak/rgb/camera_info
```

## 终端5：执行器适配器

先只观察，不发布执行器命令：

```bash
ros2 run sly_lane_following lane_adapter --ros-args \
  -p platform:=e4 -p dry_run:=true \
  -p hardware_config:="$HOME/sly_ws/src/sly_lane_following/config/e4_adapter.yaml"
```

现场验证并填写 `e4_adapter.yaml` 后，退出只观察节点，才可改为以下命令。未验证参数保持null，禁止使用模拟测试profile；现有速度上限0.2 m/s。

```bash
ros2 run sly_lane_following lane_adapter --ros-args \
  -p platform:=e4 -p dry_run:=false \
  -p hardware_config:="$HOME/sly_ws/src/sly_lane_following/config/e4_adapter.yaml"
```

## 终端6：显示识别结果

```bash
ros2 run rqt_image_view rqt_image_view /sly/lane/debug_image
```

## 终端7：录制数据

录前置RGB、标定、控制请求和车辆反馈：

```bash
ros2 bag record -o "$HOME/sly_ws/results/e4/run_$(date +%Y%m%d_%H%M%S)" \
  /oak/rgb/image_raw /oak/rgb/camera_info \
  /sly/lane/status /sly/lane/local_path /sly/control/request /sly/control/status \
  /pacmod/vehicle_speed_rpt /pacmod/global_rpt /pacmod/steering_rpt \
  /pacmod/brake_rpt /pacmod/shift_rpt /joy \
  /pacmod/global_cmd /pacmod/accel_cmd /pacmod/brake_cmd \
  /pacmod/steering_cmd /pacmod/shift_cmd
```

## 终端8：启用、停止与重置

车辆停稳、空挡、PACMod禁用，驾驶员按住已核实的使能按钮后启用；软件等待制动和前进挡确认才给油门：

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: true}'
```

正常停止：保持适配器运行，等待停稳、空挡与禁用确认。

```bash
ros2 service call /sly/control/enable std_srvs/srv/SetBool '{data: false}'
```

确认车辆停稳并退出控制后，再在各终端按 **Ctrl+C**。Ctrl+C不代替车辆制动。

仅在停稳、确认车道位置后重置识别；重置不会自动启用驾驶：

```bash
ros2 service call /sly/lane/reset std_srvs/srv/Trigger '{}'
```
