# GEM e4 简明指南：前置相机、LiDAR、GNSS 与 Joystick

车载 Ubuntu 22.04，已编译工作空间 `~/gem_ws`。以下使用 Bash，按顺序执行并保持各终端运行。仅修改本地文档，不上传 GitHub。

本次启动 OAK 前置 RGB/深度、顶部 Ouster、前方 Livox、GNSS/INS 和 joystick；不启动四路 Lucid。录制保留相机标定、TF，不保存 OAK 点云和车辆控制命令。

若旧的 sensor_init 或驱动还在运行，先按文末流程安全停车并结束旧进程。本次不要运行 `sensor_init.launch.py`，它会启动四路相机。

## 终端 1：车辆模型与 TF

```bash
cd ~/gem_ws
source install/setup.bash
export VEHICLE_NAME=e4
ros2 launch basic_launch robot_description.launch.py
```

## 终端 2：GNSS/INS

```bash
cd ~/gem_ws
source install/setup.bash
export VEHICLE_NAME=e4
ros2 launch basic_launch gnss.launch.py
```

按官方硬件流程确认 GNSS 获得足够卫星信号后，再打开 LiDAR 电源，等待设备启动，再执行终端 3、4。

## 终端 3：顶部 Ouster LiDAR

```bash
cd ~/gem_ws
source install/setup.bash
export VEHICLE_NAME=e4
ros2 launch basic_launch ouster_driver.launch.py
```

## 终端 4：前方 Livox LiDAR

```bash
cd ~/gem_ws
source install/setup.bash
export VEHICLE_NAME=e4
ros2 launch basic_launch livox_lidar.launch.py
```

## 终端 5：OAK 前置相机

```bash
cd ~/gem_ws
source install/setup.bash
export VEHICLE_NAME=e4
ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=oak_link
```

## 终端 6：Joystick

```bash
cd ~/gem_ws
source install/setup.bash
export VEHICLE_NAME=e4
ros2 launch basic_launch dbw_joystick.launch.py
```

该 launch 默认包含 PACMod，不再启动 dbw_only。使能、换挡、驾驶和退出线控按助教培训执行。Ctrl+C 或切断线控不等于车辆自动刹车。

## 终端 7：选择性录制

传感器启动完成后执行。下面只匹配指定数据，不用 `-a`；实际 topic 名称依据当前官方配置，若车载名称不同需相应修改。GNSS 允许不同命名空间。建议先静止短录约 30 秒，确认素材可用再正式采集。

```bash
cd ~/gem_ws
source install/setup.bash
export VEHICLE_NAME=e4
RUN_DIR="$HOME/gem_data/SLY/$(date +%Y%m%d_%H%M%S)_e4_front_lidar_gnss"
mkdir -p "$RUN_DIR"
printf '数据目录：%s\n' "$RUN_DIR"
GEM_RECORD_REGEX='^(/(tf|tf_static|joint_states|robot_description)|/oak/(rgb|stereo)/(image_raw|camera_info)|/ouster/(points.*|imu|metadata)|/livox/(lidar|imu)|/(.*/)?(navsatfix|imu|twist_ins|twist_gnss|insnavgeod|pvtgeodetic))$'
ros2 bag record -s sqlite3 --max-bag-size 1073741824 \
  -e "$GEM_RECORD_REGEX" -o "$RUN_DIR/bag"
```

前置 RGB/深度仍是原始数据；省去四路 Lucid 和 OAK 点云后容量会减少，但不等于普通 MP4 大小。1 GiB 参数只控制分片大小，不限制总容量。再次录制时重新执行终端 7 整段命令，创建新目录；不要提前创建 bag 子目录。匹配规则不会启动缺失传感器，也不能录到未发布的数据。

## 停止与保存

1. 操作者先按助教流程完全停车、完成规定的挡位、驻车和线控退出操作，保留软件运行直到车辆安全。
2. **终端 7：按一次 Ctrl+C**，等待录制程序正常退出。数据自动保存，不需要额外保存命令。
3. 确认车辆已安全退出线控后，**终端 6 按 Ctrl+C**。
4. 依次在终端 **5、4、3、2、1 按 Ctrl+C**，再按车辆规程关闭设备。

将终端 7 打印的整个采集目录复制到个人电脑，包括 `bag/metadata.yaml` 和所有 `.db3` 分片。建议放到 `/home/xinweiyu/Workspace/ECE484_Program/SLY_data/`，不上传 Git。确认传输和回放成功后再清理自己留在车上的数据。

## 个人电脑终端 A：离线回放

仅用于本指南选择性录制的新 bag。使用与车载一致的 ROS 发行版；下例假定已安装 Humble（Ubuntu 22.04 本身不能证明发行版）。GNSS 自定义消息需要加载本机已编译的对应 workspace。替换下面的实际目录名。

**不运行 joystick/PACMod/CAN 驱动，不在实车控制环境中回放。旧的 `-a` 全量 bag 可能含控制命令，不能直接套用此全包回放命令。**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
LOCAL_RUN='/home/xinweiyu/Workspace/ECE484_Program/SLY_data/实际采集目录名'
ros2 bag play "$LOCAL_RUN/bag" --clock 40 --rate 0.5 --start-paused
```

先启动下面的 RViz，再按播放器提示用空格开始。`0.5` 表示半速，可改成 `1.0`。在终端 A 按 Ctrl+C 停止回放。

## 个人电脑终端 B：RViz

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
rviz2 --ros-args -p use_sim_time:=true
```

Fixed Frame 选择 `base_link`，Add → By topic 添加前置图像和点云，传感器显示的 Reliability 按需设为 Best Effort。关闭窗口或 Ctrl+C 退出。下次实车采集使用新的车载终端及车上既定 ROS domain，不沿用回放 domain 88。

---

源码核对基线：`5afbd638d182b40d5300a38bd42e48df969fb42c`；本简版尚未在当前车辆实测。

参考：[官方 launch 目录](https://github.com/UIUC-Robotics/gem_ws/tree/5afbd638d182b40d5300a38bd42e48df969fb42c/src/basic_launch/launch) · [硬件上电说明](https://github.com/UIUC-Robotics/gem_ws/wiki/Hardware-Setup)
