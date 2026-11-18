#!/usr/bin/env bash
# Vehicle-side Humble entry point; never source the Jazzy Docker workspace.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-help}"
if [[ "$action" == help ]]; then
  echo 'Usage: bash scripts/e4.sh build|camera|dbw|joy|vision|observe|drive|view|record|enable|stop|reset'
  exit 0
fi
source /opt/ros/humble/setup.bash
source "${SLY_DRIVER_WS:-$HOME/gem_ws}/install/setup.bash"
if [[ -f "$root/install/local_setup.bash" ]]; then
  source "$root/install/local_setup.bash"
fi
export VEHICLE_NAME=e4
cd "$root"
config="$root/src/sly_lane_following/config"
case "$action" in
  build) exec colcon build --base-paths src --packages-select sly_lane_following --symlink-install ;;
  camera) exec ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=oak_link ;;
  dbw) exec ros2 launch basic_launch dbw_only.launch.py ;;
  joy) exec ros2 run joy joy_node ;;
  vision)
    # Temporary CPU fallback for the vehicle's unresolved CUDA library error.
    profile="$(mktemp /tmp/sly_e4_cpu.XXXXXX.yaml)"
    trap 'rm -f "$profile"' EXIT
    sed 's/^enet_device:.*/enet_device: cpu/' "$config/real_enet.yaml" > "$profile"
    python3 -m sly_lane_following.node --ros-args \
      -p use_sim_time:=false -p reliable_input:=false \
      -p config:="$profile" -p control_config:="$config/e4_pure_pursuit.yaml" \
      -p image_topic:=/oak/rgb/image_raw -p camera_info_topic:=/oak/rgb/camera_info
    ;;
  observe|drive)
    dry_run=true
    [[ "$action" == drive ]] && dry_run=false
    exec ros2 run sly_lane_following lane_adapter --ros-args \
      -p platform:=e4 -p dry_run:="$dry_run" -p hardware_config:="$config/e4_adapter.yaml"
    ;;
  view) exec ros2 run rqt_image_view rqt_image_view /sly/lane/debug_image ;;
  record)
    mkdir -p results/e4
    exec ros2 bag record -o "results/e4/run_$(date +%Y%m%d_%H%M%S)" \
      /oak/rgb/image_raw /oak/rgb/camera_info \
      /sly/lane/status /sly/lane/local_path /sly/control/request /sly/control/status \
      /pacmod/vehicle_speed_rpt /pacmod/global_rpt /pacmod/steering_rpt \
      /pacmod/brake_rpt /pacmod/shift_rpt /joy \
      /pacmod/global_cmd /pacmod/accel_cmd /pacmod/brake_cmd /pacmod/steering_cmd /pacmod/shift_cmd
    ;;
  enable|stop)
    value=false
    [[ "$action" == enable ]] && value=true
    exec ros2 service call /sly/control/enable std_srvs/srv/SetBool "{data: $value}"
    ;;
  reset) exec ros2 service call /sly/lane/reset std_srvs/srv/Trigger '{}' ;;
  *) echo "Unknown action: $action" >&2; exit 2 ;;
esac
