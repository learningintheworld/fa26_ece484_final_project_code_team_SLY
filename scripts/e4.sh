#!/usr/bin/env bash
# Vehicle-side Humble entry point; never source the Jazzy Docker workspace.
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
action="${1:-help}"
if [[ "$action" == help ]]; then
  echo 'Usage: bash scripts/e4.sh diagnose|setup|build|camera|dbw|joy|vision|vision-night|vision-cpu|vision-night-cpu|diagnose-night|observe|drive|view|record|enable|stop|reset'
  exit 0
fi
model=v3
case "$action" in vision-night|vision-night-cpu|diagnose-night) model=night ;; esac
# Preserve the caller's selected Python, but discard inherited ROS overlays.
python_bin="$(command -v "${SLY_PYTHON:-python3}")" || { echo 'Python interpreter not found' >&2; exit 1; }
[[ "$python_bin" = /* ]] || { echo 'SLY_PYTHON must resolve to an executable path' >&2; exit 1; }
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
# Diagnostics must work even when a required overlay is missing.
if [[ "$action" == diagnose || "$action" == diagnose-night ]]; then
  export SLY_DIAGNOSTIC_SOURCE_WARNINGS=''
  for setup_file in /opt/ros/humble/setup.bash "${SLY_DRIVER_WS:-$HOME/gem_ws}/install/setup.bash"; do
    if [[ -f "$setup_file" ]]; then
      source "$setup_file" || export SLY_DIAGNOSTIC_SOURCE_WARNINGS+="Failed to source $setup_file; "
    else
      export SLY_DIAGNOSTIC_SOURCE_WARNINGS+="Missing $setup_file; "
    fi
  done
  exec "$python_bin" "$root/scripts/e4_diagnose.py" --model "$model"
fi
fail() { echo "E4: $*" >&2; exit 1; }
[[ -f /opt/ros/humble/setup.bash ]] || fail 'ROS Humble is missing; run this entry on the vehicle.'
source /opt/ros/humble/setup.bash
underlay="${SLY_DRIVER_WS:-$HOME/gem_ws}/install/setup.bash"
[[ -f "$underlay" ]] || fail "Official driver setup missing: $underlay; set SLY_DRIVER_WS."
source "$underlay"
[[ "$ROS_DISTRO" == humble ]] || fail 'Official workspace is not a Humble workspace.'
export VEHICLE_NAME=e4
cd "$root"
config="$root/src/sly_lane_following/config"
check_ros() {
  ros2 pkg prefix pacmod2_msgs >/dev/null || fail 'pacmod2_msgs is not installed in the loaded driver workspace.'
  ros2 interface show pacmod2_msgs/msg/VehicleSpeedRpt >/dev/null || fail 'PACMod message interface is unavailable.'
  "$python_bin" "$root/scripts/e4_check.py" ros || fail 'Selected Python cannot load the required ROS interfaces.'
}
check_gpu() {
  # A separate process contains native CUDA aborts and reports a failed setup.
  "$python_bin" "$root/scripts/e4_check.py" gpu "$model" || fail 'ENet GPU inference failed. No build/drive was started; repair the existing environment or use vision-cpu / vision-night-cpu for observation.'
}
if [[ "$action" != build && "$action" != setup ]]; then
  [[ -f "$root/install/local_setup.bash" ]] || fail 'Project is not built; run setup first.'
  source "$root/install/local_setup.bash"
fi
case "$action" in
  setup|build)
    if [[ "$action" == setup ]]; then
      check_ros
      check_gpu
    fi
    [[ -x /usr/bin/colcon ]] || fail 'System colcon is missing; no packages were installed automatically.'
    # Build with the vehicle system Python; the inference interpreter is explicit.
    exec /usr/bin/colcon build --base-paths src --packages-select sly_lane_following --symlink-install
    ;;
  camera) exec ros2 launch depthai_ros_driver rgbd_pcl.launch.py parent_frame:=oak_link ;;
  dbw) exec ros2 launch basic_launch dbw_only.launch.py ;;
  joy) exec ros2 run joy joy_node ;;
  vision|vision-cpu|vision-night|vision-night-cpu)
    device=cuda
    if [[ "$action" == vision-cpu || "$action" == vision-night-cpu ]]; then
      device=cpu
      echo 'CPU observation mode; real-time driving is not validated.' >&2
    else
      check_gpu
    fi
    vision_config="$config/real_enet.yaml"
    [[ "$model" == night ]] && vision_config="$config/real_enet_night.yaml"
    echo "ENet model: $model | config: $vision_config | device: $device"
    [[ "$model" == night ]] && echo "Night candidate: synthetic low-light improvement only; real-night driving is not validated." >&2
    profile="$(mktemp /tmp/sly_e4_vision.XXXXXX.yaml)"
    trap 'rm -f "$profile"' EXIT
    sed "s/^enet_device:.*/enet_device: $device/" "$vision_config" > "$profile"
    "$python_bin" -m sly_lane_following.node --ros-args \
      -p use_sim_time:=false -p reliable_input:=false \
      -p config:="$profile" -p control_config:="$config/e4_pure_pursuit.yaml" \
      -p image_topic:=/oak/rgb/image_raw -p camera_info_topic:=/oak/rgb/camera_info
    ;;
  observe|drive)
    check_ros
    dry_run=true
    [[ "$action" == drive ]] && dry_run=false
    exec "$python_bin" -c 'from sly_lane_following.adapter import main; main()' --ros-args \
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
