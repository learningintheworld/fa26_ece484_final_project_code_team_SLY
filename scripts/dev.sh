#!/usr/bin/env bash
# Run on the host. Project processes execute in the existing ECE484 service.
set -eo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: ./scripts/dev.sh COMMAND [arguments]
  up                  Start the existing service with read-only /data/gem
  shell               Open a clean ROS + SLY shell
  info                Show bag metadata
  inspect [options]   Decode all messages and write results/stage1
  replay [options]    Play at 0.5x, initially paused (Space: play/pause)
  view                Show RGB and LiDAR in RViz2
  imu                 Show angular velocity and acceleration in two plots
  pause | resume      Control the running rosbag player from another terminal
  build               Build only the SLY package
  lane-offline [args] Analyze the bag, save overlay video and per-frame records
  lane [ROS args]     Run perception on replayed RGB + CameraInfo
  lane-view           View the pixel-domain debug image
  sim [launch args]   Launch Highbay with the existing simulator overlay
  lane-sim [ROS args] Original image-servo baseline on simulated RGB
  lane-sim-pp [args]  Visual Pure Pursuit on simulated RGB
  lane-pp [args]      E4 nominal-geometry preview on replay; no actuator outlet
  lane-enet [args]    Optional fine-tuned ENet replay preview, existing Python venv
  lane-offline-enet   Full offline ENet analysis (arguments accepted)
  adapter [ROS args]  Simulation command outlet (starts disarmed)
  enable | stop      Explicitly arm / latch stop
  lane-reset         Reset perception after a stopped, aligned restart
  exec COMMAND ...    Run a command in the clean project environment

Examples:
  ./scripts/dev.sh replay --rate 1.0 --start-offset 75
  ./scripts/dev.sh inspect --output results/another_run

Host overrides: SLY_DATA_DIR, SLY_IMAGE, SLY_COMPOSE_PROJECT (default ece484).
Process overrides: SLY_ROS_DOMAIN_ID (default 84), SLY_BAG (container path).
EOF
}

if [[ "${1:-}" == "_inside" ]]; then
  shift
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$repo_root/install/local_setup.bash" ]]; then
    source "$repo_root/install/local_setup.bash"
  fi
  cd "$repo_root"
  set -u
  bag="${SLY_BAG:-/data/gem/oak_front_image}"
  action="${1:-shell}"
  shift || true
  case "$action" in
    shell) exec bash --noprofile --norc -i ;;
    info) exec ros2 bag info "$bag" ;;
    inspect)
      exec python scripts/inspect_bag.py "$bag" --output results/stage1 "$@"
      ;;
    replay)
      [[ -f "$bag/metadata.yaml" ]] || { echo "Missing bag: $bag" >&2; exit 1; }
      # Use a fresh graph query; the project domain must contain only one player.
      nodes="$(ros2 node list --no-daemon --spin-time 2 2>/dev/null)"
      if [[ "$nodes" == *"/rosbag2_player"* ]]; then
        echo "A rosbag2_player is already running in this domain. Stop it before replaying." >&2
        exit 1
      fi
      exec ros2 bag play "$bag" --clock 30 --rate 0.5 --start-paused "$@"
      ;;
    view)
      exec rviz2 -d "$repo_root/config/replay.rviz" --ros-args -p use_sim_time:=true "$@"
      ;;
    imu)
      exec python scripts/view_imu.py
      ;;
    pause)
      exec timeout 10s ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause '{}'
      ;;
    resume)
      exec timeout 10s ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
      ;;
    build) exec colcon build --base-paths src --packages-select sly_lane_following --symlink-install "$@" ;;
    lane-offline) exec ros2 run sly_lane_following lane_offline --bag "$bag" "$@" ;;
    lane) exec ros2 run sly_lane_following lane_node --ros-args -p use_sim_time:=true "$@" ;;
    lane-view) exec ros2 run rqt_image_view rqt_image_view /sly/lane/debug_image ;;
    sim)
      set +u
      source /workspace/gem_simulator/install/local_setup.bash
      set -u
      export GZ_PARTITION=sly_highbay_m3
      exec ros2 launch sly_lane_following sim.launch.py "$@"
      ;;
    lane-offline-enet)
      exec python -m sly_lane_following.offline --bag "$bag" \
        --config "$repo_root/src/sly_lane_following/config/real_enet.yaml" \
        --control-config "$repo_root/src/sly_lane_following/config/e4_pure_pursuit.yaml" "$@"
      ;;
    lane-enet)
      exec python -m sly_lane_following.node --ros-args -p use_sim_time:=true \
        -p config:="$repo_root/src/sly_lane_following/config/real_enet.yaml" \
        -p control_config:="$repo_root/src/sly_lane_following/config/e4_pure_pursuit.yaml" "$@"
      ;;
    lane-pp)
      exec ros2 run sly_lane_following lane_node --ros-args -p use_sim_time:=true \
        -p control_config:="$repo_root/src/sly_lane_following/config/e4_pure_pursuit.yaml" "$@"
      ;;
    lane-sim|lane-sim-pp)
      control_profile=sim_control.yaml
      [[ "$action" == lane-sim-pp ]] && control_profile=sim_pure_pursuit.yaml
      exec ros2 run sly_lane_following lane_node --ros-args -p use_sim_time:=true \
        -p config:="$repo_root/src/sly_lane_following/config/highbay.yaml" \
        -p control_config:="$repo_root/src/sly_lane_following/config/$control_profile" \
        -p image_topic:=/camera/image_raw -p camera_info_topic:=/camera/camera_info -p reliable_input:=false "$@"
      ;;
    adapter)
      exec ros2 run sly_lane_following lane_adapter --ros-args -p use_sim_time:=true -p platform:=sim -p dry_run:=false "$@"
      ;;
    enable|stop)
      value=false; [[ "$action" == enable ]] && value=true
      exec timeout 10s ros2 service call /sly/control/enable std_srvs/srv/SetBool "{data: $value}"
      ;;
    lane-reset) exec timeout 10s ros2 service call /sly/lane/reset std_srvs/srv/Trigger '{}' ;;
    exec) exec "$@" ;;
    *) usage; exit 2 ;;
  esac
fi

set -u
action="${1:-help}"
shift || true
case "$action" in
  help|-h|--help) usage; exit 0 ;;
  up|shell|info|inspect|replay|view|imu|pause|resume|build|lane-offline|lane|lane-view|sim|lane-sim|lane-sim-pp|lane-pp|lane-enet|lane-offline-enet|adapter|enable|stop|lane-reset|exec) ;;
  *) usage; exit 2 ;;
esac

workspace_root="$(dirname -- "$repo_root")"
if [[ -f "$repo_root/.sly-local.env" && -z "${SLY_IMAGE:-}" ]]; then
  # Local image snapshot selection, intentionally not tracked by Git.
  source "$repo_root/.sly-local.env"
fi
export SLY_IMAGE="${SLY_IMAGE:-ece484:jazzy-harmonic-cu128}"
export SLY_DATA_DIR="${SLY_DATA_DIR:-/mnt/Data/01_Projects/ECE484_GEMe4/data}"
project_name="${SLY_COMPOSE_PROJECT:-ece484}"
[[ -f "$workspace_root/compose.yaml" ]] || { echo "Missing existing compose.yaml" >&2; exit 1; }
compose=(docker compose --project-name "$project_name" --project-directory "$workspace_root"
  -f "$workspace_root/compose.yaml" -f "$repo_root/compose.sly.yaml")

if [[ "$action" == "up" ]]; then
  [[ -d "$SLY_DATA_DIR" ]] || { echo "Data disk is not mounted: $SLY_DATA_DIR" >&2; exit 1; }
  [[ -f "$SLY_DATA_DIR/oak_front_image/metadata.yaml" ]] || {
    echo "Expected oak_front_image/metadata.yaml under $SLY_DATA_DIR" >&2; exit 1;
  }
  exec "${compose[@]}" up -d --no-build ece484
fi

container_id="$("${compose[@]}" ps --status running -q ece484)"
[[ -n "$container_id" ]] || { echo "First run: ./scripts/dev.sh up" >&2; exit 1; }
data_mount="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/data/gem"}}{{.Source}}|{{.RW}}{{end}}{{end}}' "$container_id")"
[[ "$data_mount" == "$SLY_DATA_DIR|false" ]] || {
  echo "Expected read-only /data/gem mount. Run ./scripts/dev.sh up." >&2; exit 1;
}
container_repo="/workspace/$(basename -- "$repo_root")"
tty_args=()
if [[ ! -t 0 || ! -t 1 ]]; then tty_args=(-T); fi

exec "${compose[@]}" exec "${tty_args[@]}" \
  -e ROS_DOMAIN_ID="${SLY_ROS_DOMAIN_ID:-84}" -e ROS_LOCALHOST_ONLY=1 \
  -e SLY_BAG="${SLY_BAG:-/data/gem/oak_front_image}" \
  -e VIRTUAL_ENV=/workspace/.venv \
  -e PATH=/workspace/.venv/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  ece484 env -u BASH_ENV -u ENV -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH -u ROS_PACKAGE_PATH -u PYTHONPATH -u LD_LIBRARY_PATH \
  bash --noprofile --norc "$container_repo/scripts/dev.sh" _inside "$action" "$@"
