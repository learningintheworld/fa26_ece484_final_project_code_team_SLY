#!/usr/bin/env bash
# Laptop-only compatibility test; no host networking, GPU or CAN devices.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -f "$root/gem_ws/src/hardware_drivers/as_drivers/pacmod2_msgs/package.xml" ]] || { echo 'Missing official gem_ws/pacmod2_msgs source'; exit 1; }
out="$root/results/humble_compat/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
docker build -t sly-humble-compat:local -f - "$root/scripts" <<'DOCKER'
FROM ros:humble-ros-base-jammy@sha256:d9f14e84747bea639a65e00cf871d2c68ae55385d2acf6239614de0cadb41dcc
RUN apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-pip python3-pytest python3-opencv ros-humble-cv-bridge ros-humble-ackermann-msgs && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install --no-cache-dir torch==1.13.1+cpu --extra-index-url https://download.pytorch.org/whl/cpu
DOCKER
docker run --rm --network none -e ROS_DOMAIN_ID=97 -e ROS_LOCALHOST_ONLY=1 \
  -e RESULT_UID="$(id -u)" -e RESULT_GID="$(id -g)" \
  -v "$root:/source:ro" -v "$out:/out" sly-humble-compat:local bash -lc '
set -eo pipefail
mkdir -p /test/src /test/scripts /test/models
trap '\''cp -r /test/results /out/ 2>/dev/null || true; chown -R "$RESULT_UID:$RESULT_GID" /out'\'' EXIT
cp -r /source/src/sly_lane_following /test/src/
cp -r /source/gem_ws/src/hardware_drivers/as_drivers/pacmod2_msgs /test/src/
cp /source/scripts/e4* /test/scripts/
cp /source/models/enet_v3.pth /test/models/
source /opt/ros/humble/setup.bash
cd /test
colcon build --base-paths src --symlink-install > /out/build.log 2>&1
source install/setup.bash
python3 scripts/e4_check.py cpu > /out/cpu.log 2>&1
python3 scripts/e4_check.py ros > /out/ros.log 2>&1
python3 -m pytest src/sly_lane_following/test/test_e4_deployment.py src/sly_lane_following/test/test_e4_diagnostics.py -q > /out/unit.log 2>&1
SLY_TEST_RESULTS=results/protocol python3 src/sly_lane_following/test/ros_e4_mock_check.py > /out/protocol.log 2>&1
python3 scripts/e4_diagnose.py > /out/diagnose.log 2>&1
'
echo "Results: $out"
