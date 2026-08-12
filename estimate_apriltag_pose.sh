#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
workspace="${DOBOT_WS:-${HOME}/dobot_ws}"
source "${workspace}/install/setup.bash"

# ROS 2 Humble setup scripts may inspect variables that do not exist in a
# clean shell. Enable nounset only after both environments have been sourced.
set -u

exec /usr/bin/python3 \
  "${workspace}/src/dobot_operator_gui/tools/estimate_apriltag_pose.py" \
  --capture \
  --camera-ip "${SC3000_IP:-192.168.192.11}" \
  --tag-size-mm 58.5 \
  --family auto \
  --timeout 12 \
  "$@"
