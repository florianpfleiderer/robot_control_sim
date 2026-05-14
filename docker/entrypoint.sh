#!/usr/bin/env bash
# ROS setup scripts reference unset variables (e.g. AMENT_TRACE_SETUP_FILES),
# so nounset is intentionally omitted here.
set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

WS_SETUP="/workspace/ros2_ws/install/setup.bash"
if [ -f "${WS_SETUP}" ]; then
  # shellcheck disable=SC1090
  source "${WS_SETUP}"
fi

exec "$@"
