#!/usr/bin/env bash
set -eo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
set -u
cd "${REPOSITORY_ROOT}/ros2_ws"
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Build completed. Run: source ${REPOSITORY_ROOT}/ros2_ws/install/setup.bash"
