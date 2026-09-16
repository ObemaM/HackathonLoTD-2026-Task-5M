#!/usr/bin/env bash
set -eo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash
set -u
cd "${REPOSITORY_ROOT}/ros2_ws"
colcon test --event-handlers console_direct+
colcon test-result --verbose
