#!/usr/bin/env bash
set -eo pipefail
cd /mnt/d/Development/HackathonLoTD-2026/HackathonLoTD-2026-Task-5M
baseline_dir=$(mktemp -d "$PWD/experiments/results/baseline-XXXXXX")
git archive HEAD | tar -x -C "$baseline_dir"
printf 'Baseline directory: %s\n' "$baseline_dir"
source /opt/ros/humble/setup.bash
cd "$baseline_dir/ros2_ws"
MAKEFLAGS='-j1 -l1' colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
