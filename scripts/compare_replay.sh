#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /mnt/d/Development/HackathonLoTD-2026/HackathonLoTD-2026-Task-5M
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/tools/replay_transport.xml"
export ROS_DOMAIN_ID=82
baseline_dir=${1:?Pass the directory printed by build_baseline.sh}
for offset in ${OFFSETS:-0 120 240}; do
  for version in ${VERSIONS:-before after}; do
    prefix="$PWD"
    if [ "$version" = before ]; then prefix="$baseline_dir"; fi
    python3 tools/replay_check.py \
      --executable "$prefix/ros2_ws/install/metro_obstacle_detector/lib/metro_obstacle_detector/detector_node" \
      --config "$prefix/ros2_ws/src/metro_obstacle_detector/config/default.yaml" \
      --output "experiments/results/${version}_${offset}.json" --offset "$offset" --frames 25 \
      "$HOME/HackathonLoDT/data/for_hackathon/doubleT_platform" \
      "$HOME/HackathonLoDT/data/for_hackathon/roundT_doubleT" \
      "$HOME/HackathonLoDT/data/for_hackathon/roundT_pressureGate_roundT" \
      "$HOME/HackathonLoDT/data/for_hackathon/roundT_squareT_pressureGate_squareT" \
      "$HOME/HackathonLoDT/data/for_hackathon/squareT_platform_squareT_switch"
  done
done
