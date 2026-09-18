#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repository_root}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${repository_root}/tools/replay_transport.xml"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-84}"
result_name="${1:-current}"

for offset in 0 120 240; do
  python3 tools/replay_check.py \
    --executable "${repository_root}/ros2_ws/install/metro_obstacle_detector/lib/metro_obstacle_detector/detector_node" \
    --config "${repository_root}/ros2_ws/src/metro_obstacle_detector/config/default.yaml" \
    --output "experiments/results/${result_name}_${offset}.json" \
    --offset "${offset}" --frames 25 \
    "${HOME}/HackathonLoDT/data/for_hackathon/doubleT_platform" \
    "${HOME}/HackathonLoDT/data/for_hackathon/roundT_doubleT" \
    "${HOME}/HackathonLoDT/data/for_hackathon/roundT_pressureGate_roundT" \
    "${HOME}/HackathonLoDT/data/for_hackathon/roundT_squareT_pressureGate_squareT" \
    "${HOME}/HackathonLoDT/data/for_hackathon/squareT_platform_squareT_switch"
done

python3 tools/summarize_replay.py --after-name "${result_name}"
