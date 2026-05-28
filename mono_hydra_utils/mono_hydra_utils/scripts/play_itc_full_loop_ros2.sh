#!/usr/bin/env bash
set -euo pipefail

BAG_PATH="test_data/itc_ros2_bags/ITC_2nd_floor_full_loop_ros2"

usage() {
  cat <<'USAGE'
Usage: play_itc_full_loop_ros2.sh [BAG_PATH]

Plays the converted ITC full-loop ROS 2 bag with the TF remaps required by
Mono Hydra. By default, playback runs at rosbag's normal real-time rate.

Examples:
  play_itc_full_loop_ros2.sh
  play_itc_full_loop_ros2.sh test_data/itc_ros2_bags/ITC_2nd_floor_full_loop_ros2
USAGE
}

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    *)
      BAG_PATH="$1"
      shift
      ;;
  esac
done

QOS_OVERRIDES="$(ros2 pkg prefix mono_hydra_utils)/share/mono_hydra_utils/config/tf_overrides.yaml"

PLAY_ARGS=(ros2 bag play "${BAG_PATH}" \
  --clock \
)

PLAY_ARGS+=(\
  --qos-profile-overrides-path "${QOS_OVERRIDES}" \
  --remap /tf:=/tf_ignore /tf_static:=/tf_static_ignore \
)

exec "${PLAY_ARGS[@]}"
