#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
repo_root=$(cd "$script_dir/.." && pwd)
workspace_root=$(cd "$repo_root/../.." && pwd)

default_config="$repo_root/config/euroc_mav/estimator_config_mh01_launch.yaml"
default_bag="${SLAM_DATASETS_ROOT:-/home/ros2/slam_datasets}/machine_hall/MH_01_easy/MH_01_easy"
bag_timeout_seconds=${REGRESSION_BAG_TIMEOUT_SECONDS:-40}
start_delay_seconds=${REGRESSION_START_DELAY_SECONDS:-3}
clock_rate=${OPENVINS_REGRESSION_CLOCK_RATE:-100.0}
run_tag=${REGRESSION_RUN_TAG:-$(date +%Y%m%d_%H%M%S)}

resolve_path() {
  local input_path="$1"
  local candidate

  if [[ "$input_path" = /* ]]; then
    printf '%s\n' "$input_path"
    return
  fi

  for candidate in \
    "$PWD/$input_path" \
    "$repo_root/$input_path" \
    "$workspace_root/$input_path"
  do
    if [[ -e "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return
    fi
  done

  printf '%s\n' "$input_path"
}

source_environment() {
  local env_script="$workspace_root/slam_dev_env.sh"
  if [[ -f "$env_script" ]]; then
    set +u
    source "$env_script"
    set -u
    return
  fi

  local setup_script="$workspace_root/install/setup.bash"
  if [[ -f "$setup_script" ]]; then
    set +u
    source "$setup_script"
    set -u
    return
  fi

  echo "failed to locate slam_dev_env.sh or install/setup.bash" >&2
  exit 65
}

config_arg=${1:-$default_config}
bag_arg=${2:-$default_bag}

config_path=$(resolve_path "$config_arg")
bag_path=$(resolve_path "$bag_arg")

if [[ ! -f "$config_path" ]]; then
  echo "OpenVINS regression config not found: $config_path" >&2
  exit 64
fi

if [[ ! -d "$bag_path" && ! -f "$bag_path" ]]; then
  echo "OpenVINS regression bag not found: $bag_path" >&2
  exit 66
fi

source_environment

config_name=$(basename "${config_path%.*}")
run_root="${OPENVINS_OUTPUT_ROOT:-$workspace_root/vio_output/openvins}/regressions/$config_name/$run_tag"
log_file="$run_root/openvins.log"
state_est="$run_root/state_estimate.txt"
state_std="$run_root/state_deviation.txt"
state_gt="$run_root/state_groundtruth.txt"

mkdir -p "$run_root"

launch_pid=

cleanup() {
  if [[ -n "${launch_pid:-}" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

(
  set +u
  source_environment
  stdbuf -oL -eL ros2 launch ov_msckf subscribe.launch.py \
    config_path:="$config_path" \
    rviz_enable:=false \
    save_total_state:=true \
    filepath_est:="$state_est" \
    filepath_std:="$state_std" \
    filepath_gt:="$state_gt"
) > >(tee -a "$log_file") 2> >(tee -a "$log_file" >&2) &
launch_pid=$!

sleep "$start_delay_seconds"

if (( bag_timeout_seconds > 0 )); then
  set +e
  timeout "$bag_timeout_seconds" \
    ros2 bag play "$bag_path" --clock "$clock_rate" --topics /imu0 /cam0/image_raw /cam1/image_raw
  bag_status=$?
  set -e
  if (( bag_status != 0 && bag_status != 124 )); then
    echo "OpenVINS regression bag playback failed with code $bag_status" >&2
    exit "$bag_status"
  fi
else
  ros2 bag play "$bag_path" --clock "$clock_rate" --topics /imu0 /cam0/image_raw /cam1/image_raw
fi

sleep 2
cleanup
trap - EXIT INT TERM

if [[ ! -s "$state_est" ]]; then
  echo "OpenVINS regression did not produce $state_est" >&2
  exit 67
fi

echo "OpenVINS regression artifacts: $run_root"
echo "  log:   $log_file"
echo "  state: $state_est"