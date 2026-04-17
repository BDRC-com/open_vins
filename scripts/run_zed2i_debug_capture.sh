#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd "$(dirname "$0")" && pwd)
repo_root=$(cd "$script_dir/.." && pwd)
workspace_root=$(cd "$repo_root/../.." && pwd)

default_config="$repo_root/config/zed2i/estimator_config_stable.yaml"
duration_seconds=${ZED2I_DEBUG_DURATION_SECONDS:-60}
run_tag=${ZED2I_DEBUG_RUN_TAG:-$(date +%Y%m%d_%H%M%S)}

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

copy_if_exists() {
  local source_path="$1"
  local target_path="$2"
  if [[ -f "$source_path" ]]; then
    cp "$source_path" "$target_path"
  fi
}

config_arg=${1:-$default_config}
duration_arg=${2:-$duration_seconds}

config_path=$(resolve_path "$config_arg")

if [[ ! -f "$config_path" ]]; then
  echo "OpenVINS ZED2i config not found: $config_path" >&2
  exit 64
fi

if ! [[ "$duration_arg" =~ ^[0-9]+$ ]]; then
  echo "duration must be an integer number of seconds, got: $duration_arg" >&2
  exit 66
fi

source_environment

run_root="${OPENVINS_OUTPUT_ROOT:-$workspace_root/vio_output/openvins}/zed2i_debug/$run_tag"
bag_root="$run_root/bag"
config_snapshot_dir="$run_root/config_snapshot"
log_file="$run_root/openvins.log"
metadata_file="$run_root/run_metadata.txt"
state_est="$run_root/state_estimate.txt"
state_std="$run_root/state_deviation.txt"
state_gt="$run_root/state_groundtruth.txt"

mkdir -p "$run_root" "$config_snapshot_dir"

cp "$config_path" "$config_snapshot_dir/$(basename "$config_path")"
copy_if_exists "$repo_root/config/zed2i/kalibr_imu_chain.yaml" "$config_snapshot_dir/kalibr_imu_chain.yaml"
copy_if_exists "$repo_root/config/zed2i/kalibr_imucam_chain.yaml" "$config_snapshot_dir/kalibr_imucam_chain.yaml"
copy_if_exists "$HOME/.ros/zed2i/sdk_calibration_runtime.yaml" "$run_root/sdk_calibration_runtime.yaml"
copy_if_exists "$workspace_root/zed2i_vins.yaml" "$run_root/zed2i_vins.yaml"
copy_if_exists "$workspace_root/zed2i_imu_params_runtime.yaml" "$run_root/zed2i_imu_params_runtime.yaml"

cat >"$metadata_file" <<EOF
run_tag=$run_tag
started_at=$(date --iso-8601=seconds)
config_path=$config_path
duration_seconds=$duration_arg
workspace_root=$workspace_root
bag_topics=/zed2i/imu /zed2i/left/image_raw /zed2i/right/image_raw /zed2i/left/camera_info_raw /zed2i/right/camera_info_raw /ov_msckf/poseimu /ov_msckf/odomimu /ov_msckf/pathimu /ov_msckf/points_msckf /ov_msckf/points_slam /tf /tf_static
EOF

record_pid=
launch_pid=

cleanup() {
  local status=0
  if [[ -n "${launch_pid:-}" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || status=$?
  fi
  if [[ -n "${record_pid:-}" ]] && kill -0 "$record_pid" 2>/dev/null; then
    kill -INT "$record_pid" 2>/dev/null || true
    wait "$record_pid" 2>/dev/null || status=$?
  fi
  return 0
}

trap cleanup EXIT INT TERM

bag_topics=(
  /zed2i/imu
  /zed2i/left/image_raw
  /zed2i/right/image_raw
  /zed2i/left/camera_info_raw
  /zed2i/right/camera_info_raw
  /ov_msckf/poseimu
  /ov_msckf/odomimu
  /ov_msckf/pathimu
  /ov_msckf/points_msckf
  /ov_msckf/points_slam
  /tf
  /tf_static
)

(
  set +u
  source_environment
  stdbuf -oL -eL ros2 bag record -o "$bag_root" --topics "${bag_topics[@]}"
) >"$run_root/rosbag_record.log" 2>&1 &
record_pid=$!

sleep 2

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

sleep 3

if (( duration_arg > 0 )); then
  sleep "$duration_arg"
else
  echo "Recording until interrupted. Artifacts: $run_root"
  while true; do
    sleep 3600
  done
fi

cleanup
trap - EXIT INT TERM

echo "OpenVINS ZED2i debug artifacts: $run_root"
echo "  bag:   $bag_root"
echo "  log:   $log_file"
echo "  state: $state_est"
