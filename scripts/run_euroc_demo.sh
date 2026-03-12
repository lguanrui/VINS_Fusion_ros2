#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
INSTALL_SETUP="${WS_ROOT}/install/setup.bash"
DEFAULT_BAG="${WS_ROOT}/dataset_ros2/machine_hall/MH_01_easy/MH_01_easy"

echo "${INSTALL_SETUP}"

BAG_PATH="${DEFAULT_BAG}"
USE_LOOP_FUSION=true
USE_RVIZ=true
PLAY_BAG=true
BAG_RATE="1.0"
CONFIG_FILE="${REPO_ROOT}/config/euroc/euroc_stereo_imu_config.yaml"
LOG_DIR="${WS_ROOT}/.ros_logs"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --bag PATH          ROS2 bag directory to play.
  --config PATH       EuRoC config yaml.
  --bag-rate RATE     ros2 bag playback rate (default: 1.0).
  --no-loop           Do not start loop_fusion.
  --no-rviz           Do not start RViz2.
  --no-bag-play       Launch nodes only.
  -h, --help          Show this help.

If no --bag is provided, the script uses:
  ${DEFAULT_BAG}
EOF
}

while (($#)); do
  case "$1" in
    --bag)
      BAG_PATH="$2"
      shift 2
      ;;
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --bag-rate)
      BAG_RATE="$2"
      shift 2
      ;;
    --no-loop)
      USE_LOOP_FUSION=false
      shift
      ;;
    --no-rviz)
      USE_RVIZ=false
      shift
      ;;
    --no-bag-play)
      PLAY_BAG=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${INSTALL_SETUP}" ]]; then
  echo "Workspace is not built yet: ${INSTALL_SETUP} not found" >&2
  exit 1
fi

if [[ "${PLAY_BAG}" == true && ! -d "${BAG_PATH}" ]]; then
  echo "ROS2 bag directory not found: ${BAG_PATH}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"

#source /opt/ros/humble/setup.bash
#source "${INSTALL_SETUP}"

exec ros2 launch vins euroc_stereo_imu_demo.launch.py \
  config_file:="${CONFIG_FILE}" \
  bag_path:="${BAG_PATH}" \
  play_bag:="${PLAY_BAG}" \
  bag_rate:="${BAG_RATE}" \
  use_loop_fusion:="${USE_LOOP_FUSION}" \
  use_rviz:="${USE_RVIZ}" \
  log_dir:="${LOG_DIR}"
