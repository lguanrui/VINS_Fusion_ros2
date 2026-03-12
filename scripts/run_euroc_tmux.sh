#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
INSTALL_SETUP="${WS_ROOT}/install/setup.bash"
DEFAULT_BAG="${WS_ROOT}/dataset_ros2/machine_hall/MH_01_easy/MH_01_easy"

SESSION="vins-euroc"
BAG_PATH="${DEFAULT_BAG}"
USE_LOOP=true
ATTACH=true
CONFIG_FILE="${REPO_ROOT}/config/euroc/euroc_stereo_imu_config.yaml"
RVIZ_CONFIG="${REPO_ROOT}/config/vins_rviz_config.rviz"
LOG_DIR="${WS_ROOT}/.ros_logs"

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --bag PATH          ROS2 bag directory to play.
  --config PATH       EuRoC config yaml.
  --session NAME      tmux session name (default: vins-euroc).
  --no-loop           Do not start loop_fusion.
  --detach            Create the tmux session without attaching.
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
    --session)
      SESSION="$2"
      shift 2
      ;;
    --no-loop)
      USE_LOOP=false
      shift
      ;;
    --detach)
      ATTACH=false
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

if [[ ! -d "${BAG_PATH}" ]]; then
  echo "ROS2 bag directory not found: ${BAG_PATH}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"

COMMON_ENV="source /opt/ros/humble/setup.bash && source ${INSTALL_SETUP} && export ROS_LOG_DIR=${LOG_DIR} && export RCUTILS_LOGGING_BUFFERED_STREAM=1"
VINS_CMD="${COMMON_ENV} && ros2 run vins vins_node ${CONFIG_FILE}"
LOOP_CMD="${COMMON_ENV} && ros2 run loop_fusion loop_fusion_node ${CONFIG_FILE}"
RVIZ_CMD="${COMMON_ENV} && rviz2 -d ${RVIZ_CONFIG}"
BAG_CMD="${COMMON_ENV} && ros2 bag play ${BAG_PATH} --disable-keyboard-controls"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
  tmux kill-session -t "${SESSION}"
fi

tmux new-session -d -s "${SESSION}" -c "${WS_ROOT}"
tmux send-keys -t "${SESSION}:0.0" "${VINS_CMD}" C-m

tmux split-window -h -t "${SESSION}:0.0" -c "${WS_ROOT}"
if [[ "${USE_LOOP}" == true ]]; then
  tmux send-keys -t "${SESSION}:0.1" "${LOOP_CMD}" C-m
else
  tmux send-keys -t "${SESSION}:0.1" "echo 'loop_fusion disabled'; bash" C-m
fi

tmux split-window -v -t "${SESSION}:0.0" -c "${WS_ROOT}"
tmux send-keys -t "${SESSION}:0.2" "${RVIZ_CMD}" C-m

tmux split-window -v -t "${SESSION}:0.1" -c "${WS_ROOT}"
tmux send-keys -t "${SESSION}:0.3" "${BAG_CMD}" C-m

tmux select-layout -t "${SESSION}:0" tiled

if [[ "${ATTACH}" == true ]]; then
  if [[ -n "${TMUX:-}" ]]; then
    exec tmux switch-client -t "${SESSION}"
  else
    exec tmux attach-session -t "${SESSION}"
  fi
fi

echo "Created tmux session: ${SESSION}"
