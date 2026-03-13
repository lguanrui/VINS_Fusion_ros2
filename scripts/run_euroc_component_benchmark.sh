#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
INSTALL_SETUP="${WS_ROOT}/install/setup.bash"
DEFAULT_BAG="${WS_ROOT}/dataset_ros2/machine_hall/MH_01_easy/MH_01_easy"
DEFAULT_CONFIG="${REPO_ROOT}/config/euroc/euroc_stereo_imu_config.yaml"
DEFAULT_OUT_DIR="${WS_ROOT}/test_results/euroc_component_benchmark"
CERES_LIB_DIR="${REPO_ROOT}/third_party/ceres-install/lib"

BAG_PATH="${DEFAULT_BAG}"
CONFIG_FILE="${DEFAULT_CONFIG}"
OUT_DIR="${DEFAULT_OUT_DIR}"
BAG_RATE="1.0"
USE_LOOP_FUSION=true
GT_TOPIC=""
SIM3=false

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --bag PATH          Input EuRoC ros2 bag directory. Default: ${DEFAULT_BAG}
  --config PATH       VINS config yaml. Default: ${DEFAULT_CONFIG}
  --out-dir PATH      Output directory for recorded bag + metrics.
  --bag-rate RATE     ros2 bag playback rate. Default: 1.0
  --no-loop           Evaluate only /vins_estimator/odometry.
  --gt-topic TOPIC    Override the ground-truth topic.
  --sim3              Use Sim3 alignment instead of SE3.
  -h, --help          Show this help.
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
    --out-dir)
      OUT_DIR="$2"
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
    --gt-topic)
      GT_TOPIC="$2"
      shift 2
      ;;
    --sim3)
      SIM3=true
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

mkdir -p "${OUT_DIR}"
RESULT_BAG="${OUT_DIR}/results_bag"
LOG_DIR="${WS_ROOT}/.ros_logs"

rm -rf "${RESULT_BAG}"
mkdir -p "${LOG_DIR}"

set +u
source /opt/ros/humble/setup.bash
source "${INSTALL_SETUP}"
set -u

# Prefer system curl/tiff over any Anaconda copy when launching the composed nodes.
export LD_LIBRARY_PATH="${CERES_LIB_DIR}:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="${LOG_DIR}"

ros2 launch vins euroc_stereo_imu_demo.launch.py \
  config_file:="${CONFIG_FILE}" \
  bag_path:="${BAG_PATH}" \
  play_bag:=true \
  bag_rate:="${BAG_RATE}" \
  use_loop_fusion:="${USE_LOOP_FUSION}" \
  use_rviz:=false \
  record_results:=true \
  record_bag_path:="${RESULT_BAG}" \
  log_dir:="${LOG_DIR}"

ANALYZER="${REPO_ROOT}/scripts/analyze_recorded_estimation_bag.py"
SIM3_FLAG=()
GT_TOPIC_FLAG=()

if [[ "${SIM3}" == true ]]; then
  SIM3_FLAG+=(--sim3)
fi

if [[ -n "${GT_TOPIC}" ]]; then
  GT_TOPIC_FLAG+=(--gt-topic "${GT_TOPIC}")
fi

/usr/bin/python3 "${ANALYZER}" \
  --results-bag "${RESULT_BAG}" \
  --gt-bag "${BAG_PATH}" \
  --traj-topic /vins_estimator/odometry \
  --out-dir "${OUT_DIR}/vins_estimator" \
  "${SIM3_FLAG[@]}" \
  "${GT_TOPIC_FLAG[@]}"

if [[ "${USE_LOOP_FUSION}" == true ]]; then
  /usr/bin/python3 "${ANALYZER}" \
    --results-bag "${RESULT_BAG}" \
    --gt-bag "${BAG_PATH}" \
    --traj-topic /loop_fusion/odometry_rect \
    --out-dir "${OUT_DIR}/loop_fusion" \
    "${SIM3_FLAG[@]}" \
    "${GT_TOPIC_FLAG[@]}"
fi

echo "Recorded bag: ${RESULT_BAG}"
echo "Benchmark output: ${OUT_DIR}"
