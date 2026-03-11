#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CERES_SRC="${REPO_ROOT}/third_party/ceres-solver"
CERES_BUILD="${REPO_ROOT}/third_party/ceres-build"
CERES_INSTALL="${REPO_ROOT}/third_party/ceres-install"

if [[ ! -f "${CERES_SRC}/CMakeLists.txt" ]]; then
  echo "Ceres submodule not found at ${CERES_SRC}"
  echo "Run: git submodule update --init --recursive"
  exit 1
fi

cmake -S "${CERES_SRC}" -B "${CERES_BUILD}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CERES_INSTALL}" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DBUILD_SHARED_LIBS=ON \
  -DBUILD_TESTING=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DMINIGLOG=ON \
  -DSUITESPARSE=OFF \
  -DCXSPARSE=OFF \
  -DLAPACK=OFF

cmake --build "${CERES_BUILD}" -- -j"$(nproc)"
cmake --install "${CERES_BUILD}"

echo "Ceres installed to ${CERES_INSTALL}"
