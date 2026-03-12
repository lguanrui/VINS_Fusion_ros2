# VINS-Fusion (ROS2 Humble)

This repository contains a ROS2 Humble port of VINS-Fusion in C++.

The code keeps the original estimator structure (`camera_models`, `vins_estimator`, `loop_fusion`, `global_fusion`) and uses a pinned Ceres submodule to avoid host-specific `/tmp` builds.

## 1. Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- CMake, GCC/G++, OpenCV, Boost, Eigen

Install common dependencies:

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libeigen3-dev libopencv-dev libboost-all-dev \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-tf2 ros-humble-tf2-ros ros-humble-rviz2
```

## 2. Clone Workspace and Submodules

```bash
mkdir -p ~/vins_fusion_ws/src
cd ~/vins_fusion_ws/src
git clone --recursive https://github.com/lguanrui/VINS_Fusion_ros2.git vins_fusion
```

If you already cloned without submodules:

```bash
cd ~/vins_fusion_ws/src/vins_fusion
git submodule update --init --recursive
```

## 3. Build Bundled Ceres (Pinned to 1.14.0)

```bash
cd ~/vins_fusion_ws/src/vins_fusion
./scripts/build_ceres.sh
```

This installs Ceres into:

`~/vins_fusion_ws/src/vins_fusion/third_party/ceres-install`

All ROS2 packages in this repo automatically fall back to that path if system Ceres is not found.

## 4. Build ROS2 Packages

```bash
cd ~/vins_fusion_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. One-Line EuRoC Demo

If your converted EuRoC bag is available under the standard workspace layout, the fastest path is:

```bash
cd ~/vins_fusion_ws
./src/vins_fusion/scripts/run_euroc_demo.sh
```

To launch the same stack directly with ROS2:

```bash
cd ~/vins_fusion_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch vins euroc_stereo_imu_demo.launch.py \
  bag_path:=~/vins_fusion_ws/dataset_ros2/machine_hall/MH_01_easy/MH_01_easy \
  play_bag:=true
```

This launch file starts:

- `vins_node`
- `loop_fusion_node`
- `rviz2`
- `ros2 bag play`

For pane-based debugging with `tmux`:

```bash
cd ~/vins_fusion_ws
./src/vins_fusion/scripts/run_euroc_tmux.sh
```

Useful options:

- `./src/vins_fusion/scripts/run_euroc_demo.sh --bag /absolute/path/to/ros2_bag`
- `./src/vins_fusion/scripts/run_euroc_demo.sh --no-loop`
- `./src/vins_fusion/scripts/run_euroc_tmux.sh --session vins-euroc-mh03 --bag /absolute/path/to/ros2_bag`

## 6. Other Dataset Entry Points

### 6.1 KITTI Odometry

```bash
source ~/vins_fusion_ws/install/setup.bash
ros2 run vins kitti_odom_test ~/vins_fusion_ws/src/vins_fusion/config/kitti_odom/kitti_config00-02.yaml <KITTI_SEQUENCE_PATH>
```

### 6.2 KITTI GPS Fusion

```bash
source ~/vins_fusion_ws/install/setup.bash
ros2 run vins kitti_gps_test ~/vins_fusion_ws/src/vins_fusion/config/kitti_raw/kitti_10_03_config.yaml <KITTI_RAW_PATH>
ros2 run global_fusion global_fusion_node
```

## 7. Documentation Site

The repository includes a Sphinx site styled with Furo:

```bash
cd ~/vins_fusion_ws/src/vins_fusion
/usr/bin/python3 -m pip install --user -r docs/requirements.txt
./scripts/build_docs.sh
```

The built HTML is written to:

`~/vins_fusion_ws/src/vins_fusion/docs/build/html`

Main docs entry points:

- `docs/source/index.rst`
- `docs/source/quickstart.rst`
- `docs/source/euroc_demo.rst`
- `docs/source/architecture.rst`
- `docs/source/codebase.rst`
- `docs/source/evaluation.rst`

## 8. Troubleshooting

- `CeresConfig.cmake not found`:
  1. `cd ~/vins_fusion_ws/src/vins_fusion`
  2. `git submodule update --init --recursive`
  3. `./scripts/build_ceres.sh`
  4. rebuild with `colcon build --symlink-install`

- If you reorganize the workspace, keep this repository under `.../src/vins_fusion` and rerun `source /opt/ros/humble/setup.bash` before `colcon build`.

- `ModuleNotFoundError: No module named catkin_pkg` during `colcon build`:
  1. ensure ROS uses system Python (`export PATH=/usr/bin:/bin:$PATH`)
  2. install package parser: `sudo apt install -y python3-catkin-pkg`

## 9. Acknowledgements

The original VINS-Fusion was developed by Tong Qin, Shaozu Cao, Jie Pan, Peiliang Li, and Shaojie Shen (HKUST Aerial Robotics Group).

Core upstream projects used by VINS-Fusion include:

- Ceres Solver
- DBoW2
- CamOdoCal camera models
- GeographicLib

## 10. License

GPLv3. See [LICENCE](./LICENCE).
