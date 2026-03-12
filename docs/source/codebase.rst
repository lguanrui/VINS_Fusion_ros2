Codebase Guide
==============

Package map
-----------

``camera_models``
  Camera calibration models and utilities reused from the original project.

``vins_estimator``
  The main estimator package, installed as ROS2 package ``vins``.

``loop_fusion``
  Loop detection and pose-graph correction.

``global_fusion``
  Optional GPS/global alignment logic.

Where to start reading
----------------------

For algorithm flow:

* ``vins_estimator/src/rosNodeTest.cpp``: ROS2 subscriptions and main runtime loop
* ``vins_estimator/src/estimator/estimator.cpp``: estimator logic
* ``vins_estimator/src/factor/*.cpp``: residual factors used in Ceres
* ``vins_estimator/src/utility/visualization.cpp``: publishers and trajectory CSV output

For loop closure:

* ``loop_fusion/src/pose_graph_node.cpp``: ROS2 integration and keyframe subscriptions
* ``loop_fusion/src/pose_graph.cpp``: loop constraints and graph optimization
* ``loop_fusion/src/keyframe.cpp``: keyframe data model

For launch and deployment:

* ``vins_estimator/launch/euroc_stereo_imu_demo.launch.py``
* ``scripts/run_euroc_demo.sh``
* ``scripts/run_euroc_tmux.sh``

For evaluation:

* ``scripts/evaluate_slam_trajectory.py``

ROS2 topic flow
---------------

The main estimator publishes local outputs such as:

* ``odometry``
* ``path``
* ``keyframe_pose``
* ``keyframe_point``
* ``extrinsic``

``loop_fusion`` subscribes to the keyframe-related outputs and republishes loop-corrected data products.

Configuration files
-------------------

Configs live under ``config/`` and are now installed with the package. The most important ones for students are:

* ``config/euroc/euroc_stereo_imu_config.yaml``
* ``config/euroc/euroc_mono_imu_config.yaml``
* ``config/kitti_odom/kitti_config00-02.yaml``

When debugging, always check three things first:

* topic names
* image size and calibration files
* output paths and whether the dataset mode matches the config
