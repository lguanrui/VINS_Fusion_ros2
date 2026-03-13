Architecture
============

System view
-----------

VINS-Fusion here is still the original optimization-based design, but split into ROS2 packages:

* ``vins``: visual-inertial front-end and sliding-window estimator
* ``loop_fusion``: place recognition, keyframe matching, and pose-graph optimization
* ``global_fusion``: optional GPS fusion for datasets that provide it
* ``camera_models``: camera model and calibration support

Composition model
-----------------

The ROS2 port now exposes each major runtime as a composable node:

* ``vins::VinsEstimatorComponent``
* ``loop_fusion::LoopFusionComponent``
* ``global_fusion::GlobalFusionComponent``

This is the ROS2 analogue of ROS1 nodelets. The EuRoC launch file loads the
estimator and loop-fusion components into one ``component_container_mt`` process
with intra-process communication enabled, which reduces message-copy overhead
for the highest-rate topics.

Estimator state
---------------

The estimator tracks a sliding window of poses, velocities, and IMU biases. In practical terms, each state contains:

* body pose
* body velocity
* accelerometer bias
* gyroscope bias
* camera extrinsics relative to the body, depending on configuration

Core estimation stages
----------------------

Feature tracking
  Stereo or mono image streams are tracked frame to frame. Outlier rejection and parallax checks decide whether a frame becomes a keyframe candidate.

Initialization
  The system bootstraps scale, gravity direction, and motion estimates from visual constraints plus IMU information.

Nonlinear optimization
  After initialization, the estimator solves a sliding-window nonlinear least-squares problem with Ceres. This is the main source of the ``odometry`` output.

Marginalization
  Old states are summarized and removed so the window stays bounded while preserving historical information.

Loop closure
  ``loop_fusion`` builds a keyframe database, matches revisits, and optimizes a pose graph on top of the local VIO estimates.

Why the system is split this way
--------------------------------

This split is useful operationally:

* VIO can run alone for local state estimation.
* Loop closure can be enabled only when long-run drift matters.
* Global fusion stays optional and isolated from the core estimator.

Operational note
----------------

The standalone executables are still present as thin wrappers around the same
runner classes, so ``ros2 run`` workflows continue to work. The composable-node
path is the preferred integration path for performance-sensitive ROS2 runs.

Important implementation detail
-------------------------------

The ROS2 port still uses the legacy Ceres local-parameterization API, so the repository vendors Ceres ``1.14.0`` as a submodule for compatibility.
