EuRoC Demo Workflow
===================

Dataset layout
--------------

The repository now uses two dataset roots:

* ``dataset/`` for original ROS1 ``.bag`` files
* ``dataset_ros2/`` for converted ROS2 bag directories

The launch and tmux helpers expect ROS2 bags.

Typical EuRoC sequence
----------------------

Example path:

.. code-block:: text

   /home/guanrui/ws/vins_fusion_ws/dataset_ros2/machine_hall/MH_01_easy/MH_01_easy

What the demo launches
----------------------

The combined launch file in ``vins_estimator/launch/euroc_stereo_imu_demo.launch.py`` starts the stack in this order:

1. a default static TF from ``world`` to ``world_anchor`` so RViz2 has a fixed frame
2. ``component_container_mt`` as the ROS2 composable-node host
3. ``vins::VinsEstimatorComponent`` using the stereo+IMU EuRoC config
4. ``loop_fusion::LoopFusionComponent`` with the same config, if enabled
5. ``rviz2`` with the packaged visualization file
6. ``ros2 bag play`` after a short startup delay, if enabled
7. ``ros2 bag record`` when result recording is enabled

The estimator and loop-closure code are now launched as ROS2 composable nodes,
which is the ROS2 replacement for the old ROS1 nodelet model. This keeps the
heavy message traffic inside one process when composition is enabled.

When playback finishes, the launch file triggers shutdown automatically.

Outputs
-------

The EuRoC config writes estimator artifacts to ``~/output`` after path expansion. The main files are:

* ``~/output/vio.csv`` for VIO trajectory
* ``~/output/vio_loop.csv`` for loop-corrected trajectory
* ``~/output/pose_graph/`` for saved pose-graph data when loop fusion is enabled

If ``record_results:=true`` is set, the launch file also writes a ROS2 bag that
contains:

* ``/vins_estimator/odometry``
* ``/vins_estimator/path``
* ``/loop_fusion/odometry_rect``
* ``/loop_fusion/pose_graph_path``

What to look for in RViz
------------------------

* ``odometry`` and ``path`` should begin updating quickly after image and IMU playback starts.
* ``keyframe_pose`` density should increase as the estimator matures.
* With loop fusion enabled, loop-corrected pose graph outputs will appear after revisits.

Recommended lab workflow
------------------------

* Start with ``MH_01_easy`` or ``V1_01_easy``.
* Run once with ``use_loop_fusion:=false`` to baseline the VIO path.
* Run again with loop fusion enabled to inspect long-term drift correction.
* Use ``record_results:=true`` when you want a result bag for repeatable benchmarking.
* Save trajectory CSVs before comparing multiple algorithm or parameter variants.

Example launch
--------------

.. code-block:: bash

   cd ~/vins_fusion_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch vins euroc_stereo_imu_demo.launch.py \
     bag_path:=~/vins_fusion_ws/dataset_ros2/machine_hall/MH_01_easy/MH_01_easy \
     play_bag:=true \
     use_loop_fusion:=true \
     use_rviz:=true
