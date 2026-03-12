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

1. ``vins_node`` using the stereo+IMU EuRoC config
2. ``loop_fusion_node`` with the same config, if enabled
3. ``rviz2`` with the packaged visualization file
4. ``ros2 bag play`` after a short startup delay, if enabled

When playback finishes, the launch file shuts the stack down cleanly.

Outputs
-------

The EuRoC config writes estimator artifacts to ``~/output`` after path expansion. The main files are:

* ``~/output/vio.csv`` for VIO trajectory
* ``~/output/vio_loop.csv`` for loop-corrected trajectory
* ``~/output/pose_graph/`` for saved pose-graph data when loop fusion is enabled

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
* Save trajectory CSVs before comparing multiple algorithm or parameter variants.
