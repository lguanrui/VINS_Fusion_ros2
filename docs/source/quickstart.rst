Quickstart
==========

Prerequisites
-------------

The repository assumes:

* Ubuntu 22.04
* ROS2 Humble
* A built workspace at ``~/vins_fusion_ws`` or an equivalent colcon workspace
* Bundled Ceres already built with ``./scripts/build_ceres.sh``

Build
-----

From the workspace root:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   source install/setup.bash

One-line EuRoC demo
-------------------

If the converted EuRoC bag exists at the default location
``dataset_ros2/machine_hall/MH_01_easy/MH_01_easy``, the shortest command is:

.. code-block:: bash

   ./src/vins_fusion/scripts/run_euroc_demo.sh

That wrapper launches:

* ``vins_node``
* ``loop_fusion_node``
* ``rviz2``
* ``ros2 bag play`` on the selected bag

Equivalent direct launch invocation
-----------------------------------

.. code-block:: bash

   ros2 launch vins euroc_stereo_imu_demo.launch.py \
     bag_path:=/absolute/path/to/MH_01_easy \
     play_bag:=true

Useful launch arguments
-----------------------

* ``config_file``: choose another YAML config.
* ``bag_path``: point to a different ROS2 bag directory.
* ``play_bag``: set ``false`` to start only the nodes and RViz.
* ``use_loop_fusion``: disable loop closure for baseline VIO runs.
* ``use_rviz``: useful for headless runs or logging-only sessions.
* ``bag_rate``: slow down playback when debugging initialization.

tmux workflow
-------------

When you want each process in its own pane:

.. code-block:: bash

   ./src/vins_fusion/scripts/run_euroc_tmux.sh

This creates a 2x2 tiled session with panes for:

* estimator
* loop fusion
* RViz2
* bag playback

By default the tmux session is attached immediately. Use ``--detach`` if you only want to create it.
