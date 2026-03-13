Evaluation
==========

Trajectory evaluation script
----------------------------

The repository includes ``scripts/evaluate_slam_trajectory.py``. It reads:

* a VINS output CSV
* a ground-truth topic from a ROS1 or ROS2 bag

It then:

* aligns the estimate to ground truth
* computes trajectory metrics
* saves ``x/y/z`` versus time plots
* saves a 3D trajectory comparison plot
* writes ``metrics.json``

Recorded-bag benchmark
----------------------

The composable EuRoC workflow adds a second evaluation path:
``scripts/analyze_recorded_estimation_bag.py`` reads estimator trajectories
directly from a recorded ROS2 bag and compares them with the ground truth in the
original dataset bag.

For a one-command run, use:

.. code-block:: bash

   ./src/vins_fusion/scripts/run_euroc_component_benchmark.sh \
     --out-dir ./test_results/euroc_component_benchmark_MH01

This produces:

* ``results_bag/`` with the recorded estimator outputs
* ``vins_estimator/metrics.json`` and plots
* ``loop_fusion/metrics.json`` and plots when loop fusion is enabled

Example
-------

.. code-block:: bash

   /usr/bin/python3 src/vins_fusion/scripts/evaluate_slam_trajectory.py \
     --vins-csv test_results/MH_01_easy_vio_short.csv \
     --bag dataset_ros2/machine_hall/MH_01_easy/MH_01_easy \
     --out-dir test_results/eval/MH_01_easy

Metrics currently reported
--------------------------

APE / ATE translation
  Absolute position error after trajectory alignment. This is the most common high-level summary of global trajectory quality.

Axis RMSE
  RMSE computed independently for ``x``, ``y``, and ``z``.

RPE translation
  Relative pose error over a fixed time delta. This exposes local drift better than global APE alone.

RPE rotation
  Reported when the ground-truth topic contains orientation, for example EuRoC Vicon topics.

Metrics commonly used in SLAM/VIO papers
----------------------------------------

The standard references students should know are:

* TUM-style ``ATE`` and ``RPE`` for trajectory benchmarking.
* KITTI odometry translational drift percentage and rotational drift in degrees per meter, averaged over fixed segment lengths.
* Runtime and success/failure behavior, especially initialization robustness.

Reference links
---------------

* TUM RGB-D tools and metric definitions: https://vision.in.tum.de/data/datasets/rgbd-dataset/tools
* TUM evaluation paper: https://vision.in.tum.de/_media/spezial/bib/sturm12iros.pdf
* KITTI odometry benchmark: https://www.cvlibs.net/datasets/kitti/eval_odometry.php
* ``evo`` trajectory evaluation toolkit: https://github.com/MichaelGrupp/evo

Important caveat for this repository
------------------------------------

The current VINS CSV writer stores timestamps with integer-second precision. The evaluation script reconstructs sub-second ordering heuristically so short-run plots and APE/RPE remain usable, but this is still weaker than logging full ROS timestamps directly.

Another EuRoC-specific caveat is that the converted ``MH_01_easy`` ROS2 bag in
this workspace exposes only ``/leica/position`` as ground truth. That means the
benchmark currently reports translation metrics only for that sequence.
