VINS-Fusion ROS2
================

This site documents the ROS2 Humble port of VINS-Fusion in this repository. It is written as a lab wiki: start from the quickstart if you want a working demo, or jump to the architecture pages if you need to understand the estimator internals and code layout.

What this documentation is for
------------------------------

* Running the EuRoC example with one command.
* Understanding how the estimator, loop closure, and optional global fusion are split across packages.
* Mapping the original VINS-Fusion ideas to the ROS2 code in this repository.
* Evaluating output trajectories against ground truth with repeatable plots and metrics.

Documentation map
-----------------

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   quickstart
   euroc_demo
   evaluation
   troubleshooting

.. toctree::
   :maxdepth: 2
   :caption: Concepts and Code

   architecture
   codebase

Recommended reading order
-------------------------

1. :doc:`quickstart`
2. :doc:`euroc_demo`
3. :doc:`architecture`
4. :doc:`codebase`
5. :doc:`evaluation`

Repository pointers
-------------------

* Source tree: ``src/vins_fusion``
* Main estimator package: ``vins_estimator``
* Loop closure package: ``loop_fusion``
* One-line EuRoC launcher: ``scripts/run_euroc_demo.sh``
* tmux launcher: ``scripts/run_euroc_tmux.sh``
* Evaluation script: ``scripts/evaluate_slam_trajectory.py``
