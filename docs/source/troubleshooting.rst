Troubleshooting
===============

Build failures
--------------

Ceres not found
  Run ``git submodule update --init --recursive`` and then ``./scripts/build_ceres.sh`` from the repository root.

``catkin_pkg`` missing during ``colcon build``
  Your shell is probably preferring Anaconda Python. Use system Python first:

  .. code-block:: bash

     export PATH=/usr/bin:/bin:$PATH
     sudo apt install -y python3-catkin-pkg

Runtime failures
----------------

No estimator output file
  The repository now expands ``~/output`` correctly, but older runs may have written to a literal path if they used the pre-fix binaries. Rebuild after pulling the latest changes.

Loop fusion starts but cannot find the vocabulary
  This usually means the package was launched from installed resources without ``support_files/`` being present. Rebuild after the current install rules are applied.

Bag plays but estimator stays idle
  Check that the config file topic names match the bag. For EuRoC stereo+IMU they should be ``/imu0``, ``/cam0/image_raw``, and ``/cam1/image_raw``.

``RTPS_TRANSPORT_SHM Error`` with ``open_and_lock_file failed``
  Fast DDS failed to initialize its shared-memory transport. The most common causes are stale files in ``/dev/shm`` from an older ROS2 run, permission mismatches from mixing ``sudo`` and non-``sudo`` launches, or a restricted shared-memory environment. The system often continues by falling back to another transport, but if startup is unstable:

  * stop all ROS2 processes
  * remove stale Fast DDS shared-memory files
  * relaunch everything as the same user

Composable container needs ``SIGKILL`` on shutdown
  The current benchmark run completes, records the result bag, and writes metrics, but the component container can still hang during teardown and be force-killed by launch. This is a known shutdown issue in the current composition path rather than a data-quality failure.

Documentation build
-------------------

Install docs dependencies:

.. code-block:: bash

   /usr/bin/python3 -m pip install --user -r docs/requirements.txt

Then build:

.. code-block:: bash

   ./scripts/build_docs.sh

The generated HTML is written to ``docs/build/html``.

To check the site locally:

.. code-block:: bash

   xdg-open docs/build/html/index.html

Or serve it over HTTP:

.. code-block:: bash

   cd docs/build/html
   /usr/bin/python3 -m http.server 8000

Then open ``http://127.0.0.1:8000`` in a browser.
