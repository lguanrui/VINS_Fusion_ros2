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

Documentation build
-------------------

Install docs dependencies:

.. code-block:: bash

   /usr/bin/python3 -m pip install --user -r docs/requirements.txt

Then build:

.. code-block:: bash

   ./scripts/build_docs.sh

The generated HTML is written to ``docs/build/html``.
