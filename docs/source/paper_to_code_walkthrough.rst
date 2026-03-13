Paper-to-Code Walkthrough
=========================

This page connects the estimator theory in two core papers to the
``vins_estimator`` implementation in this repository:

* `VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator <https://arxiv.org/abs/1708.03852>`_
* `Online Temporal Calibration for Monocular Visual-Inertial Systems <https://arxiv.org/abs/1808.00692>`_

The goal is to help a first-year master's student answer two questions at the
same time:

* What mathematical problem is VINS solving?
* Where does each part of that math live in the code?

Scope
-----

This walkthrough focuses on the local visual-inertial estimator under
``src/vins_fusion/vins_estimator``. The ``loop_fusion`` and ``global_fusion``
packages sit on top of this local estimator and are covered better by
:doc:`architecture` and :doc:`codebase`.

.. contents:: On this page
   :local:
   :depth: 2

Repository Roadmap
------------------

If you want the shortest path from the papers to the code, these are the most
important files.

.. list-table:: Main estimator files
   :widths: 28 32 40
   :header-rows: 1

   * - Topic
     - Main files
     - Why they matter
   * - ROS2 entry points
     - ``vins_estimator/src/rosNodeTest.cpp``, ``vins_estimator/src/runners.hpp``
     - Subscribes to IMU and image topics, synchronizes stereo images, and feeds the estimator.
   * - Estimator orchestration
     - ``vins_estimator/src/estimator/estimator.h``, ``vins_estimator/src/estimator/estimator.cpp``
     - Owns the sliding window, initialization, optimization, marginalization, and state updates.
   * - Configuration
     - ``vins_estimator/src/estimator/parameters.h``, ``vins_estimator/src/estimator/parameters.cpp``
     - Defines window size, parameter block sizes, camera/IMU config loading, and the ``td`` option.
   * - Feature front-end
     - ``vins_estimator/src/featureTracker/feature_tracker.cpp``
     - Tracks points with KLT, computes image-plane velocity, and packages feature observations.
   * - Feature management
     - ``vins_estimator/src/estimator/feature_manager.h``, ``vins_estimator/src/estimator/feature_manager.cpp``
     - Stores tracks across the window, chooses keyframes, triangulates landmarks, and manages inverse depth.
   * - IMU preintegration
     - ``vins_estimator/src/factor/integration_base.h``, ``vins_estimator/src/factor/imu_factor.h``
     - Implements midpoint preintegration, covariance propagation, bias correction, and the IMU residual.
   * - Visual factors
     - ``vins_estimator/src/factor/projectionTwoFrameOneCamFactor.cpp``, ``projectionOneFrameTwoCamFactor.cpp``, ``projectionTwoFrameTwoCamFactor.cpp``
     - Implements reprojection residuals for monocular, stereo, and online temporal calibration.
   * - Initialization
     - ``vins_estimator/src/initial/initial_sfm.cpp``, ``initial_aligment.cpp``
     - Performs vision-only SfM, gyroscope bias solve, gravity estimation, and scale recovery.
   * - Marginalization
     - ``vins_estimator/src/factor/marginalization_factor.h``, ``vins_estimator/src/factor/marginalization_factor.cpp``
     - Converts old measurements into a Schur-complement prior so the window stays fixed size.

Runtime Data Flow
-----------------

At runtime the estimator follows this chain:

1. ``img0_callback()`` and ``img1_callback()`` in ``rosNodeTest.cpp`` buffer images.
2. ``sync_process()`` forms a mono or stereo frame and calls ``Estimator::inputImage()``.
3. ``imu_callback()`` sends each inertial sample to ``Estimator::inputIMU()``.
4. ``FeatureTracker::trackImage()`` converts the new image into tracked features.
5. ``Estimator::processMeasurements()`` pulls one image packet and the matching IMU interval.
6. ``Estimator::processIMU()`` grows the current preintegration and predicts the newest state.
7. ``Estimator::processImage()`` decides whether the frame is a keyframe, initializes if needed, otherwise runs ``optimization()``.
8. ``optimization()`` builds a Ceres problem from IMU factors, visual factors, extrinsic blocks, feature inverse depth blocks, the time offset block, and the marginalization prior.
9. ``slideWindow()`` removes one old state and turns it into a new prior.

This is the most important mental model for reading the code: the repository is
not a collection of unrelated classes. It is one repeated loop that turns raw
measurements into a small nonlinear least-squares problem.

State Vector and Parameter Blocks
---------------------------------

The VINS-Mono paper defines the local estimator state at keyframe :math:`k` as

.. math::

   \mathbf{x}_k =
   \left[
     \mathbf{p}^w_{b_k},
     \mathbf{v}^w_{b_k},
     \mathbf{q}^w_{b_k},
     \mathbf{b}^a_k,
     \mathbf{b}^g_k
   \right].

The repository extends that state in exactly the way the temporal calibration
paper suggests: it keeps camera-IMU extrinsics, feature inverse depths, and a
global time offset variable:

.. math::

   \mathbf{x}^{(i)}_{bc} =
   \left[
     \mathbf{p}^b_{c_i},
     \mathbf{q}^b_{c_i}
   \right],
   \qquad
   \lambda_l = \rho_l^{-1},

.. math::

   \mathcal{X} =
   \left\{
     \mathbf{x}_0, \ldots, \mathbf{x}_N,
     \mathbf{x}^{(0)}_{bc}, \mathbf{x}^{(1)}_{bc},
     \lambda_0, \ldots, \lambda_M,
     t_d
   \right\}.

The most important code cross-check is that the mathematical state above has a
direct one-to-one storage layout in ``Estimator``.

.. list-table:: Math to code mapping
   :widths: 32 34 34
   :header-rows: 1

   * - Mathematical object
     - Runtime storage
     - Ceres parameter block
   * - Keyframe pose :math:`(\mathbf{p}, \mathbf{q})`
     - ``Ps[i]``, ``Rs[i]``
     - ``para_Pose[i]`` with ``SIZE_POSE = 7``
   * - Keyframe velocity and biases
     - ``Vs[i]``, ``Bas[i]``, ``Bgs[i]``
     - ``para_SpeedBias[i]`` with ``SIZE_SPEEDBIAS = 9``
   * - Camera extrinsics
     - ``tic[i]``, ``ric[i]``
     - ``para_Ex_Pose[i]``
   * - Feature inverse depth
     - Stored as depth in ``FeaturePerId::estimated_depth``
     - Stored as inverse depth in ``para_Feature[idx][0]``
   * - Global time offset
     - ``td``
     - ``para_Td[0][0]``

Two implementation details are worth remembering:

* ``WINDOW_SIZE`` is ``10`` in ``parameters.h``, so the estimator stores
  ``WINDOW_SIZE + 1`` keyframe states.
* The optimization variable for a landmark is inverse depth, but the feature
  manager often stores and reasons about positive depth. ``Estimator::vector2double()``
  and ``FeatureManager::setDepth()`` convert between these two views.

From ROS Messages to Feature Measurements
-----------------------------------------

The visual front-end does more than just return pixel coordinates. Each tracked
feature becomes a 7-vector:

.. math::

   \mathbf{y}_{l,k} =
   \begin{bmatrix}
   x & y & 1 & u & v & \dot{u} & \dot{v}
   \end{bmatrix}^{\top}.

In code this is the ``Eigen::Matrix<double, 7, 1>`` named
``xyz_uv_velocity`` in ``FeatureTracker::trackImage()``.

The meaning of each entry is:

* ``x, y, 1``: undistorted normalized image coordinates after
  ``Camera::liftProjective()``
* ``u, v``: raw pixel coordinates in the current image
* ``dot_u, dot_v``: image-plane velocity estimated from the previous frame

Those 7-vectors are grouped into a
``map<int, vector<pair<int, Matrix<double, 7, 1>>>>``:

* the map key is ``feature_id``
* the ``pair.first`` is ``camera_id`` (left or right)
* the vector contains one observation for mono, or two for stereo

That observation packet is then converted into ``FeaturePerFrame`` objects in
``FeatureManager::addFeatureCheckParallax()``.

Visual Front-end
----------------

The VINS-Mono paper says the front-end should do four things well:

* keep long tracks alive,
* add new features when coverage drops,
* reject obvious outliers,
* decide when a frame should enter the optimization window as a keyframe.

This repository follows that design almost literally.

Tracking and new point creation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``FeatureTracker::trackImage()`` does the following:

1. Track old features with ``cv::calcOpticalFlowPyrLK``.
2. Optionally perform forward-backward consistency checking when ``FLOW_BACK`` is enabled.
3. Drop tracks that leave the image border.
4. Build a spatial mask that prefers long-lived tracks.
5. Detect new corners with ``cv::goodFeaturesToTrack``.
6. Undistort all surviving tracks.
7. Compute point velocities with ``FeatureTracker::ptsVelocity()``.
8. If stereo is enabled, track left-image features into the right image as well.

Image-plane velocity and the temporal calibration paper
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The temporal calibration paper introduces a simple but powerful model:
over a short interval, a feature moves approximately at constant velocity on
the image plane:

.. math::

   \mathbf{V}^k_l =
   \frac{
     \begin{bmatrix}
     u^{k+1}_l - u^k_l \\
     v^{k+1}_l - v^k_l
     \end{bmatrix}
   }{
     t_{k+1} - t_k
   }.

The code implementation is direct:

* ``FeatureTracker::ptsVelocity()`` computes
  ``(pts[i] - prev_pts[id]) / dt``.
* ``FeatureTracker::trackImage()`` stores that value into
  ``xyz_uv_velocity(5)`` and ``xyz_uv_velocity(6)``.
* ``FeaturePerFrame`` copies those numbers into ``velocity`` and
  ``velocityRight``.

This is one of the cleanest paper-to-code correspondences in the whole
repository.

Keyframe selection
~~~~~~~~~~~~~~~~~~

The paper describes keyframe selection using average parallax and tracking
quality. The code implements that idea in ``FeatureManager::addFeatureCheckParallax()``.

Concretely, the function tends to force a keyframe when:

* there are too few frames in the window,
* too few tracks survived,
* too few long tracks survived,
* or too many features are newly created.

Otherwise it computes an average parallax across tracks seen in the two newest
frames and compares it against ``MIN_PARALLAX``.

.. note::

   The paper says gyroscope rotation can be used to compensate parallax for
   keyframe selection. In this repository the rotation-compensated line inside
   ``FeatureManager::compensatedParallax2()`` is commented out and
   ``p_i_comp = p_i`` is used instead. So the current ROS2 port uses the
   simpler uncompensated version.

IMU Model and Preintegration
----------------------------

The IMU model in the paper is

.. math::

   \hat{\mathbf{a}}_t =
   \mathbf{a}_t + \mathbf{b}^a_t + \mathbf{R}^t_w \mathbf{g}^w + \mathbf{n}_a,
   \qquad
   \hat{\boldsymbol{\omega}}_t =
   \boldsymbol{\omega}_t + \mathbf{b}^g_t + \mathbf{n}_g,

with bias random walks

.. math::

   \dot{\mathbf{b}}^a_t = \mathbf{n}_{b_a},
   \qquad
   \dot{\mathbf{b}}^g_t = \mathbf{n}_{b_g}.

The code loads the corresponding noise standard deviations from the YAML file
through ``readParameters()``:

* ``acc_n``, ``gyr_n`` for measurement noise
* ``acc_w``, ``gyr_w`` for bias random walk

These values populate the continuous-time noise matrix in
``IntegrationBase::IntegrationBase()``.

Preintegrated quantities
~~~~~~~~~~~~~~~~~~~~~~~~

Between two image times the paper defines the IMU preintegrated motion terms
:math:`\alpha`, :math:`\beta`, and :math:`\gamma`. In code they appear as

* ``delta_p`` for :math:`\alpha`
* ``delta_v`` for :math:`\beta`
* ``delta_q`` for :math:`\gamma`

inside ``IntegrationBase``.

The repository uses midpoint integration in
``IntegrationBase::midPointIntegration()``:

.. math::

   \Delta \mathbf{q}_{k+1} =
   \Delta \mathbf{q}_{k} \otimes
   \delta q \left(
     \frac{1}{2}
     (\boldsymbol{\omega}_0 + \boldsymbol{\omega}_1 - 2\mathbf{b}^g)
     \Delta t
   \right),

.. math::

   \Delta \mathbf{v}_{k+1} =
   \Delta \mathbf{v}_{k} +
   \frac{1}{2}
   \left(
     \Delta \mathbf{R}_{k}(\mathbf{a}_0 - \mathbf{b}^a) +
     \Delta \mathbf{R}_{k+1}(\mathbf{a}_1 - \mathbf{b}^a)
   \right)\Delta t,

.. math::

   \Delta \mathbf{p}_{k+1} =
   \Delta \mathbf{p}_{k} +
   \Delta \mathbf{v}_{k}\Delta t +
   \frac{1}{2}
   \left(
     \frac{\Delta \mathbf{v}_{k+1} - \Delta \mathbf{v}_k}{\Delta t}
   \right)\Delta t^2.

The exact code formulas are written in terms of ``un_acc_0``, ``un_acc_1``,
``un_acc``, and ``un_gyr``. The Jacobian ``F`` and noise injection matrix
``V`` are also updated there, which is why ``midPointIntegration()`` is much
longer than the three equations above.

Bias correction
~~~~~~~~~~~~~~~

The VINS-Mono paper uses first-order bias correction instead of recomputing the
full integration every optimization step. The code matches that exactly.

``IntegrationBase::evaluate()`` forms

.. math::

   \hat{\alpha}_{ij}(\delta \mathbf{b}) =
   \hat{\alpha}_{ij} +
   \mathbf{J}_{\alpha b_a}\delta \mathbf{b}^a +
   \mathbf{J}_{\alpha b_g}\delta \mathbf{b}^g,

.. math::

   \hat{\beta}_{ij}(\delta \mathbf{b}) =
   \hat{\beta}_{ij} +
   \mathbf{J}_{\beta b_a}\delta \mathbf{b}^a +
   \mathbf{J}_{\beta b_g}\delta \mathbf{b}^g,

.. math::

   \hat{\gamma}_{ij}(\delta \mathbf{b}^g) =
   \hat{\gamma}_{ij} \otimes
   \delta q \left( \mathbf{J}_{\gamma b_g}\delta \mathbf{b}^g \right).

Here the Jacobians are stored in the 15x15 matrix ``jacobian`` and the needed
blocks are pulled out as
``dp_dba``, ``dp_dbg``, ``dq_dbg``, ``dv_dba``, and ``dv_dbg``.

IMU residual
~~~~~~~~~~~~

The residual implemented by ``IMUFactor::Evaluate()`` is the 15-dimensional
vector

.. math::

   \mathbf{r}_{imu} =
   \begin{bmatrix}
   \mathbf{R}_i^\top
   \left(
     \mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t + \frac{1}{2}\mathbf{g}\Delta t^2
   \right)
   - \hat{\alpha}_{ij}(\delta \mathbf{b}) \\
   2\,\mathrm{vec}
   \left(
     \hat{\gamma}_{ij}(\delta \mathbf{b}^g)^{-1}
     \otimes
     (\mathbf{q}_i^{-1}\otimes\mathbf{q}_j)
   \right) \\
   \mathbf{R}_i^\top
   \left(
     \mathbf{v}_j - \mathbf{v}_i + \mathbf{g}\Delta t
   \right)
   - \hat{\beta}_{ij}(\delta \mathbf{b}) \\
   \mathbf{b}^a_j - \mathbf{b}^a_i \\
   \mathbf{b}^g_j - \mathbf{b}^g_i
   \end{bmatrix}.

This is exactly what ``IntegrationBase::evaluate()`` returns before
``IMUFactor`` whitens it with the inverse covariance square root.

Initialization
--------------

Initialization is the hardest conceptual part of monocular VIO because the
camera-only structure is only known up to scale, while inertial integration is
metric but drift-prone. The paper and the code solve this by first building a
vision-only reconstruction and then aligning it to IMU preintegration.

Vision-only SfM
~~~~~~~~~~~~~~~

``Estimator::initialStructure()`` runs the vision-only stage. The chain is:

1. ``Estimator::relativePose()`` finds a frame pair with enough correspondences and parallax.
2. ``MotionEstimator::solveRelativeRT()`` recovers a relative pose with the five-point method.
3. ``GlobalSFM::construct()`` triangulates points, solves additional poses by PnP, and runs a full vision-only bundle adjustment.
4. The result is stored into ``all_image_frame`` as up-to-scale camera poses.

This is the code realization of the paper's statement: "build an up-to-scale
visual structure first, then align it with the IMU."

Gyroscope bias calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~

The paper solves gyroscope bias before scale and gravity. The code does this in
``solveGyroscopeBias()``.

For each consecutive pair of frames, it compares

* the relative rotation from visual SfM, and
* the relative rotation predicted by IMU preintegration.

The linear least-squares problem is

.. math::

   \delta \mathbf{b}^g =
   \arg \min
   \sum_k
   \left\|
   2\,\mathrm{vec}
   \left(
     \hat{\gamma}_{k,k+1}^{-1}
     \otimes
     (\mathbf{q}_{k}^{-1}\otimes\mathbf{q}_{k+1})
   \right)
   \right\|^2.

In code the normal equations are assembled as ``A += tmp_A.transpose() * tmp_A``
and ``b += tmp_A.transpose() * tmp_b``. Then all window biases ``Bgs[i]`` are
shifted by the solved ``delta_bg`` and every preintegration is repropagated.

Velocity, gravity, and scale alignment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After gyroscope bias is fixed, the paper solves for per-frame velocity, gravity
direction, and metric scale. The code path is

* ``VisualIMUAlignment()``
* ``LinearAlignment()``
* ``RefineGravity()``

The unknown vector is

.. math::

   \mathbf{X}_I =
   \left[
     \mathbf{v}_0,\mathbf{v}_1,\ldots,\mathbf{v}_N,\mathbf{g},s
   \right].

For two consecutive frames, the paper writes

.. math::

   \hat{\alpha}_{k,k+1} =
   \mathbf{R}^{b_k}_{c_0}
   \left(
     s(\bar{\mathbf{p}}_{k+1} - \bar{\mathbf{p}}_k)
     + \frac{1}{2}\mathbf{g}\Delta t_k^2
     - \mathbf{R}^{c_0}_{b_k}\mathbf{v}_k \Delta t_k
   \right),

.. math::

   \hat{\beta}_{k,k+1} =
   \mathbf{R}^{b_k}_{c_0}
   \left(
     \mathbf{R}^{c_0}_{b_{k+1}}\mathbf{v}_{k+1}
     + \mathbf{g}\Delta t_k
     - \mathbf{R}^{c_0}_{b_k}\mathbf{v}_k
   \right).

``LinearAlignment()`` builds these equations into the block matrix
``tmp_A`` and right-hand side ``tmp_b`` for each consecutive frame pair, then
solves the stacked least-squares system with ``A.ldlt().solve(b)``.

Gravity refinement
~~~~~~~~~~~~~~~~~~

The paper notes that gravity magnitude is known, so only 2 DOF should be
optimized. ``RefineGravity()`` implements exactly this tangent-space refinement.

It builds a basis ``lxly = TangentBasis(g0)`` and updates gravity by

.. math::

   \mathbf{g} \leftarrow
   \left(
     \mathbf{g}_0 + \mathbf{B}\,\delta \mathbf{g}
   \right)
   \frac{\|\mathbf{g}_{true}\|}{\|\mathbf{g}_0 + \mathbf{B}\,\delta \mathbf{g}\|},

where :math:`\mathbf{B}` is the 3x2 tangent basis.

Completing initialization in code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once alignment succeeds, ``Estimator::visualInitialAlign()``:

* copies aligned poses from ``all_image_frame`` into ``Ps[]`` and ``Rs[]``
* rescales translations to metric units
* writes per-frame velocities into ``Vs[]``
* rotates the whole solution so gravity points along the world z-axis
* clears and retriangulates feature depths in metric scale

After that, ``solver_flag`` changes from ``INITIAL`` to ``NON_LINEAR`` and the
normal sliding-window estimator takes over.

Initialization branches in this repository
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The two papers studied here mainly explain the monocular-plus-IMU estimator.
The repository keeps that branch, but ``Estimator::processImage()`` actually
contains three initialization modes:

* ``!STEREO && USE_IMU``: the full VINS-Mono initialization described above
* ``STEREO && USE_IMU``: initialize poses by PnP and triangulation, solve gyroscope bias, repropagate IMU terms, then optimize
* ``STEREO && !USE_IMU``: initialize visually with PnP and triangulation, then optimize

So if you are reading this repository as ``vins_fusion`` rather than strictly
as ``VINS-Mono``, keep in mind that the monocular initialization math is the
main theoretical story, but not the only startup path implemented in code.

Sliding-window Nonlinear Optimization
-------------------------------------

Once initialized, the estimator repeatedly solves a bounded nonlinear least
squares problem. The VINS-Mono paper writes the MAP objective as

.. math::

   \min_{\mathcal{X}}
   \left\|
   \mathbf{r}_p - \mathbf{H}_p \mathcal{X}
   \right\|^2 +
   \sum_{k \in \mathcal{B}}
   \left\|
   \mathbf{r}_B(\hat{\mathbf{z}}_{k,k+1}, \mathcal{X})
   \right\|^2_{\mathbf{P}^{k}_{k+1}} +
   \sum_{(l,j) \in \mathcal{C}}
   \rho\left(
     \left\|
     \mathbf{r}_C(\hat{\mathbf{z}}^j_l, \mathcal{X})
     \right\|^2_{\mathbf{P}^j_l}
   \right).

The temporal calibration paper adds :math:`t_d` to the state and uses the same
optimization pattern. ``Estimator::optimization()`` is the direct code
translation:

* ``vector2double()`` serializes the current state into parameter blocks.
* Every pose block gets ``PoseLocalParameterization``.
* IMU factors are added between consecutive frames.
* Visual factors are added for every track observed at least four times.
* The previous marginalization prior is added as ``MarginalizationFactor``.
* A Huber loss is used for visual residuals.
* Ceres solves the problem with ``DENSE_SCHUR`` and ``DOGLEG``.
* ``double2vector()`` maps the optimized blocks back into ``Ps``, ``Rs``, ``Vs``, biases, extrinsics, feature depth, and ``td``.

The mathematical idea and the code architecture are therefore the same:
the estimator is a bundle adjustment over a fixed-lag window.

Visual Residuals and the Online Time Offset
-------------------------------------------

This repository implements the most important contribution of the temporal
calibration paper: treat the camera-IMU time offset as a state variable and let
visual reprojection residuals depend on it.

Paper model
~~~~~~~~~~~

The paper defines

.. math::

   t_{imu} = t_{cam} + t_d,

and compensates a feature observation with its image-plane velocity:

.. math::

   \mathbf{z}_l^k(t_d) =
   \mathbf{z}_l^k + t_d \mathbf{V}_l^k.

For inverse-depth parametrization, the reprojection residual becomes

.. math::

   \mathbf{e}_{l}^{j} =
   \mathbf{z}_l^j(t_d) -
   \pi
   \left(
     \mathbf{R}_{c_j}^{w\top}
     \left(
       \mathbf{R}_{c_i}^w \lambda_i^{-1}\mathbf{z}_l^i(t_d) +
       \mathbf{p}_{c_i}^w - \mathbf{p}_{c_j}^w
     \right)
   \right).

Incremental implementation in code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The code uses a slightly more engineering-friendly version of the same idea.
Each observation stores the time offset value that was current when that
observation entered the system:

* ``FeaturePerFrame::cur_td``
* constructor call in ``FeatureManager::addFeatureCheckParallax(frame_count, image, td)``

Because of that, the factor only applies the *incremental* change in time
offset:

.. math::

   \tilde{\mathbf{z}}_i(t_d) =
   \mathbf{z}_i - (t_d - t_{d,i})\mathbf{v}_i,
   \qquad
   \tilde{\mathbf{z}}_j(t_d) =
   \mathbf{z}_j - (t_d - t_{d,j})\mathbf{v}_j.

This is exactly how the temporal calibration paper's "coarse to fine"
compensation idea appears in code. The paper says: shift future timestamps
using the current best :math:`t_d`, then keep estimating the remaining error.
The code stores ``td_i`` and ``td_j`` per observation so each factor only needs
to optimize the remaining correction.

Monocular reprojection factor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The core factor is ``ProjectionTwoFrameOneCamFactor``. Its geometry is

.. math::

   \mathbf{p}^{c_i}_l = \frac{\tilde{\mathbf{z}}_i(t_d)}{\rho_l},

.. math::

   \mathbf{p}^{b_i}_l =
   \mathbf{R}_{ic}\mathbf{p}^{c_i}_l + \mathbf{t}_{ic},
   \qquad
   \mathbf{p}^{w}_l =
   \mathbf{R}_i \mathbf{p}^{b_i}_l + \mathbf{p}_i,

.. math::

   \mathbf{p}^{c_j}_l =
   \mathbf{R}_{ic}^{\top}
   \left(
     \mathbf{R}_j^{\top}(\mathbf{p}^w_l - \mathbf{p}_j) - \mathbf{t}_{ic}
   \right).

With the default build flags in this repository, the residual is the
2-dimensional normalized-plane error

.. math::

   \mathbf{r}_{ij} =
   \left(
     \frac{\mathbf{p}^{c_j}_l}{p^{c_j}_{l,z}}
   \right)_{xy}
   - \tilde{\mathbf{z}}_{j,xy}(t_d).

That is exactly what you see in
``ProjectionTwoFrameOneCamFactor::Evaluate()``:

* ``pts_i_td`` and ``pts_j_td`` are the time-compensated observations
* ``inv_dep_i`` is the optimized inverse depth
* ``pts_camera_i -> pts_imu_i -> pts_w -> pts_imu_j -> pts_camera_j`` is the frame transform chain
* ``residual = (pts_camera_j / dep_j).head<2>() - pts_j_td.head<2>()`` is the final residual

Stereo variants
~~~~~~~~~~~~~~~

The same math is reused in two additional factors:

* ``ProjectionOneFrameTwoCamFactor`` for left-right reprojection in the same time step
* ``ProjectionTwoFrameTwoCamFactor`` for left-to-right reprojection across time

These appear in ``Estimator::optimization()`` depending on whether the current
track has a right-camera observation and whether the two observations happen at
the same time or different times.

How the time offset enters the optimization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``Estimator::optimization()`` always adds ``para_Td`` as a parameter block, but
it freezes that block unless both of the following are true:

* ``ESTIMATE_TD`` is enabled in the YAML config
* the motion magnitude is large enough, checked by ``Vs[0].norm() >= 0.2``

This makes sense mathematically: if the platform is barely moving, image-plane
velocity is too small to identify a time shift.

How the runtime compensates timestamps
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The temporal calibration paper recommends updating future timestamps after each
optimization. The repository does that in two places:

* ``Estimator::processMeasurements()`` uses ``curTime = feature.first + td`` before selecting the IMU interval for the image.
* ``Estimator::updateLatestStates()`` sets ``latest_time = Headers[frame_count] + td`` before forward propagating with new IMU samples.

So the optimized ``td`` does not just live inside a residual. It changes how
future data are aligned.

.. note::

   The VINS-Mono paper presents the visual residual on the tangent plane of a
   unit sphere. The code still contains that version behind
   ``#ifdef UNIT_SPHERE_ERROR``, but the macro is commented out in
   ``parameters.h``. The default build therefore uses the simpler normalized
   image-plane residual.

Triangulation and Inverse Depth
-------------------------------

The optimizer needs a starting value for every landmark depth. That logic lives
in ``FeatureManager::triangulate()``.

The function uses three cases:

* stereo case: triangulate from left-right rays in the same time step
* simple monocular case: triangulate from the first two temporal observations
* general case: triangulate from all observations with an SVD linear system

After triangulation:

* ``FeaturePerId::estimated_depth`` stores positive depth
* ``FeatureManager::getDepthVector()`` converts it to inverse depth for Ceres
* ``FeatureManager::setDepth()`` converts optimized inverse depth back to depth

This is why the code sometimes talks about depth and sometimes about inverse
depth. The estimator stores what is convenient for initialization and debugging,
but optimizes what is numerically convenient for monocular BA.

Marginalization and Window Sliding
----------------------------------

The sliding window stays constant size because old states are removed and turned
into a prior. The paper says this is done with the Schur complement, and the
code does exactly that.

There are two modes:

* ``MARGIN_OLD``: the newest image is a keyframe, so the oldest state is marginalized
* ``MARGIN_SECOND_NEW``: the newest image is not a keyframe, so the second newest frame is dropped while keeping the chained IMU information

The core implementation is:

* ``Estimator::optimization()`` decides which residual blocks belong to the prior update.
* ``MarginalizationInfo::preMarginalize()`` evaluates residuals and Jacobians at the current linearization point.
* ``MarginalizationInfo::marginalize()`` builds the normal equations and applies the Schur complement.
* ``MarginalizationFactor`` exposes the resulting linearized prior as one more Ceres residual block in the next optimization.

Mathematically, if we partition the Hessian as

.. math::

   \mathbf{H} =
   \begin{bmatrix}
   \mathbf{H}_{mm} & \mathbf{H}_{mr} \\
   \mathbf{H}_{rm} & \mathbf{H}_{rr}
   \end{bmatrix},
   \qquad
   \mathbf{b} =
   \begin{bmatrix}
   \mathbf{b}_{m} \\
   \mathbf{b}_{r}
   \end{bmatrix},

then marginalizing the old variables gives

.. math::

   \mathbf{H}' =
   \mathbf{H}_{rr} -
   \mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{H}_{mr},
   \qquad
   \mathbf{b}' =
   \mathbf{b}_{r} -
   \mathbf{H}_{rm}\mathbf{H}_{mm}^{-1}\mathbf{b}_{m}.

``MarginalizationInfo::marginalize()`` implements this literally with
``Arr - Arm * Amm_inv * Amr`` and ``brr - Arm * Amm_inv * bmm``.

Paper-to-Code Differences Worth Knowing
---------------------------------------

Most of the core estimator matches the two papers very closely. The differences
below are the ones students usually stumble over when they compare the paper to
the source.

* The paper writes the visual residual on the unit sphere. The default code path uses normalized-plane reprojection because ``UNIT_SPHERE_ERROR`` is disabled.
* The temporal calibration paper writes :math:`\mathbf{z}(t_d) = \mathbf{z} + t_d \mathbf{V}`. The code stores the previous offset with each measurement and therefore uses the incremental form ``td - td_i`` and ``td - td_j``.
* The paper describes gyroscope-compensated parallax for keyframe selection. The current ROS2 port does not apply that compensation because the line is commented out in ``FeatureManager::compensatedParallax2()``.
* The repository contains optional online extrinsic calibration support through ``ESTIMATE_EXTRINSIC`` and ``InitialEXRotation``. That is related to the broader VINS family, but it is not the main topic of the two papers studied here.
* ``Estimator::failureDetection()`` currently returns ``false`` immediately, so the failure-recovery checks written below that line are effectively disabled in this port.

How to Study This Codebase Efficiently
--------------------------------------

If you are learning this system for the first time, read the code in this order:

1. ``rosNodeTest.cpp`` to see how data enters the estimator.
2. ``feature_tracker.cpp`` to understand what a "feature measurement" means in this codebase.
3. ``estimator.cpp`` from ``processMeasurements()`` to ``processImage()`` to understand the control flow.
4. ``integration_base.h`` and ``imu_factor.h`` to connect the IMU equations to code.
5. The three ``projection*.cpp`` files to understand the visual residuals.
6. ``initial_sfm.cpp`` and ``initial_aligment.cpp`` to understand initialization.
7. ``marginalization_factor.cpp`` only after the previous steps make sense.

Two small experiments help a lot:

* Run EuRoC once with ``estimate_td: 0`` and once with ``estimate_td: 1`` and compare the logged ``td`` value and final trajectory.
* Put a debugger or temporary logs in ``ProjectionTwoFrameOneCamFactor::Evaluate()`` and follow one landmark from its stored ``velocity`` and ``cur_td`` to the final residual.

References
----------

Primary papers:

* `VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator <https://arxiv.org/abs/1708.03852>`_
* `VINS-Mono DOI page <https://doi.org/10.1109/TRO.2018.2853729>`_
* `Online Temporal Calibration for Monocular Visual-Inertial Systems <https://arxiv.org/abs/1808.00692>`_

Repository files to keep open while reading this page:

* ``vins_estimator/src/estimator/estimator.cpp``
* ``vins_estimator/src/factor/integration_base.h``
* ``vins_estimator/src/factor/imu_factor.h``
* ``vins_estimator/src/factor/projectionTwoFrameOneCamFactor.cpp``
* ``vins_estimator/src/initial/initial_aligment.cpp``
* ``vins_estimator/src/factor/marginalization_factor.cpp``
