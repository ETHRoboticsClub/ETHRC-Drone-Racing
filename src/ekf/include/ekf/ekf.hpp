// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <deque>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace ekf
{

/**
 * Ekf
 *
 * A ROS-agnostic Extended Kalman Filter (EKF) core for fusing a high-rate
 * visual-inertial odometry (VIO) odometry stream with a lower-rate vision pose
 * stream. This class only defines the API and data layout; math is implemented
 * in the corresponding source file. Keep this class free of any ROS headers to
 * facilitate offline testing and reuse.
 *
 * Notes on representation and API choices:
 * - Nominal state layout mirrors the EKF definition (ICRA07 §III-A, §III-C)
 *   with an IMU core and a sliding window of camera clones. Orientation errors
 *   use the standard three-parameter small-angle approximation.
 * - This header exposes Eigen-based type aliases consistent with the EKF
 *   sizing while the external API continues to traffic in flattened row-major
 *   buffers for compatibility with existing code.
 * - All measurement update methods accept both the measurement vector and its
 *   covariance matrix. If the input covariance is unknown, pass reasonable
 *   defaults or configure them in Params.
 */
class Ekf
{
public:
  // ICRA07 §III-A/§III-C: Type system below encodes EKF nominal plus clone error-state structure.
  /** Dimension constants of the filter's nominal state. */
  static constexpr int ERR_IMU_DIM = 15;  // ICRA07 §III-A: 3 dtheta + b_g + v + b_a + p -> 15-dim IMU error core.
  static constexpr int ERR_CLONE_DIM = 6; // ICRA07 §III-C: Each camera clone contributes 3 dtheta + 3 dpos = 6 error states.
  static constexpr std::size_t kStateDim = static_cast<std::size_t>(ERR_IMU_DIM); // ICRA07 §III-A: Legacy constant reflects 15-dim IMU error subspace.
  static constexpr std::size_t kPosOffset = 0;
  static constexpr std::size_t kVelOffset = 3;
  static constexpr std::size_t kQuatOffset = 6;

  /** Lightweight vector/matrix aliases using row-major storage. */
  using Vector = std::vector<double>;                // size = n
  using Matrix = std::vector<double>;                // size = rows * cols, row-major

  /**
   * Configuration parameters that govern process/measurement modeling and
   * runtime behavior. These values have reasonable defaults; tune as needed.
   */
  struct Params
  {
    // Process noise (per-axis) for constant-velocity model [units^2/s].
    double q_pos = 1e-3;      // position random walk strength
    double q_vel = 1e-2;      // velocity random walk strength
    double q_ori = 1e-4;      // orientation random walk (small-angle) strength

    // Initial covariance diagonal entries for state components.
    double p0_pos = 1.0;      // initial position uncertainty [m^2]
    double p0_vel = 1.0;      // initial velocity uncertainty [(m/s)^2]
    double p0_ori = 1.0;      // initial orientation uncertainty [rad^2]

    // Simple update gating (squared Mahalanobis distance threshold).
    double gate_threshold_pose = 9.21;  // ~95% for 3 DoF
    double gate_threshold_odom = 12.59; // ~95% for 6 DoF

    // Behavior: treat VIO as measurement (true) or mainly as process driver.
    bool use_vio_as_measurement = true;

    // Limits
    double max_dt = 0.1;      // clamp prediction step to avoid blow-ups [s]
  };

  /** Snapshot of the current nominal state for external consumers. */
  struct ImuState
  {
    double stamp_sec{0.0}; // ICRA07 §III-B: Not part of the estimated vector; timestamp bookkeeping only.
    // Attitude stored as unit quaternion [x, y, z, w]; EKF attitude error is a 3-vector small angle. ICRA07 §III-A.
    std::array<double, 4> orientation{ {0.0, 0.0, 0.0, 1.0} }; // ICRA07 §III-A: Nominal quaternion ^I_G q in storage form.
    std::array<double, 3> gyro_bias{ {0.0, 0.0, 0.0} };        // ICRA07 §III-A: Gyroscope bias b_g estimated in the core state.
    std::array<double, 3> velocity{ {0.0, 0.0, 0.0} };         // ICRA07 §III-A: IMU velocity ^G v_I stored in nominal state.
    std::array<double, 3> accel_bias{ {0.0, 0.0, 0.0} };       // ICRA07 §III-A: Accelerometer bias b_a required for propagation consistency.
    std::array<double, 3> position{ {0.0, 0.0, 0.0} };         // ICRA07 §III-A: IMU position ^G p_I completes the 16-dof nominal IMU state.
  };

  struct CamClone
  {
    double stamp_sec{0.0}; // ICRA07 §III-C: Clone timestamp stored for book-keeping, not part of error state.
    // Attitude stored as unit quaternion [x, y, z, w]; error modeled as 3-vector small angle. ICRA07 §III-C.
    std::array<double, 4> orientation{ {0.0, 0.0, 0.0, 1.0} }; // ICRA07 §III-C: Camera clone quaternion ^C_k_G q nominal storage (7 dof).
    std::array<double, 3> position{ {0.0, 0.0, 0.0} };         // ICRA07 §III-C: Camera clone position ^G p_{C_k} contributes 3 of 6 error dims.
  };

  struct CamExtrinsics
  {
    // Attitude stored as unit quaternion [x, y, z, w]; fixed extrinsic rotation. ICRA07 §III-C.
    std::array<double, 4> orientation{ {0.0, 0.0, 0.0, 1.0} }; // ICRA07 §III-C: ^C R_I kept constant (not estimated).
    std::array<double, 3> position{ {0.0, 0.0, 0.0} };         // ICRA07 §III-C: ^I p_C is known translation offset.
  };

  struct EkfState : ImuState
  {
    EkfState() = default;
    EkfState &operator=(const ImuState &imu)
    {
      static_cast<ImuState &>(*this) = imu; // ICRA07 §III-A: Reuses IMU nominal overwrite while preserving EKF bookkeeping.
      clones.clear(); // ICRA07 §III-C: Resetting IMU nominal requires clearing existing camera clones.
      last_propagation_stamp_sec = imu.stamp_sec; // ICRA07 §III-B: Sync bookkeeping timestamp with new IMU nominal time.
      return *this;
    }

    std::deque<CamClone> clones{};       // ICRA07 §III-C: Sliding-window camera pose clones (size N).
    CamExtrinsics T_CI{};                // ICRA07 §III-C: Fixed IMU<->camera extrinsics held outside error state.
    Eigen::Vector3d g{0.0, 0.0, 0.0};    // ICRA07 §III-B: Gravity treated as known constant vector.
    double last_propagation_stamp_sec{0.0}; // ICRA07 §III-B: Book-keeping timestamp only, not estimated.
  };

  using State = EkfState; // ICRA07 §III-A: Legacy alias maintains API returning IMU core nominal state.

  using CoreErrorJacobian = Eigen::Matrix<double, ERR_IMU_DIM, ERR_IMU_DIM>; // ICRA07 §III-B: Continuous error dynamics F in R^{15x15}.
  using CoreNoiseJacobian = Eigen::Matrix<double, ERR_IMU_DIM, 12>;          // ICRA07 §III-B: Noise input G in R^{15x12} for gyro/acc noises and bias random walks.
  using CoreStateTransition = Eigen::Matrix<double, ERR_IMU_DIM, ERR_IMU_DIM>; // ICRA07 §III-B: Discrete transition Phi in R^{15x15}.
  using CoreProcessNoise = Eigen::Matrix<double, ERR_IMU_DIM, ERR_IMU_DIM>;     // ICRA07 §III-B: Discrete process noise Q_d in R^{15x15}.
  using ErrorCovariance = Eigen::MatrixXd;                                      // ICRA07 §III-A/§III-C: Covariance P sized (15+6N)x(15+6N).
  using CloneAugmentationJacobian = Eigen::Matrix<double, ERR_CLONE_DIM, ERR_IMU_DIM>; // ICRA07 §III-C: J_clone_from_imu in R^{6x15} for augmentation.

  int err_state_dim() const noexcept
  {
    return ERR_IMU_DIM + ERR_CLONE_DIM * static_cast<int>(current_num_clones());
  } // ICRA07 §III-A/§III-C: Total error-state = 15 + 6N for IMU plus N clones.

  /**
   * Construct an EKF with default parameters. The filter starts uninitialized
   * until initialize(...) is called.
   */
  Ekf();

  /** Construct an EKF with explicit parameters. */
  explicit Ekf(const Params &params);

  /** Virtual destructor to allow safe subclassing/testing if needed. */
  virtual ~Ekf() = default;

  /** Return true if initialize(...) has been called successfully. */
  bool isInitialized() const noexcept;

  /** Access current configuration parameters. */
  const Params &params() const noexcept;

  /** Replace configuration parameters at runtime. */
  void setParams(const Params &p);

  /**
   * Reset the filter to an uninitialized state. State and covariance are
   * cleared; isInitialized() returns false until initialize(...) is called.
   */
  void reset();

  /**
   * Initialize the filter state and covariance from an external pose/velocity
   * estimate, typically the first VIO or Vision sample.
   *
   * Arguments:
   *  - stamp_sec: timestamp (seconds) associated with this initialization
   *    sample. Subsequent predict(dt) expects dt relative to this time.
   *  - position: initial position [m] in world frame.
   *  - velocity: initial linear velocity [m/s] in world frame.
   *  - orientation_xyzw: unit quaternion [x, y, z, w].
   *  - P_diag: optional diagonal entries for initial covariance P (size = 10).
   *            If empty, defaults from Params are used.
   */
  void initialize(
      double stamp_sec,
      const std::array<double, 3> &position,
      const std::array<double, 3> &velocity,
      const std::array<double, 4> &orientation_xyzw,
      const Vector &P_diag = {});

  /**
   * Time update (prediction) using a constant-velocity motion model.
   *
   * This advances the nominal state forward by dt seconds, constructs the
   * continuous/discrete state transition (F/Phi) and process noise (Q), and
   * propagates the covariance P accordingly. Orientation is propagated either
   * as constant or via small-angle random walk depending on Params.
   *
   * Arguments:
   *  - dt: time step in seconds. Values > params().max_dt may be clamped
   *    internally for numerical stability.
   */
  void predict(double dt);

  /**
   * Measurement update with VIO odometry (position + velocity). If
   * params().use_vio_as_measurement is false, you may choose to skip calling
   * this in the node and rely on predict(...) instead.
   *
   * Arguments:
   *  - position: measured position [m].
   *  - velocity: measured velocity [m/s].
   *  - R_row_major: 6x6 measurement covariance (row-major flattened, size 36):
   *      [ px py pz vx vy vz ] ordering.
   *
   * Behavior:
   *  - Builds measurement model H_vio and residual r, performs gating, computes
   *    innovation covariance S and Kalman gain K, and updates x, P.
   */
  void updateVioOdom(
      const std::array<double, 3> &position,
      const std::array<double, 3> &velocity,
      const Matrix &R_row_major);

  /**
   * Measurement update with Vision pose (position only).
   *
   * Arguments:
   *  - position: measured position [m].
   *  - R_row_major: 3x3 measurement covariance (row-major flattened, size 9).
   *
   * Behavior:
   *  - Builds measurement model H_vis and residual r, performs gating, computes
   *    innovation covariance S and Kalman gain K, and updates x, P.
   */
  void updateVisionPose(
      const std::array<double, 3> &position,
      const Matrix &R_row_major);

  /** Return a snapshot of the current nominal state. */
  State state() const noexcept;

  /**
   * Return the current covariance P in row-major flattened form
   * (size = kStateDim * kStateDim). External users can inspect diagonals for
   * uncertainty metrics or export to logs.
   */
  Matrix covariance() const;

  /** Update the internal timestamp (in seconds) associated with the state. */
  void setStamp(double stamp_sec) noexcept;

  /** Read the internal timestamp (in seconds) associated with the state. */
  double stamp() const noexcept;

private:
  // --- Internal helpers (declared here; implemented in .cpp) ---

  /** Normalize the quaternion portion of the nominal state. */
  void normalizeQuaternion();

  /**
   * Compute or update the discrete-time state transition matrix Phi (phi) for a
   * constant-velocity model over dt seconds. Stored internally as row-major
   * (kStateDim x kStateDim).
   */
  void computeStateTransitionMatrix(double dt);

  /**
   * Compute or update the discrete process noise matrix Q for the chosen
   * motion model over dt seconds. Stored internally as row-major
   * (kStateDim x kStateDim).
   */
  void computeProcessNoise(double dt);

  /** Build the VIO measurement matrix H_vio (6 x kStateDim). */
  void buildVioMeasurementMatrix();

  /** Build the Vision measurement matrix H_vis (3 x kStateDim). */
  void buildVisionMeasurementMatrix();

  /**
   * Compute residual r = z - h(x) for VIO odometry measurement. The argument
   * r_out is resized to 6 on return.
   */
  void computeVioResidual(const std::array<double, 3> &z_pos,
                          const std::array<double, 3> &z_vel,
                          Vector &r_out) const;

  /**
   * Compute residual r = z - h(x) for Vision pose measurement. The argument
   * r_out is resized to 3 on return.
   */
  void computeVisionResidual(const std::array<double, 3> &z_pos,
                             Vector &r_out) const;

  /**
   * Perform a simple chi-square gating test on a residual given an innovation
   * covariance S (flattened row-major). Returns true if the residual passes
   * the gate, false otherwise. The dimension m must match the residual size.
   */
  bool passesGate(const Vector &residual,
                  const Matrix &S_row_major,
                  std::size_t m,
                  double threshold) const;

  /**
   * Compute innovation covariance S = H P H^T + R for a measurement model H
   * (m x n) given the current covariance P (n x n) and measurement noise R
   * (m x m). All matrices are row-major flattened.
   */
  void computeInnovationCovariance(const Matrix &H_row_major,
                                   std::size_t m,
                                   std::size_t n,
                                   const Matrix &R_row_major,
                                   Matrix &S_row_major) const;

  /**
   * Compute Kalman gain K = P H^T S^{-1} for a measurement model H (m x n).
   * The result K has shape (n x m). All matrices are row-major flattened.
   */
  void computeKalmanGain(const Matrix &H_row_major,
                         std::size_t m,
                         std::size_t n,
                         const Matrix &S_row_major,
                         Matrix &K_row_major) const;

  /**
   * Apply the standard EKF state and covariance update using a Kalman gain K
   * (n x m), measurement model H (m x n), and residual r (m).
   */
  void applyUpdate(const Matrix &K_row_major,
                   const Matrix &H_row_major,
                   const Vector &residual,
                   std::size_t m,
                   std::size_t n);

  /** Inject a small-angle orientation error into the nominal quaternion. */
  void injectOrientationError(const std::array<double, 3> &dtheta);

  // --- Internal storage ---
  Params params_{};
  EkfState state_{};
  CoreErrorJacobian F_core_{};                   // ICRA07 §III-B: Continuous-time IMU error Jacobian F.
  CoreNoiseJacobian G_core_{};                   // ICRA07 §III-B: IMU noise Jacobian G for gyro/acc noises and bias RWs.
  CoreStateTransition Phi_core_{};               // ICRA07 §III-B: Discrete IMU transition Phi from integrating F.
  CoreProcessNoise Qd_core_{};                   // ICRA07 §III-B: Discrete IMU process noise covariance Q_d.
  ErrorCovariance P_ekf_{};                      // ICRA07 §III-A/§III-C: Full covariance partitioned into IMU/clones blocks.
  CloneAugmentationJacobian J_clone_from_imu_{}; // ICRA07 §III-C: Augmentation Jacobian mapping IMU error into new clone.
  bool initialized_{false};

  // Row-major matrices sized to kStateDim x kStateDim unless noted.
  Matrix P_;           // Covariance of the nominal state
  Matrix Phi_;         // Discrete state transition matrix
  Matrix Q_;           // Discrete process noise matrix

  // Measurement model matrices (row-major):
  // H_vio: 6 x kStateDim, H_vis: 3 x kStateDim
  Matrix H_vio_;
  Matrix H_vis_;
  // ICRA07 §IV: After left-nullspace projection, effective measurement Jacobians map {IMU (+) clones} only; rows depend on feature track length.

  std::size_t current_num_clones() const noexcept
  {
    return state_.clones.size();
  } // ICRA07 §III-C: Clone count N from sliding-window maintained clones deque.
};

} // namespace ekf
