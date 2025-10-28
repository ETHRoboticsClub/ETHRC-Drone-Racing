// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

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
 * State vector layout (error-state for orientation):
 *   x = [ px, py, pz, vx, vy, vz, qx, qy, qz, qw ]^T
 * where position p and velocity v are in meters (world frame), and q is a
 * unit quaternion (world->body or body->world per convention you adopt; the
 * class does not fix the convention but requires consistency across inputs).
 *
 * Notes on representation and API choices:
 * - This header intentionally avoids external dependencies (e.g., Eigen) to
 *   keep the public interface lightweight. Matrices passed to or returned from
 *   the API use flattened row-major vectors and explicit dimension parameters
 *   where relevant. Internals may use a linear algebra library in the .cpp.
 * - All measurement update methods accept both the measurement vector and its
 *   covariance matrix. If the input covariance is unknown, pass reasonable
 *   defaults or configure them in Params.
 */
class Ekf
{
public:
  /** Dimension constants of the filter's nominal state. */
  static constexpr std::size_t kStateDim = 10;  // 3 pos + 3 vel + 4 quat
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
  struct State
  {
    std::array<double, 3> position{ {0.0, 0.0, 0.0} };
    std::array<double, 3> velocity{ {0.0, 0.0, 0.0} };
    // Quaternion stored as [x, y, z, w], normalized.
    std::array<double, 4> orientation{ {0.0, 0.0, 0.0, 1.0} };
    // Timestamp (seconds) of the nominal state, typically the last predict time.
    double stamp_sec{0.0};
  };

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
   * continuous/discrete state transition (F/Φ) and process noise (Q), and
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
   * Compute or update the discrete-time state transition matrix Φ (phi) for a
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
  State state_{};
  bool initialized_{false};

  // Row-major matrices sized to kStateDim x kStateDim unless noted.
  Matrix P_;           // Covariance of the nominal state
  Matrix Phi_;         // Discrete state transition matrix
  Matrix Q_;           // Discrete process noise matrix

  // Measurement model matrices (row-major):
  // H_vio: 6 x kStateDim, H_vis: 3 x kStateDim
  Matrix H_vio_;
  Matrix H_vis_;
};

} // namespace ekf
