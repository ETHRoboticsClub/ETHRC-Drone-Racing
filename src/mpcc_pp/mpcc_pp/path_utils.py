#!/usr/bin/env python
"""
Arc-length parameterized path utilities for MPCC.

Implements Section III-C and Algorithm 1 of:
  Romero et al., "Model Predictive Contouring Control for Time-Optimal
  Quadrotor Flight", IEEE T-RO 2022.

Given a sequence of 3D waypoints:
  1. Computes arc-length parameterization (linear between samples,
     as described in Algorithm 1 for sampled-point paths).
  2. Fits cubic splines segment-by-segment (eq. 8).
  3. Evaluates position p^d(θ) and unit tangent t(θ) = dp^d/dθ.
  4. Computes the dynamic contouring weight qc(p^d(θ)) (Section IV-C).
  5. Projects a given position onto the path to update θ (Algorithm 1).

GatedRacingPath extends RacingPath with the Track Constraint from:
  Krinner et al., "mpcc_pp: Model Predictive Contouring Control for
  Time-Optimal Flight with Safety Constraints", RSS 2024.

The Track Constraint (Section IV-A, eq. 7) defines a prismatic tunnel
centred on the reference path:
  p_0(θ) = p^d(θ) - W(θ)·n(θ) - H(θ)·b(θ)   [bottom-left corner]

  (p - p_0)·n  ≥ 0          left wall
  2W - (p-p_0)·n  ≥ 0       right wall   } total width  = 2W(θ)
  (p - p_0)·b  ≥ 0          bottom wall
  2H - (p-p_0)·b  ≥ 0       top wall     } total height = 2H(θ)

Simplifying with e_n = (p - p^d)·n  and  e_b = (p - p^d)·b:
  |e_n| ≤ W(θ)   and   |e_b| ≤ H(θ)

W(θ) and H(θ) shrink near each gate (enforcing safe passage) and
widen between gates (freedom for time-optimal flight).

Local frame (NED, for vertical racing gates):
  n(θ) = gate-width axis, derived from tangent or gate yaw near vertical
  b(θ) = normalize(t(θ) × n(θ)) — local gate-height axis
"""

import numpy as np
from scipy.interpolate import CubicHermiteSpline, CubicSpline


# Tunnel scale factor:
#   tau = 1.0  -> nominal tunnel dimensions
#   tau = 0.0  -> zero-size tunnel
# Values outside [0, 1] are clipped when GatedRacingPath is built.
tau = 0.95

# Number of columns in the geometry parameter matrix returned by
# get_tunnel_params(). Must match DroneOptimizerMpccPp.GEOMETRY_NP in
# drone_opt.py (p_ref[3] + t_ref[3] + qc[1] + n_ref[3] + b_ref[3] +
# W[1] + H[1] + theta_ref[1] + c_ref[3] = 19).
GEOMETRY_NP = 19


class RacingPath:
    """
    Arc-length parameterized 3D path built from waypoints.

    The path parameter θ ∈ [0, total_length] represents the arc length
    travelled along the path. Internally, cubic splines are fitted to
    the waypoints with θ as the independent variable.

    Args:
        waypoints:      (N, 3) array of ordered 3D positions.
        gate_positions: (M, 3) positions of gates/waypoints where the
                        contouring weight is increased. Defaults to
                        waypoints themselves.
        sigma:          Gaussian half-width for the dynamic qc weight [m].
        lookup_resolution_m:
                        Arc-length spacing [m] for the p_ref/t_ref lookup table.
    """

    def __init__(self, waypoints: np.ndarray,
                 gate_positions: np.ndarray = None,
                 sigma: float = 0.5,
                 lookup_resolution_m: float = 0.05):
        assert waypoints.ndim == 2 and waypoints.shape[1] == 3
        assert waypoints.shape[0] >= 2

        self._sigma = sigma
        self._lookup_resolution_m = self._validate_lookup_resolution(
            lookup_resolution_m)
        self._gates = (gate_positions if gate_positions is not None
                       else waypoints.copy())

        # --- Arc-length parameterisation (Algorithm 1: assume linearity) ---
        diffs = np.diff(waypoints, axis=0)              # (N-1, 3)
        seg_lengths = np.linalg.norm(diffs, axis=1)     # (N-1,)
        arc = np.concatenate([[0.0], np.cumsum(seg_lengths)])  # (N,)

        self.total_length: float = float(arc[-1])

        # Fit cubic splines for each axis (not-a-knot boundary condition)
        self._cs_x = CubicSpline(arc, waypoints[:, 0])
        self._cs_y = CubicSpline(arc, waypoints[:, 1])
        self._cs_z = CubicSpline(arc, waypoints[:, 2])

        self._cs_dx = self._cs_x.derivative()
        self._cs_dy = self._cs_y.derivative()
        self._cs_dz = self._cs_z.derivative()

        # Second derivatives w.r.t. arc length — curvature vector p^{d''}(θ),
        # used for the 2nd-order Taylor correction of the path reference.
        self._cs_d2x = self._cs_x.derivative(2)
        self._cs_d2y = self._cs_y.derivative(2)
        self._cs_d2z = self._cs_z.derivative(2)
        self._build_lookup_table()

    # ------------------------------------------------------------------
    # Core path evaluation
    # ------------------------------------------------------------------

    def position(self, theta: float) -> np.ndarray:
        """Evaluate p^d(θ) → (3,)."""
        theta = self._clip(theta)
        return np.array([float(self._cs_x(theta)),
                         float(self._cs_y(theta)),
                         float(self._cs_z(theta))], dtype=np.float64)

    def tangent(self, theta: float) -> np.ndarray:
        """Unit tangent t(θ) = dp^d/dθ / ‖dp^d/dθ‖ → (3,).

        Arc-length parameterisation guarantees ‖t‖ = 1 (eq. 9); in
        practice numerical splines may deviate slightly, so we normalise.
        """
        theta = self._clip(theta)
        raw = np.array([float(self._cs_dx(theta)),
                        float(self._cs_dy(theta)),
                        float(self._cs_dz(theta))], dtype=np.float64)
        norm = np.linalg.norm(raw)
        if norm < 1e-8:
            return np.array([1.0, 0.0, 0.0])
        return raw / norm

    # ------------------------------------------------------------------
    # Dynamic contouring weight (Section IV-C)
    # ------------------------------------------------------------------

    def contouring_weight(self, pd: np.ndarray,
                          qc_nom: float = 0.5,
                          qc_gate: float = 10.0) -> float:
        """
        Dynamic qc(p^d(θ)) — sum of Gaussians at gate positions.

        qc → qc_gate near a gate (tracking is critical),
        qc → qc_nom on straight segments (progress dominates).
        """
        return float(self.contouring_weights(
            np.asarray(pd, dtype=np.float64).reshape(1, 3),
            qc_nom,
            qc_gate,
        )[0])

    def contouring_weights(self, pds: np.ndarray,
                           qc_nom: float = 0.5,
                           qc_gate: float = 10.0) -> np.ndarray:
        """Vectorized dynamic contouring weights for sampled path points."""
        pds = np.asarray(pds, dtype=np.float64).reshape(-1, 3)
        sigma2 = self._sigma ** 2
        diffs = pds[:, np.newaxis, :] - self._gates[np.newaxis, :, :]
        sq_dists = np.sum(diffs * diffs, axis=2)
        weights = np.sum(np.exp(-0.5 * sq_dists / sigma2), axis=1)
        weights = np.clip(weights, 0.0, 1.0)
        return qc_nom + (qc_gate - qc_nom) * weights

    # ------------------------------------------------------------------

    def get_tunnel_params(self, theta: float, n_stages: int, dt: float,
                          vtheta_guess: float = 3.0,
                          qc_nom: float = 0.5,
                          qc_gate: float = 10.0,
                          W_default: float = 1000.0,
                          H_default: float = 1000.0,
                          theta_values: np.ndarray = None) -> np.ndarray:
        """
        19-column geometry parameters for the Track-Constraint OCP.

        This default implementation uses very large W and H (1000 m) so the
        track constraint is inactive.  GatedRacingPath overrides this method
        with real gate geometry.  The controller appends runtime cost weights
        before passing the full parameter matrix to ACADOS.

        Returns:
            params: (n_stages, 19) — columns:
              [0:3]   p_ref  nominal reference position p^d(theta_ref)
              [3:6]   t_ref  unit tangent               p^{d'}(theta_ref)
              [6]     qc     contouring weight
              [7:10]  n_ref  horizontal tunnel normal  (right direction)
              [10:13] b_ref  vertical   tunnel normal  (up direction)
              [13]    W      tunnel half-width
              [14]    H      tunnel half-height
              [15]    theta_ref nominal progress used for p_ref update
              [16:19] c_ref  curvature vector           p^{d''}(theta_ref),
                             used for the 2nd-order Taylor correction of
                             p_ref (curvature-aware linearisation, cf.
                             Liniger's getSecondDerivative/getRefPoint)
        """
        params = np.zeros((n_stages, GEOMETRY_NP))
        thetas = self._resolve_horizon_thetas(
            theta, n_stages, dt, vtheta_guess, theta_values)
        pds, tds, cds = self._lookup_path_many(thetas)
        qcs = self.contouring_weights(pds, qc_nom, qc_gate)

        for k, td in enumerate(tds):
            horiz = np.array([-td[1], td[0], 0.0])
            norm = np.linalg.norm(horiz)
            n = horiz / norm if norm > 1e-8 else np.array([1.0, 0.0, 0.0])
            b = np.array([0.0, 0.0, -1.0])

            params[k,  0:3]  = pds[k]
            params[k,  3:6]  = td
            params[k,  6]    = qcs[k]
            params[k,  7:10] = n
            params[k, 10:13] = b
            params[k, 13]    = W_default
            params[k, 14]    = H_default
            params[k, 15]    = thetas[k]
            params[k, 16:19] = cds[k]
        return params

    # ------------------------------------------------------------------
    # Closest-point projection (Algorithm 1: update θ from position)
    # ------------------------------------------------------------------

    def find_closest_theta(self, position: np.ndarray,
                           theta_current: float,
                           search_ahead: float = 3.0,
                           n_points: int = 200) -> float:
        """
        Project `position` onto the path to find the closest θ.

        Searches in [max(0, θ_current − 0.5), θ_current + search_ahead]
        to prevent backward jumps while handling small backward oscillations.
        """
        th_lo = max(0.0, theta_current - 0.5)
        th_hi = min(self.total_length, theta_current + search_ahead)
        candidates = np.linspace(th_lo, th_hi, n_points)
        pts = np.column_stack([
            self._cs_x(candidates),
            self._cs_y(candidates),
            self._cs_z(candidates),
        ])
        dists = np.linalg.norm(pts - position[np.newaxis, :], axis=1)
        return float(candidates[np.argmin(dists)])

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_lookup_resolution(value: float) -> float:
        resolution = float(value)
        if not np.isfinite(resolution) or resolution <= 0.0:
            raise ValueError('lookup_resolution_m must be a finite positive value')
        return resolution

    def _build_lookup_table(self) -> None:
        n_samples = max(
            2,
            int(np.ceil(self.total_length / self._lookup_resolution_m)) + 1,
        )
        theta = np.linspace(0.0, self.total_length, n_samples)
        positions = np.column_stack([
            self._cs_x(theta),
            self._cs_y(theta),
            self._cs_z(theta),
        ]).astype(np.float64)
        raw_tangents = np.column_stack([
            self._cs_dx(theta),
            self._cs_dy(theta),
            self._cs_dz(theta),
        ]).astype(np.float64)
        norms = np.linalg.norm(raw_tangents, axis=1)
        tangents = np.zeros_like(raw_tangents)
        valid = norms > 1e-8
        tangents[valid] = raw_tangents[valid] / norms[valid, np.newaxis]
        tangents[~valid] = np.array([1.0, 0.0, 0.0], dtype=np.float64)

        # Curvature vector p^{d''}(θ) = d²p/dθ² (raw, unnormalised — pairs
        # with the raw first derivative in the 2nd-order Taylor expansion
        # of the path reference; analogous to Liniger's getSecondDerivative).
        curvatures = np.column_stack([
            self._cs_d2x(theta),
            self._cs_d2y(theta),
            self._cs_d2z(theta),
        ]).astype(np.float64)

        self._lut_theta = theta
        self._lut_position = positions
        self._lut_tangent = tangents
        self._lut_curvature = curvatures

    def _lookup_path_many(self, theta_values: np.ndarray) -> tuple:
        """Vectorized lookup of position, unit tangent, and curvature vector."""
        thetas = np.clip(
            np.asarray(theta_values, dtype=np.float64).reshape(-1),
            0.0,
            self.total_length,
        )
        positions = np.column_stack([
            np.interp(thetas, self._lut_theta, self._lut_position[:, axis])
            for axis in range(3)
        ])
        tangents = np.column_stack([
            np.interp(thetas, self._lut_theta, self._lut_tangent[:, axis])
            for axis in range(3)
        ])
        curvatures = np.column_stack([
            np.interp(thetas, self._lut_theta, self._lut_curvature[:, axis])
            for axis in range(3)
        ])
        norms = np.linalg.norm(tangents, axis=1)
        valid = norms > 1e-8
        tangents[valid] = tangents[valid] / norms[valid, np.newaxis]
        tangents[~valid] = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        return positions, tangents, curvatures

    def _lookup_position_tangent_many(self, theta_values: np.ndarray) -> tuple:
        positions, tangents, _ = self._lookup_path_many(theta_values)
        return positions, tangents

    def _lookup_position_tangent(self, theta: float) -> tuple:
        positions, tangents = self._lookup_position_tangent_many(
            np.array([theta], dtype=np.float64))
        return positions[0], tangents[0]

    def _horizon_thetas(self, theta: float, n_stages: int, dt: float,
                        vtheta_guess: float) -> np.ndarray:
        theta0 = self._clip(theta)
        step = float(vtheta_guess) * float(dt)
        return np.minimum(
            theta0 + step * np.arange(n_stages, dtype=np.float64),
            self.total_length,
        )

    def _resolve_horizon_thetas(self, theta: float, n_stages: int, dt: float,
                                vtheta_guess: float,
                                theta_values: np.ndarray = None) -> np.ndarray:
        if theta_values is None:
            return self._horizon_thetas(theta, n_stages, dt, vtheta_guess)

        thetas = np.asarray(theta_values, dtype=np.float64).reshape(-1)
        if thetas.size == 0 or not np.isfinite(thetas).all():
            return self._horizon_thetas(theta, n_stages, dt, vtheta_guess)
        if thetas.size < n_stages:
            thetas = np.pad(thetas, (0, n_stages - thetas.size),
                            mode='edge')
        return np.clip(thetas[:n_stages], 0.0, self.total_length)

    def _clip(self, theta: float) -> float:
        return float(np.clip(theta, 0.0, self.total_length))


# ======================================================================
# GatedRacingPath — Track Constraint extension (mpcc_pp paper, RSS 2024)
# ======================================================================

class GatedRacingPath(RacingPath):
    """
    Arc-length parameterized path with gate-aware tunnel constraints.

    Extends RacingPath with:
      - Gate positions, yaw angles, and inner aperture dimensions.
      - Arc-length-varying tunnel half-extents W(θ) and H(θ).
      - Local tunnel frame n(θ), b(θ) for halfspace constraints.
      - get_tunnel_params() returning the 19-column geometry matrix that the
        controller extends with runtime cost weights.

    Track Constraint (mpcc_pp paper, Section IV-A, eq. 7):
    --------------------------------------------------------
    The tunnel cross-section at arc-length θ is a rectangle of size
    2W(θ) × 2H(θ) centred at the reference path p^d(θ).

    Bottom-left corner:
        p_0 = p^d − W·n − H·b

    Halfspace inequalities (all must be ≥ 0 for safe passage):
        h1 = (p − p_0)·n         ≥ 0
        h2 = 2W − (p − p_0)·n   ≥ 0
        h3 = (p − p_0)·b         ≥ 0
        h4 = 2H − (p − p_0)·b   ≥ 0

    Equivalently (after simplification):
        h1 = W + e_n             where e_n = (p − p^d)·n
        h2 = W − e_n
        h3 = H + e_b             where e_b = (p − p^d)·b
        h4 = H − e_b

    All four are ≥ 0 iff  |e_n| ≤ W  and  |e_b| ≤ H.

    W(θ) / H(θ) parameterization:
    --------------------------------
    Wide between gates (W_nominal) and narrow at each gate (W_gate_i):

        W(θ) = W_n + Σ_i (W_gate_i − W_n) · exp(−½(θ − θ_gate_i)² / σ²)

    Since W_gate_i < W_n, each term is negative → W decreases near gates.
    The result is clamped to [min_gate_half_width, W_n].
    The final half-extents are scaled by tau ∈ [0, 1].

    Local frame (NED, vertical racing gates):
    ------------------------------------------
        n(θ) is the gate-width axis, derived from the local path tangent where
        possible and from gate yaw near vertical maneuvers.
        b(θ) = normalize(t(θ) × n(θ)) is the local height axis.

    Args:
        gates:       List of gate dicts: {x, y, z, yaw_deg, width, height, name}.
        W_nominal:   Tunnel half-width  between gates [m].
        H_nominal:   Tunnel half-height between gates [m].
        sigma_gate:  Gaussian σ for W/H transitions [m].
        sigma_qc:    Gaussian σ for contouring-weight transitions [m].
        tau_scale:   Global tunnel scale factor in [0, 1].
        lookup_resolution_m:
                     Arc-length spacing [m] for the p_ref/t_ref lookup table.
    """

    def __init__(self, gates: list,
                 W_nominal: float = 3.0,
                 H_nominal: float = 3.0,
                 sigma_gate: float = 2.0,
                 sigma_qc: float = 0.5,
                 tau_scale: float = tau,
                 start_position: np.ndarray = None,
                 lookup_resolution_m: float = 0.05):

        assert len(gates) >= 2, "At least 2 gates required."

        self.idxprint = 0

        # Gate centre positions (NED)
        positions = np.array(
            [[g['x'], g['y'], g['z']] for g in gates], dtype=np.float64)
        gate_tangents = np.array(
            [self._yaw_to_tangent(g.get('yaw_deg', 0.0)) for g in gates],
            dtype=np.float64)
        self._gate_normals = np.array(
            [self._yaw_to_width_axis(g.get('yaw_deg', 0.0)) for g in gates],
            dtype=np.float64)

        # Inner half-extents (half of the aperture dimension)
        self._gate_half_widths  = np.array(
            [g.get('width',  2.0) / 2.0 for g in gates], dtype=np.float64)
        self._gate_half_heights = np.array(
            [g.get('height', 2.0) / 2.0 for g in gates], dtype=np.float64)

        self._W_n        = float(W_nominal)
        self._H_n        = float(H_nominal)
        self._sigma_gate = float(sigma_gate)
        self._tau        = float(np.clip(tau_scale, 0.0, 1.0))
        self._min_gate_half_w = float(np.min(self._gate_half_widths))
        self._min_gate_half_h = float(np.min(self._gate_half_heights))

        # Optional lead-in: prepend drone start position before gate 1.
        # The circuit still closes at gate 1 (not the start position).
        if start_position is not None:
            sp = np.asarray(start_position, dtype=np.float64).reshape(1, 3)
            positions_with_start = np.vstack([sp, positions])
            start_tangent = self._direction_or_fallback(
                positions[0] - sp[0], gate_tangents[0])
            tangents_closed = np.vstack([
                start_tangent,
                gate_tangents,
                gate_tangents[0],
            ])
        else:
            positions_with_start = positions
            tangents_closed = np.vstack([gate_tangents, gate_tangents[0]])

        # Closed-loop path: append first gate to close the circuit
        positions_closed = np.vstack([positions_with_start, positions[0]])

        # Compute arc-length knots BEFORE calling super().__init__
        # so they align exactly with the spline knots the parent will build.
        diffs = np.diff(positions_closed, axis=0)
        seg_lengths = np.linalg.norm(diffs, axis=1)
        arc = np.concatenate([[0.0], np.cumsum(seg_lengths)])

        if start_position is not None:
            # arc[0] = θ at start_position; arc[1..n_gates] = θ at each gate
            self._gate_thetas = arc[1:-1].copy()   # shape (n_gates,)
        else:
            # arc[0..n_gates-1] = θ at each gate
            self._gate_thetas = arc[:-1].copy()    # shape (n_gates,)

        # Initialise parent with closed positions; gate centres used for qc.
        super().__init__(
            positions_closed,
            gate_positions=positions,
            sigma=sigma_qc,
            lookup_resolution_m=lookup_resolution_m,
        )
        self._fit_yaw_constrained_splines(arc, positions_closed, tangents_closed)
        self._build_lookup_table()

    # ------------------------------------------------------------------
    # Periodic circuit geometry
    # ------------------------------------------------------------------

    def _map_circuit_thetas(self, theta_values: np.ndarray) -> np.ndarray:
        """Map progress beyond the first lap onto the closed gate circuit."""
        thetas = np.asarray(theta_values, dtype=np.float64).reshape(-1)
        mapped = np.maximum(thetas, 0.0)
        beyond_first_lap = mapped > self.total_length
        mapped[beyond_first_lap] = (
            self.circuit_start_theta
            + np.mod(
                mapped[beyond_first_lap] - self.total_length,
                self.circuit_length,
            )
        )
        return mapped

    def _map_circuit_theta(self, theta: float) -> float:
        """Scalar form of :meth:`_map_circuit_thetas`."""
        return float(self._map_circuit_thetas(
            np.array([theta], dtype=np.float64))[0])

    def position(self, theta: float) -> np.ndarray:
        """Evaluate position with periodic geometry after the first lap."""
        return super().position(self._map_circuit_theta(theta))

    def tangent(self, theta: float) -> np.ndarray:
        """Evaluate tangent with periodic geometry after the first lap."""
        return super().tangent(self._map_circuit_theta(theta))

    def _lookup_path_many(self, theta_values: np.ndarray) -> tuple:
        """Look up periodic circuit geometry for unwrapped progress values."""
        return super()._lookup_path_many(
            self._map_circuit_thetas(theta_values))

    def _horizon_thetas(self, theta: float, n_stages: int, dt: float,
                        vtheta_guess: float) -> np.ndarray:
        """Return an unwrapped gated-path horizon that may cross the seam."""
        theta0 = max(float(theta), 0.0)
        step = float(vtheta_guess) * float(dt)
        return theta0 + step * np.arange(n_stages, dtype=np.float64)

    def _resolve_horizon_thetas(self, theta: float, n_stages: int, dt: float,
                                vtheta_guess: float,
                                theta_values: np.ndarray = None) -> np.ndarray:
        """Resolve unwrapped, nondecreasing progress for gated horizons."""
        if theta_values is None:
            return self._horizon_thetas(
                theta, n_stages, dt, vtheta_guess)

        thetas = np.asarray(theta_values, dtype=np.float64).reshape(-1)
        if thetas.size == 0 or not np.isfinite(thetas).all():
            return self._horizon_thetas(
                theta, n_stages, dt, vtheta_guess)
        if thetas.size < n_stages:
            thetas = np.pad(
                thetas, (0, n_stages - thetas.size), mode='edge')
        thetas = np.maximum(thetas[:n_stages], 0.0)
        return np.maximum.accumulate(thetas)

    # ------------------------------------------------------------------
    # Tunnel geometry
    # ------------------------------------------------------------------

    @staticmethod
    def _yaw_to_tangent(yaw_deg: float) -> np.ndarray:
        """Gate flight direction in NED from yaw_deg: 0=N, 90=E."""
        yaw = np.deg2rad(float(yaw_deg))
        return np.array([np.cos(yaw), np.sin(yaw), 0.0], dtype=np.float64)

    @staticmethod
    def _yaw_to_width_axis(yaw_deg: float) -> np.ndarray:
        """Gate width axis: horizontal right direction for the given yaw."""
        yaw = np.deg2rad(float(yaw_deg))
        return np.array([-np.sin(yaw), np.cos(yaw), 0.0], dtype=np.float64)

    @staticmethod
    def _direction_or_fallback(vector: np.ndarray,
                               fallback: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(vector)
        if norm < 1e-8:
            return np.asarray(fallback, dtype=np.float64)
        return np.asarray(vector, dtype=np.float64) / norm

    def _fit_yaw_constrained_splines(self, arc: np.ndarray,
                                     positions: np.ndarray,
                                     tangents: np.ndarray) -> None:
        """
        Replace the center-only cubic spline with a Hermite spline.

        Each gate remains an interpolation knot, and yaw_deg constrains
        dp/dtheta at that gate so the reference passes through the aperture
        along the configured gate normal instead of letting neighboring centers
        pick the crossing direction.
        """
        self.total_length = float(arc[-1])
        self._cs_x = CubicHermiteSpline(arc, positions[:, 0], tangents[:, 0])
        self._cs_y = CubicHermiteSpline(arc, positions[:, 1], tangents[:, 1])
        self._cs_z = CubicHermiteSpline(arc, positions[:, 2], tangents[:, 2])

        self._cs_dx = self._cs_x.derivative()
        self._cs_dy = self._cs_y.derivative()
        self._cs_dz = self._cs_z.derivative()

        self._cs_d2x = self._cs_x.derivative(2)
        self._cs_d2y = self._cs_y.derivative(2)
        self._cs_d2z = self._cs_z.derivative(2)

    def tunnel_width(self, theta: float) -> float:
        """
        W(θ): tunnel half-width at arc-length θ.

        Gaussian transitions between W_nominal and W_gate_i.
        Wrap-around is handled by taking the nearest periodic equivalent.
        """
        theta = self._map_circuit_theta(theta)
        L = self.total_length
        W = self._W_n
        # for th_g, W_g in zip(self._gate_thetas, self._gate_half_widths):
        #     d = theta - th_g
        #     d -= L * round(d / L)   # shortest periodic distance
        #     W += (W_g - self._W_n) * np.exp(-0.5 * d * d / (self._sigma_gate ** 2))
        # use numpz vectorization for speedup:
        d = theta - self._gate_thetas  # (n_gates,)
        d -= L * np.round(d / L)       # shortest periodic distance (n_gates,)
        W += np.sum((self._gate_half_widths - self._W_n) *
                    np.exp(-0.5 * d * d / (self._sigma_gate ** 2)))

        return self._tau * float(np.clip(W, self._min_gate_half_w, self._W_n))

    def tunnel_height(self, theta: float) -> float:
        """
        H(θ): tunnel half-height at arc-length θ.
        """
        theta = self._map_circuit_theta(theta)
        L = self.total_length
        H = self._H_n
        # for th_g, H_g in zip(self._gate_thetas, self._gate_half_heights):
        #     d = theta - th_g
        #     d -= L * round(d / L)
        #     H += (H_g - self._H_n) * np.exp(-0.5 * d * d / (self._sigma_gate ** 2))
        # use numpz vectorization for speedup:
        d = theta - self._gate_thetas  # (n_gates,)
        d -= L * np.round(d / L)       # shortest periodic distance (n_gates,)
        H += np.sum((self._gate_half_heights - self._H_n) *
                    np.exp(-0.5 * d * d / (self._sigma_gate ** 2)))

        return self._tau * float(np.clip(H, self._min_gate_half_h, self._H_n))

    def tunnel_widths(self, theta_values: np.ndarray) -> np.ndarray:
        thetas = self._map_circuit_thetas(theta_values)
        d = thetas[:, np.newaxis] - self._gate_thetas[np.newaxis, :]
        d -= self.total_length * np.round(d / self.total_length)
        W = self._W_n + np.sum(
            (self._gate_half_widths - self._W_n)[np.newaxis, :]
            * np.exp(-0.5 * d * d / (self._sigma_gate ** 2)),
            axis=1,
        )
        return self._tau * np.clip(W, self._min_gate_half_w, self._W_n)

    def tunnel_heights(self, theta_values: np.ndarray) -> np.ndarray:
        thetas = self._map_circuit_thetas(theta_values)
        d = thetas[:, np.newaxis] - self._gate_thetas[np.newaxis, :]
        d -= self.total_length * np.round(d / self.total_length)
        H = self._H_n + np.sum(
            (self._gate_half_heights - self._H_n)[np.newaxis, :]
            * np.exp(-0.5 * d * d / (self._sigma_gate ** 2)),
            axis=1,
        )
        return self._tau * np.clip(H, self._min_gate_half_h, self._H_n)

    def normal_binormal(self, theta: float, t: np.ndarray = None) -> tuple:
        """
        Local tunnel frame at arc-length θ.

        Returns:
            n  (3,) — horizontal right direction (gate width axis)
            b  (3,) — NED up direction           (gate height axis)

        For mostly horizontal path sections, the width axis is the horizontal
        right direction of the tangent.  For near-vertical sections, that
        direction is ill-defined, so we use the nearest gate's yaw-defined
        width axis.  The final basis is orthogonalized against the tangent.
        """
        if t is None:
            t = self.tangent(theta)
        
        horiz = np.array([-t[1], t[0], 0.0])
        norm = np.linalg.norm(horiz)
        if norm < 1e-8:
            mapped_theta = self._map_circuit_theta(theta)
            idx = int(np.argmin(np.abs(self._gate_thetas - mapped_theta)))
            n_raw = self._gate_normals[idx]
        else:
            n_raw = horiz / norm

        n = n_raw - np.dot(n_raw, t) * t
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-8:
            candidates = np.eye(3)
            n = candidates[int(np.argmin(np.abs(candidates @ t)))]
            n = n - np.dot(n, t) * t
            n_norm = np.linalg.norm(n)
        n = n / n_norm

        b = np.cross(t, n)
        b_norm = np.linalg.norm(b)
        if b_norm < 1e-8:
            b = np.array([0.0, 0.0, -1.0])
        else:
            b = b / b_norm
        return n, b

    # ------------------------------------------------------------------
    # 19-column geometry horizon parameters (overrides RacingPath default)
    # ------------------------------------------------------------------

    def get_tunnel_params(self, theta: float, n_stages: int, dt: float,
                          vtheta_guess: float = 3.0,
                          qc_nom: float = 0.5,
                          qc_gate: float = 10.0,
                          theta_values: np.ndarray = None,
                          **kwargs) -> np.ndarray:
        """
        Pre-compute ACADOS stage parameters including tunnel geometry.

        Returns:
            params: (n_stages, 19) with columns:
              [0:3]   p_ref  — nominal path position       p^d(theta_ref)
              [3:6]   t_ref  — unit tangent                t(theta_ref)
              [6]     qc     — dynamic contouring weight   qc(theta_ref)
              [7:10]  n_ref  — horizontal tunnel normal    n(theta_ref)
              [10:13] b_ref  — vertical   tunnel normal    b(theta_ref)
              [13]    W      — tunnel half-width           W(theta_ref)
              [14]    H      — tunnel half-height          H(theta_ref)
              [15]    theta_ref — nominal progress for symbolic p_ref update
              [16:19] c_ref  — curvature vector            p^{d''}(theta_ref),
                               2nd-order Taylor correction of p_ref
        """
        params = np.zeros((n_stages, GEOMETRY_NP))
        thetas = self._resolve_horizon_thetas(
            theta, n_stages, dt, vtheta_guess, theta_values)
        pds, tds, cds = self._lookup_path_many(thetas)
        qcs = self.contouring_weights(pds, qc_nom, qc_gate)
        widths = self.tunnel_widths(thetas)
        heights = self.tunnel_heights(thetas)

        for k, (th, td) in enumerate(zip(thetas, tds)):
            n, b = self.normal_binormal(th, td)

            params[k,  0:3]  = pds[k]
            params[k,  3:6]  = td
            params[k,  6]    = qcs[k]
            params[k,  7:10] = n
            params[k, 10:13] = b
            params[k, 13]    = widths[k]
            params[k, 14]    = heights[k]
            params[k, 15]    = th
            params[k, 16:19] = cds[k]

        return params

    # ------------------------------------------------------------------
    # Convenience: gate arc-length positions
    # ------------------------------------------------------------------

    @property
    def gate_thetas(self) -> np.ndarray:
        """Arc-length (θ) at each gate centre. Shape (n_gates,)."""
        return self._gate_thetas.copy()

    @property
    def n_gates(self) -> int:
        """Number of gates."""
        return len(self._gate_thetas)

    @property
    def circuit_start_theta(self) -> float:
        """Arc-length θ at gate 1 — used to skip the lead-in on lap wrap."""
        return float(self._gate_thetas[0])

    @property
    def circuit_length(self) -> float:
        """Closed gate-circuit length, excluding the optional lead-in."""
        return float(self.total_length - self.circuit_start_theta)
