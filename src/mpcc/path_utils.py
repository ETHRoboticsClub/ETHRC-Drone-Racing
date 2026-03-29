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
"""

import numpy as np
from scipy.interpolate import CubicSpline


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
    """

    def __init__(self, waypoints: np.ndarray,
                 gate_positions: np.ndarray = None,
                 sigma: float = 0.5):
        assert waypoints.ndim == 2 and waypoints.shape[1] == 3
        assert waypoints.shape[0] >= 2

        self._sigma = sigma
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

    def contouring_weight(self, theta: float,
                          qc_nom: float = 0.5,
                          qc_gate: float = 10.0) -> float:
        """
        Dynamic qc(p^d(θ)) — sum of Gaussians at gate positions.

        qc → qc_gate near a gate (tracking is critical),
        qc → qc_nom on straight segments (progress dominates).
        """
        pd = self.position(theta)
        sigma2 = self._sigma ** 2
        weight = 0.0
        for gp in self._gates:
            diff = pd - gp
            weight += float(np.exp(-0.5 * np.dot(diff, diff) / sigma2))
        # Normalise so that a single Gaussian peaks at 1
        weight = np.clip(weight, 0.0, 1.0)
        return qc_nom + (qc_gate - qc_nom) * weight

    # ------------------------------------------------------------------
    # Horizon parameter construction (called every control step)
    # ------------------------------------------------------------------

    def get_horizon_params(self, theta: float, n_stages: int, dt: float,
                           vtheta_guess: float = 3.0,
                           qc_nom: float = 0.5,
                           qc_gate: float = 10.0) -> np.ndarray:
        """
        Pre-compute ACADOS stage parameters for all n_stages of the OCP.

        In the RTI loop (Algorithm 1) the predicted θ from the previous
        solution is used to evaluate C(θk). Here we approximate those
        predicted thetas by propagating θ forward at `vtheta_guess` m/s.

        Returns:
            params: (n_stages, 7) — columns [px, py, pz, tx, ty, tz, qc].
        """
        params = np.zeros((n_stages, 7))
        th = theta
        for k in range(n_stages):
            pd = self.position(th)
            td = self.tangent(th)
            qc = self.contouring_weight(th, qc_nom, qc_gate)
            params[k, :3] = pd
            params[k, 3:6] = td
            params[k, 6] = qc
            # Advance θ by guessed progress speed
            th = min(th + vtheta_guess * dt, self.total_length)
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

    def _clip(self, theta: float) -> float:
        return float(np.clip(theta, 0.0, self.total_length))
