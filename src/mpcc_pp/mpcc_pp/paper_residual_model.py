"""Paper-style polynomial residual model for MPCC++.

This module implements the lightweight residual structure described in
Krinner et al., "MPCC++: Model Predictive Contouring Control for
Time-Optimal Flight with Safety Constraints" (RSS 2024).

The paper models aerodynamic force / torque effects with small polynomial
feature sets built from body-frame velocity and the mean squared rotor
speed. In this repository we learn the equivalent correction on body-frame
linear acceleration and body-frame angular acceleration, because the
available logging data stores state transitions rather than force/torque
ground truth directly.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping

import numpy as np


ACCEL_AXES = ('ax', 'ay', 'az')
ANGULAR_AXES = ('wx', 'wy', 'wz')
MODEL_AXES = ACCEL_AXES + ANGULAR_AXES

FEATURE_NAMES: dict[str, tuple[str, ...]] = {
    'ax': ('vx', 'vx_cubed', 'omega_sq', 'vx_omega_sq'),
    'ay': ('vy', 'vy_cubed', 'omega_sq', 'vy_omega_sq'),
    'az': (
        'vz',
        'vz_cubed',
        'vxy',
        'vxy_squared',
        'vxy_omega_sq',
        'vz_omega_sq',
        'vxy_vz_omega_sq',
    ),
    'wx': ('vy', 'omega_sq', 'vy_omega_sq'),
    'wy': ('vx', 'omega_sq', 'vx_omega_sq'),
    'wz': ('vx', 'vy'),
}


def rotation_matrix_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    """Return body-to-NED rotation from quaternion [w, x, y, z]."""
    q = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return np.eye(3, dtype=np.float64)
    qw, qx, qy, qz = q / norm
    return np.array([
        [1.0 - 2.0 * (qy**2 + qz**2), 2.0 * (qx * qy - qw * qz), 2.0 * (qx * qz + qw * qy)],
        [2.0 * (qx * qy + qw * qz), 1.0 - 2.0 * (qx**2 + qz**2), 2.0 * (qy * qz - qw * qx)],
        [2.0 * (qx * qz - qw * qy), 2.0 * (qy * qz + qw * qx), 1.0 - 2.0 * (qx**2 + qy**2)],
    ], dtype=np.float64)


def velocity_body_from_ned(quaternion: np.ndarray, velocity_ned: np.ndarray) -> np.ndarray:
    """Convert NED velocity to body-frame velocity using q body->NED."""
    rotation_bn = rotation_matrix_from_quaternion(quaternion)
    return rotation_bn.T @ np.asarray(velocity_ned, dtype=np.float64).reshape(3)


def omega_sq_proxy_from_thrust(
    rotor_thrust: np.ndarray,
    source: str,
    rotor_force_constant: float | None,
) -> float:
    """Return the scalar proxy used in place of the paper's mean rotor speed squared."""
    mean_thrust = float(np.mean(np.asarray(rotor_thrust, dtype=np.float64).reshape(4)))
    if source == 'mean_thrust':
        return mean_thrust
    if source == 'thrust_over_kf':
        if rotor_force_constant is None or rotor_force_constant <= 0.0:
            raise ValueError(
                "rotor_force_constant must be positive when omega_sq_source='thrust_over_kf'"
            )
        return mean_thrust / float(rotor_force_constant)
    raise ValueError(
        "omega_sq_source must be 'mean_thrust' or 'thrust_over_kf', "
        f"got {source!r}"
    )


def build_feature_map(v_body: np.ndarray, omega_sq: float) -> dict[str, np.ndarray]:
    """Build all paper-style feature vectors from body velocity and omega_sq."""
    vx, vy, vz = np.asarray(v_body, dtype=np.float64).reshape(3)
    vxy = float(np.sqrt(vx * vx + vy * vy))
    return {
        'ax': np.array([vx, vx**3, omega_sq, vx * omega_sq], dtype=np.float64),
        'ay': np.array([vy, vy**3, omega_sq, vy * omega_sq], dtype=np.float64),
        'az': np.array([
            vz,
            vz**3,
            vxy,
            vxy**2,
            vxy * omega_sq,
            vz * omega_sq,
            vxy * vz * omega_sq,
        ], dtype=np.float64),
        'wx': np.array([vy, omega_sq, vy * omega_sq], dtype=np.float64),
        'wy': np.array([vx, omega_sq, vx * omega_sq], dtype=np.float64),
        'wz': np.array([vx, vy], dtype=np.float64),
    }


@dataclass(frozen=True)
class PaperResidualLinearModel:
    """Linear residual predictor using the paper's polynomial feature structure."""

    coefficients: Mapping[str, np.ndarray]
    omega_sq_source: str = 'mean_thrust'
    rotor_force_constant: float | None = None
    dt_seconds: float | None = None
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        normalized: dict[str, np.ndarray] = {}
        for axis in MODEL_AXES:
            if axis not in self.coefficients:
                raise ValueError(f"Missing coefficients for axis {axis!r}")
            coeff = np.asarray(self.coefficients[axis], dtype=np.float64).reshape(-1)
            expected = len(FEATURE_NAMES[axis])
            if coeff.shape != (expected,):
                raise ValueError(
                    f"Axis {axis!r} expects {expected} coefficients, got shape {coeff.shape}"
                )
            normalized[axis] = coeff
        object.__setattr__(self, 'coefficients', normalized)
        object.__setattr__(self, 'metadata', dict(self.metadata or {}))

    @classmethod
    def load(cls, path: str | Path) -> 'PaperResidualLinearModel':
        """Load a JSON model generated by fit_paper_residual_model.py."""
        json_path = Path(path).expanduser().resolve()
        with json_path.open('r') as f:
            raw = json.load(f)
        coeffs = raw.get('coefficients', {})
        if not isinstance(coeffs, Mapping):
            raise ValueError(f"Invalid coefficients block in {json_path}")
        return cls(
            coefficients={axis: coeffs[axis] for axis in MODEL_AXES},
            omega_sq_source=str(raw.get('omega_sq_source', 'mean_thrust')),
            rotor_force_constant=(
                None
                if raw.get('rotor_force_constant_n_per_rad_s2') in (None, '')
                else float(raw['rotor_force_constant_n_per_rad_s2'])
            ),
            dt_seconds=(
                None if raw.get('nominal_dt_s') in (None, '') else float(raw['nominal_dt_s'])
            ),
            metadata={
                'model_path': str(json_path),
                'feature_names': raw.get('feature_names', {}),
                'train_metrics': raw.get('train_metrics', {}),
                'validation_metrics': raw.get('validation_metrics', {}),
                'csv_files': raw.get('csv_files', []),
            },
        )

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable representation."""
        return {
            'model_type': 'paper_polynomial_residual',
            'target_type': 'body_accel_and_body_angular_accel',
            'omega_sq_source': self.omega_sq_source,
            'rotor_force_constant_n_per_rad_s2': self.rotor_force_constant,
            'nominal_dt_s': self.dt_seconds,
            'feature_names': {axis: list(FEATURE_NAMES[axis]) for axis in MODEL_AXES},
            'coefficients': {
                axis: self.coefficients[axis].astype(float).tolist()
                for axis in MODEL_AXES
            },
            **dict(self.metadata),
        }

    def save(self, path: str | Path) -> None:
        """Write the model to JSON."""
        json_path = Path(path).expanduser().resolve()
        json_path.parent.mkdir(parents=True, exist_ok=True)
        with json_path.open('w') as f:
            json.dump(self.to_dict(), f, indent=2, sort_keys=True)
            f.write('\n')

    def predict(
        self,
        quaternion: np.ndarray,
        velocity_ned: np.ndarray,
        rotor_thrust: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Predict body-frame residual linear / angular accelerations."""
        v_body = velocity_body_from_ned(quaternion, velocity_ned)
        omega_sq = omega_sq_proxy_from_thrust(
            rotor_thrust,
            source=self.omega_sq_source,
            rotor_force_constant=self.rotor_force_constant,
        )
        features = build_feature_map(v_body, omega_sq)
        accel = np.array([
            float(np.dot(self.coefficients[axis], features[axis]))
            for axis in ACCEL_AXES
        ], dtype=np.float64)
        angular = np.array([
            float(np.dot(self.coefficients[axis], features[axis]))
            for axis in ANGULAR_AXES
        ], dtype=np.float64)
        return accel, angular
