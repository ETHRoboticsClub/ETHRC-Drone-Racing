"""
Fit a paper-style polynomial residual model from model_validation CSV logs.

This follows the residual structure proposed in the MPCC++ paper more closely
than train_residual_model.py:
  - small polynomial feature sets
  - linear least-squares fit
  - target residual enters the dynamics model directly

Usage:
    python3 fit_paper_residual_model.py

Output:
    paper_residual_model.json
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

_SRC = Path(__file__).parents[2]
sys.path.insert(0, str(_SRC / 'mpcc_pp'))

from mpcc_pp.paper_residual_model import (  # noqa: E402
    ACCEL_AXES,
    ANGULAR_AXES,
    FEATURE_NAMES,
    MODEL_AXES,
    PaperResidualLinearModel,
    build_feature_map,
    rotation_matrix_from_quaternion,
    velocity_body_from_ned,
    omega_sq_proxy_from_thrust,
)


# ── Configure manually ─────────────────────────────────────────────────────
CSV_PATHS = sorted(
    p for p in Path(__file__).parent.glob('model_validation_*.csv')
    if '_residuals' not in p.name and '_correlations' not in p.name
)
OUT_PATH = Path(__file__).parent / 'paper_residual_model.json'
VALIDATION_FRACTION = 0.2
SEED = 42
DT_MIN_S = 0.005
DT_MAX_S = 0.05

# The paper uses mean squared rotor speed Ω². If your logs only contain rotor
# thrust, `mean_thrust` is acceptable because the unknown scaling can be
# absorbed into the learned coefficients.
OMEGA_SQ_SOURCE = 'mean_thrust'   # or 'thrust_over_kf'
ROTOR_FORCE_CONSTANT = None       # set when OMEGA_SQ_SOURCE='thrust_over_kf'
# ───────────────────────────────────────────────────────────────────────────


def _resolve_csv_paths() -> list[Path]:
    paths = [Path(p).expanduser().resolve() for p in CSV_PATHS]
    if not paths:
        raise ValueError("CSV_PATHS is empty")
    missing = [str(p) for p in paths if not p.exists()]
    if missing:
        raise FileNotFoundError(f"Missing CSV file(s): {missing}")
    return paths


def _estimate_nominal_dt(df: pd.DataFrame) -> float:
    dt = np.diff(df['sim_time_s'].values)
    valid = dt[np.isfinite(dt)]
    valid = valid[(valid >= DT_MIN_S) & (valid <= DT_MAX_S)]
    if len(valid) == 0:
        raise ValueError(
            "Unable to estimate nominal dt from sim_time_s. "
            "Adjust DT_MIN_S / DT_MAX_S if needed."
        )
    return float(np.median(valid))


def _r2_score(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = float(np.sum((y_true - y_pred) ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    if ss_tot < 1e-12:
        return 0.0
    return 1.0 - ss_res / ss_tot


def _rmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_true - y_pred) ** 2)))


def build_dataset() -> tuple[dict[str, np.ndarray], dict[str, np.ndarray], dict[str, float]]:
    feature_rows = {axis: [] for axis in MODEL_AXES}
    targets = {axis: [] for axis in MODEL_AXES}
    dt_by_file: dict[str, float] = {}

    for csv_path in _resolve_csv_paths():
        df = pd.read_csv(csv_path).dropna()
        nominal_dt = _estimate_nominal_dt(df)
        dt_by_file[csv_path.name] = nominal_dt

        for row in df.itertuples(index=False):
            quaternion = np.array([row.x_q_w, row.x_q_x, row.x_q_y, row.x_q_z], dtype=np.float64)
            velocity_ned = np.array([row.x_v_n, row.x_v_e, row.x_v_d], dtype=np.float64)
            rotor_thrust = np.array([row.x_f0, row.x_f1, row.x_f2, row.x_f3], dtype=np.float64)

            rotation_bn = rotation_matrix_from_quaternion(quaternion)
            velocity_body = velocity_body_from_ned(quaternion, velocity_ned)
            omega_sq = omega_sq_proxy_from_thrust(
                rotor_thrust,
                source=OMEGA_SQ_SOURCE,
                rotor_force_constant=ROTOR_FORCE_CONSTANT,
            )
            features = build_feature_map(velocity_body, omega_sq)

            velocity_residual_ned = np.array([
                row.xreal_v_n - row.xpred_v_n,
                row.xreal_v_e - row.xpred_v_e,
                row.xreal_v_d - row.xpred_v_d,
            ], dtype=np.float64)
            accel_residual_body = (rotation_bn.T @ velocity_residual_ned) / nominal_dt

            angular_residual_body = np.array([
                row.xreal_w_x - row.xpred_w_x,
                row.xreal_w_y - row.xpred_w_y,
                row.xreal_w_z - row.xpred_w_z,
            ], dtype=np.float64) / nominal_dt

            for axis in ACCEL_AXES:
                feature_rows[axis].append(features[axis])
            for axis in ANGULAR_AXES:
                feature_rows[axis].append(features[axis])

            targets['ax'].append(accel_residual_body[0])
            targets['ay'].append(accel_residual_body[1])
            targets['az'].append(accel_residual_body[2])
            targets['wx'].append(angular_residual_body[0])
            targets['wy'].append(angular_residual_body[1])
            targets['wz'].append(angular_residual_body[2])

    feature_arrays = {
        axis: np.asarray(rows, dtype=np.float64) for axis, rows in feature_rows.items()
    }
    target_arrays = {
        axis: np.asarray(rows, dtype=np.float64) for axis, rows in targets.items()
    }
    return feature_arrays, target_arrays, dt_by_file


def main() -> None:
    np.random.seed(SEED)
    features, targets, dt_by_file = build_dataset()
    n_samples = len(next(iter(targets.values())))
    if n_samples < 10:
        raise ValueError(f"Dataset too small: {n_samples} samples")

    indices = np.random.permutation(n_samples)
    n_val = max(1, int(n_samples * VALIDATION_FRACTION))
    val_idx = indices[:n_val]
    train_idx = indices[n_val:]

    coefficients: dict[str, np.ndarray] = {}
    train_metrics: dict[str, dict[str, float]] = {}
    validation_metrics: dict[str, dict[str, float]] = {}

    print(f'Loaded {n_samples} samples from {len(dt_by_file)} file(s)')
    print('Nominal dt by file:')
    for name, nominal_dt in dt_by_file.items():
        print(f'  {name}: {nominal_dt:.6f} s')

    print(f'\n{"Axis":<4}  {"#feat":>5}  {"Train R2":>10}  {"Train RMSE":>12}  {"Val R2":>8}  {"Val RMSE":>10}')
    print('-' * 64)
    for axis in MODEL_AXES:
        X = features[axis]
        y = targets[axis]
        coef, *_ = np.linalg.lstsq(X[train_idx], y[train_idx], rcond=None)
        coefficients[axis] = coef

        pred_train = X[train_idx] @ coef
        pred_val = X[val_idx] @ coef
        train_metrics[axis] = {
            'r2': _r2_score(y[train_idx], pred_train),
            'rmse': _rmse(y[train_idx], pred_train),
        }
        validation_metrics[axis] = {
            'r2': _r2_score(y[val_idx], pred_val),
            'rmse': _rmse(y[val_idx], pred_val),
        }
        print(
            f'{axis:<4}  {len(FEATURE_NAMES[axis]):>5d}  '
            f'{train_metrics[axis]["r2"]:>10.3f}  '
            f'{train_metrics[axis]["rmse"]:>12.5f}  '
            f'{validation_metrics[axis]["r2"]:>8.3f}  '
            f'{validation_metrics[axis]["rmse"]:>10.5f}'
        )

    nominal_dt = float(np.mean(list(dt_by_file.values())))
    model = PaperResidualLinearModel(
        coefficients=coefficients,
        omega_sq_source=OMEGA_SQ_SOURCE,
        rotor_force_constant=ROTOR_FORCE_CONSTANT,
        dt_seconds=nominal_dt,
        metadata={
            'csv_files': [str(p) for p in _resolve_csv_paths()],
            'train_metrics': train_metrics,
            'validation_metrics': validation_metrics,
            'dt_by_file_s': dt_by_file,
            'notes': [
                "Targets are body-frame residual linear acceleration and "
                "body-frame residual angular acceleration derived from the "
                "logged one-step prediction error.",
                "This is closer to the MPCC++ paper than the MLP post-solve "
                "correction, but still depends on the fidelity of the logged dt "
                "and the thrust proxy used for omega_sq.",
            ],
        },
    )
    model.save(OUT_PATH)

    print(f'\nSaved → {OUT_PATH}')
    print('\nCoefficients:')
    for axis in MODEL_AXES:
        print(f'  {axis}: {np.round(coefficients[axis], 6).tolist()}')


if __name__ == '__main__':
    main()
