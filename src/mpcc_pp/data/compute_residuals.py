"""
Compute prediction residuals from a model_validation CSV.

Set CSV_PATH to the file you want to analyse, then run:
    python3 compute_residuals.py

Outputs:
  <stem>_residuals.csv    — per-step residuals (xpred - xreal) + norm columns
  <stem>_correlations.csv — Pearson r matrix: rows = residuals, cols = state + control
"""

from pathlib import Path

import numpy as np
import pandas as pd

# ── Configure manually ──────────────────────────────────────────────────────
CSV_PATH = Path(__file__).parent / 'model_validation_20260520_225200.csv'
# ───────────────────────────────────────────────────────────────────────────

# Pairs: (predicted_col, real_col)
# xpred uses augmented convention [p,q,v,ω,f,θ,vθ]
# xreal uses external convention  [p,v,q,ω]
PAIRS = [
    # position
    ('xpred_p_n', 'xreal_p_n'),
    ('xpred_p_e', 'xreal_p_e'),
    ('xpred_p_d', 'xreal_p_d'),
    # quaternion
    ('xpred_q_w', 'xreal_q_w'),
    ('xpred_q_x', 'xreal_q_x'),
    ('xpred_q_y', 'xreal_q_y'),
    ('xpred_q_z', 'xreal_q_z'),
    # velocity
    ('xpred_v_n', 'xreal_v_n'),
    ('xpred_v_e', 'xreal_v_e'),
    ('xpred_v_d', 'xreal_v_d'),
    # angular rates
    ('xpred_w_x', 'xreal_w_x'),
    ('xpred_w_y', 'xreal_w_y'),
    ('xpred_w_z', 'xreal_w_z'),
]

RESIDUAL_NAMES = [
    'res_p_n', 'res_p_e', 'res_p_d',
    'res_q_w', 'res_q_x', 'res_q_y', 'res_q_z',
    'res_v_n', 'res_v_e', 'res_v_d',
    'res_w_x', 'res_w_y', 'res_w_z',
]

# State columns (x_*) and control columns (u_*) used as features for correlation
STATE_COLS = [
    'x_p_n', 'x_p_e', 'x_p_d',
    'x_q_w', 'x_q_x', 'x_q_y', 'x_q_z',
    'x_v_n', 'x_v_e', 'x_v_d',
    'x_w_x', 'x_w_y', 'x_w_z',
    'x_f0', 'x_f1', 'x_f2', 'x_f3',
    'x_theta', 'x_vtheta',
]
CONTROL_COLS = ['u_dvtheta', 'u_df0', 'u_df1', 'u_df2', 'u_df3']
FEATURE_COLS = STATE_COLS + CONTROL_COLS


def main():
    csv_path = Path(CSV_PATH).expanduser().resolve()
    if not csv_path.exists():
        raise FileNotFoundError(f'CSV not found: {csv_path}')

    df = pd.read_csv(csv_path)

    out = pd.DataFrame()
    out['sim_time_s'] = df['sim_time_s']

    for name, (pred_col, real_col) in zip(RESIDUAL_NAMES, PAIRS):
        out[name] = df[pred_col].values - df[real_col].values

    # Per-channel RMSE printed to stdout
    print(f'\nResidual statistics ({len(df)} steps)\n{"─"*50}')
    print(f'{"Channel":<12}  {"RMSE":>10}  {"mean":>10}  {"std":>10}')
    for name in RESIDUAL_NAMES:
        vals = out[name].values
        rmse = float(np.sqrt(np.mean(vals**2)))
        print(f'{name:<12}  {rmse:>10.5f}  {np.mean(vals):>10.5f}  {np.std(vals):>10.5f}')

    # Also add position and velocity norm residuals for convenience
    out['res_pos_norm'] = np.linalg.norm(
        out[['res_p_n', 'res_p_e', 'res_p_d']].values, axis=1)
    out['res_vel_norm'] = np.linalg.norm(
        out[['res_v_n', 'res_v_e', 'res_v_d']].values, axis=1)
    out['res_omega_norm'] = np.linalg.norm(
        out[['res_w_x', 'res_w_y', 'res_w_z']].values, axis=1)

    out_path = csv_path.parent / (csv_path.stem + '_residuals.csv')
    out.to_csv(out_path, index=False)
    print(f'\nSaved → {out_path}')

    # ── Correlation matrix: features (state + control) vs residuals ─────────
    missing = [c for c in FEATURE_COLS if c not in df.columns]
    if missing:
        print(f'\n[WARN] Missing feature columns, skipping correlations: {missing}')
        return

    features = df[FEATURE_COLS]
    residuals = out[RESIDUAL_NAMES]

    # Pearson r between every feature and every residual
    corr = pd.DataFrame(index=RESIDUAL_NAMES, columns=FEATURE_COLS, dtype=float)
    for res_col in RESIDUAL_NAMES:
        for feat_col in FEATURE_COLS:
            r = np.corrcoef(features[feat_col].values, residuals[res_col].values)[0, 1]
            corr.loc[res_col, feat_col] = round(float(r), 6)

    corr.index.name = 'residual \\ feature'
    corr_path = csv_path.parent / (csv_path.stem + '_correlations.csv')
    corr.to_csv(corr_path)
    print(f'Saved → {corr_path}')

    # Print the strongest correlations (|r| > 0.3) to stdout
    print(f'\nStrongest correlations (|r| > 0.30)\n{"─"*50}')
    print(f'{"residual":<14}  {"feature":<16}  {"r":>8}')
    rows = []
    for res_col in RESIDUAL_NAMES:
        for feat_col in FEATURE_COLS:
            r = corr.loc[res_col, feat_col]
            if abs(r) > 0.30:
                rows.append((abs(r), res_col, feat_col, r))
    for _, res_col, feat_col, r in sorted(rows, reverse=True):
        print(f'{res_col:<14}  {feat_col:<16}  {r:>8.4f}')


if __name__ == '__main__':
    main()
