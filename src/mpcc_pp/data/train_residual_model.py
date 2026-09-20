"""
Train a residual dynamics MLP from model_validation CSV data.

Set CSV_PATH to one or more model_validation_*.csv files, then run:
    python3 train_residual_model.py

Output: residual_model.pt  (same directory as this script)
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, random_split

# ── Add mpcc_pp to path so ResidualMLP is importable ──────────────────────
_SRC = Path(__file__).parents[2]   # …/src
sys.path.insert(0, str(_SRC / 'mpcc_pp'))
from mpcc_pp.residual_model import ResidualMLP, NX_IN, NX_OUT

# ── Configure manually ─────────────────────────────────────────────────────
CSV_PATHS = [
    Path(__file__).parent / 'model_validation_20260520_225200.csv',
    # add more files here to train on multiple runs
]
OUT_PATH     = Path(__file__).parent / 'residual_model.pt'
HIDDEN       = (256, 256, 128)
DROPOUT      = 0.1
LR           = 3e-4
BATCH_SIZE   = 512
MAX_EPOCHS   = 300
PATIENCE     = 20          # early stopping patience (epochs without val improvement)
VAL_FRACTION = 0.15
SEED         = 42
# ──────────────────────────────────────────────────────────────────────────

STATE_COLS = [
    'x_p_n', 'x_p_e', 'x_p_d',
    'x_q_w', 'x_q_x', 'x_q_y', 'x_q_z',
    'x_v_n', 'x_v_e', 'x_v_d',
    'x_w_x', 'x_w_y', 'x_w_z',
    'x_f0', 'x_f1', 'x_f2', 'x_f3',
    'x_theta', 'x_vtheta',
]
CONTROL_COLS = ['u_dvtheta', 'u_df0', 'u_df1', 'u_df2', 'u_df3']
FEATURE_COLS = STATE_COLS + CONTROL_COLS   # 24

# Pairs used to compute residuals inline (same logic as compute_residuals.py)
PAIRS = [
    ('xpred_p_n',  'xreal_p_n'),
    ('xpred_p_e',  'xreal_p_e'),
    ('xpred_p_d',  'xreal_p_d'),
    ('xpred_q_w',  'xreal_q_w'),
    ('xpred_q_x',  'xreal_q_x'),
    ('xpred_q_y',  'xreal_q_y'),
    ('xpred_q_z',  'xreal_q_z'),
    ('xpred_v_n',  'xreal_v_n'),
    ('xpred_v_e',  'xreal_v_e'),
    ('xpred_v_d',  'xreal_v_d'),
    ('xpred_w_x',  'xreal_w_x'),
    ('xpred_w_y',  'xreal_w_y'),
    ('xpred_w_z',  'xreal_w_z'),
]
RESIDUAL_NAMES = [f'res_{p[0].split("_",1)[1]}' for p in PAIRS]


def load_data() -> tuple[np.ndarray, np.ndarray]:
    frames = []
    for p in CSV_PATHS:
        p = Path(p).expanduser().resolve()
        if not p.exists():
            raise FileNotFoundError(f'CSV not found: {p}')
        frames.append(pd.read_csv(p))
    df = pd.concat(frames, ignore_index=True)
    df = df.dropna()

    X = df[FEATURE_COLS].values.astype(np.float32)

    Y = np.stack(
        [df[pred].values - df[real].values for pred, real in PAIRS],
        axis=1,
    ).astype(np.float32)

    print(f'Loaded {len(df)} steps from {len(CSV_PATHS)} file(s)')
    print(f'X shape: {X.shape}  Y shape: {Y.shape}')
    return X, Y


def train():
    torch.manual_seed(SEED)
    np.random.seed(SEED)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'Device: {device}')

    X, Y = load_data()

    # Normalisation statistics (computed on full dataset — small enough)
    x_mean = X.mean(0)
    x_std  = X.std(0) + 1e-8
    y_mean = Y.mean(0)
    y_std  = Y.std(0) + 1e-8

    X_norm = (X - x_mean) / x_std
    Y_norm = (Y - y_mean) / y_std

    dataset = TensorDataset(
        torch.from_numpy(X_norm),
        torch.from_numpy(Y_norm),
    )
    n_val  = max(1, int(len(dataset) * VAL_FRACTION))
    n_train = len(dataset) - n_val
    train_ds, val_ds = random_split(
        dataset, [n_train, n_val],
        generator=torch.Generator().manual_seed(SEED),
    )
    train_dl = DataLoader(train_ds, batch_size=BATCH_SIZE, shuffle=True)
    val_dl   = DataLoader(val_ds,   batch_size=BATCH_SIZE)

    model = ResidualMLP(hidden=HIDDEN, dropout=DROPOUT).to(device)
    opt   = torch.optim.Adam(model.parameters(), lr=LR)
    sched = torch.optim.lr_scheduler.ReduceLROnPlateau(opt, patience=8, factor=0.5)
    loss_fn = nn.MSELoss()

    best_val = float('inf')
    best_state = None
    no_improve = 0

    print(f'\n{"Epoch":>6}  {"train MSE":>12}  {"val MSE":>12}  {"val RMSE":>10}')
    for epoch in range(1, MAX_EPOCHS + 1):
        # ── train ──
        model.train()
        train_loss = 0.0
        for xb, yb in train_dl:
            xb, yb = xb.to(device), yb.to(device)
            opt.zero_grad()
            loss = loss_fn(model(xb), yb)
            loss.backward()
            opt.step()
            train_loss += loss.item() * len(xb)
        train_loss /= n_train

        # ── validate ──
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for xb, yb in val_dl:
                xb, yb = xb.to(device), yb.to(device)
                val_loss += loss_fn(model(xb), yb).item() * len(xb)
        val_loss /= n_val
        val_rmse = float(np.sqrt(val_loss))

        sched.step(val_loss)

        if epoch % 10 == 0 or epoch == 1:
            print(f'{epoch:>6}  {train_loss:>12.6f}  {val_loss:>12.6f}  {val_rmse:>10.6f}')

        if val_loss < best_val - 1e-7:
            best_val = val_loss
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
            no_improve = 0
        else:
            no_improve += 1
            if no_improve >= PATIENCE:
                print(f'Early stopping at epoch {epoch}')
                break

    # ── save ──
    model.load_state_dict(best_state)
    torch.save({
        'model_state': best_state,
        'hidden':  list(HIDDEN),
        'dropout': DROPOUT,
        'x_mean':  x_mean,
        'x_std':   x_std,
        'y_mean':  y_mean,
        'y_std':   y_std,
        'feature_cols':  FEATURE_COLS,
        'residual_names': RESIDUAL_NAMES,
    }, OUT_PATH)
    print(f'\nBest val RMSE (normalised): {np.sqrt(best_val):.6f}')
    print(f'Model saved → {OUT_PATH}')

    # ── per-channel RMSE on validation set (denormalised) ──
    model.eval()
    all_pred, all_true = [], []
    with torch.no_grad():
        for xb, yb in val_dl:
            pred = model(xb.to(device)).cpu().numpy()
            all_pred.append(pred * y_std + y_mean)
            all_true.append(yb.numpy() * y_std + y_mean)
    pred_all = np.concatenate(all_pred)
    true_all = np.concatenate(all_true)
    err = pred_all - true_all

    print(f'\nValidation per-channel RMSE (original units)\n{"─"*45}')
    print(f'{"channel":<14}  {"RMSE":>10}')
    for i, name in enumerate(RESIDUAL_NAMES):
        rmse = float(np.sqrt(np.mean(err[:, i] ** 2)))
        print(f'{name:<14}  {rmse:>10.6f}')


if __name__ == '__main__':
    train()
