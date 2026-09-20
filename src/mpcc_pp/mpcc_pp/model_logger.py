"""
Model validation logger.

At each MPCC control step the controller holds:
  x_aug_t   — 19-dim augmented state fed to the OCP
  u_t       — 5-dim optimal control from the OCP
  x_pred_t1 — 19-dim predicted next state (x_pred[1] from OCP)

When the *next* step arrives, x_ext_{t+1} is the real state observed from
the simulator.  This logger buffers the previous-step triple and writes one
CSV row per step once the ground-truth is available.

CSV columns
───────────
  sim_time_s                     — wall-clock time at step t
  x_{label}       (19 cols)      — augmented state at step t
  u_{label}       ( 5 cols)      — control applied at step t
  xpred_{label}   (19 cols)      — model-predicted state at step t+1
  xreal_{label}   (13 cols)      — real state at step t+1 (from simulator)

Augmented state convention (19-dim):
  [p(3), q(4), v(3), ω(3), f(4), θ(1), vθ(1)]

External state convention (13-dim):
  [p(3), v(3), q(4), ω(3)]
"""

import csv
import threading
import time
from pathlib import Path

import numpy as np


_AUG_LABELS = [
    'p_n', 'p_e', 'p_d',
    'q_w', 'q_x', 'q_y', 'q_z',
    'v_n', 'v_e', 'v_d',
    'w_x', 'w_y', 'w_z',
    'f0', 'f1', 'f2', 'f3',
    'theta', 'vtheta',
]
_CTRL_LABELS = ['dvtheta', 'df0', 'df1', 'df2', 'df3']
_EXT_LABELS = [
    'p_n', 'p_e', 'p_d',
    'v_n', 'v_e', 'v_d',
    'q_w', 'q_x', 'q_y', 'q_z',
    'w_x', 'w_y', 'w_z',
]

_HEADER = (
    ['sim_time_s']
    + [f'x_{l}' for l in _AUG_LABELS]
    + [f'u_{l}' for l in _CTRL_LABELS]
    + [f'xpred_{l}' for l in _AUG_LABELS]
    + [f'xreal_{l}' for l in _EXT_LABELS]
)


class ModelValidationLogger:
    """Thread-safe CSV logger for MPCC model-vs-reality validation."""

    def __init__(self, data_dir: str | Path | None = None):
        if data_dir is None:
            data_dir = Path(__file__).parent.parent / 'data'
        data_path = Path(data_dir).expanduser()
        data_path.mkdir(parents=True, exist_ok=True)
        ts = time.strftime('%Y%m%d_%H%M%S')
        csv_path = data_path / f'model_validation_{ts}.csv'

        self._lock = threading.Lock()
        self._pending: tuple | None = None   # (sim_time, x_aug, u_opt, x_pred1)
        self._closed = False

        self._file = open(csv_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(_HEADER)
        self._file.flush()
        print(f'[ModelValidationLogger] Logging to {csv_path}')

    # ------------------------------------------------------------------

    def record(self,
               sim_time: float,
               x_aug: np.ndarray,
               u_opt: np.ndarray,
               x_pred1: np.ndarray,
               x_ext_next: np.ndarray) -> None:
        """
        Write one row using the previous step's buffered prediction and the
        current step's real state, then buffer the current step's data.

        Call at the END of each _run_step with:
          sim_time   — time.monotonic() snapshot taken at the start of this step
          x_aug      — augmented state used this step (19-dim)
          u_opt      — optimal control this step (5-dim)
          x_pred1    — x_pred[1] from the OCP this step (19-dim)
          x_ext_next — raw 13-dim external state received at the START of this step
                       (it is the ground-truth for the *previous* step's prediction)
        """
        with self._lock:
            if self._closed:
                return
            # Flush the previous-step row now that we have the ground truth
            if self._pending is not None:
                prev_time, prev_x, prev_u, prev_pred = self._pending
                row = (
                    [prev_time]
                    + prev_x.tolist()
                    + prev_u.tolist()
                    + prev_pred.tolist()
                    + x_ext_next.tolist()
                )
                self._writer.writerow(row)
            # Buffer this step for the next call
            self._pending = (
                sim_time,
                np.array(x_aug, dtype=np.float64),
                np.array(u_opt, dtype=np.float64),
                np.array(x_pred1, dtype=np.float64),
            )

    def reset(self) -> None:
        """Discard the pending buffer (call on controller reset / new episode)."""
        with self._lock:
            self._pending = None

    def close(self) -> None:
        """Flush and close the CSV file."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._pending = None
            self._file.flush()
            self._file.close()
        print('[ModelValidationLogger] Closed.')

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
