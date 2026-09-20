"""
Residual dynamics model for MPCC++.

Maps (x_aug, u) → residual between the nominal model's 1-step prediction
and the real next state observed in the simulator.

Input  (24-dim): augmented state x [19] + control u [5]
Output (13-dim): additive correction to x_pred[1] for the observable channels
  [0:3]   Δp  (NED position)
  [3:7]   Δq  (quaternion)
  [7:10]  Δv  (NED velocity)
  [10:13] Δω  (body rates)

The correction is applied to x_pred[1] indices [0:13] directly.
"""

from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

NX_IN  = 24   # 19 state + 5 control
NX_OUT = 13   # observable residual channels


class ResidualMLP(nn.Module):
    """Feed-forward MLP for residual dynamics."""

    def __init__(self, hidden: tuple[int, ...] = (256, 256, 128), dropout: float = 0.1):
        super().__init__()
        layers = []
        in_dim = NX_IN
        for h in hidden:
            layers += [nn.Linear(in_dim, h), nn.ReLU(), nn.Dropout(dropout)]
            in_dim = h
        layers.append(nn.Linear(in_dim, NX_OUT))
        self.net = nn.Sequential(*layers)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


class ResidualModelPredictor:
    """
    Wraps a trained ResidualMLP for zero-copy numpy inference.

    Usage:
        predictor = ResidualModelPredictor.load('path/to/residual_model.pt')
        delta = predictor.predict(x_aug, u_opt)   # (13,) numpy array
        x_pred1[0:13] += delta
    """

    def __init__(self, model: ResidualMLP,
                 x_mean: np.ndarray, x_std: np.ndarray,
                 y_mean: np.ndarray, y_std: np.ndarray,
                 device: str = 'cpu'):
        self._model = model.to(device).eval()
        self._device = device
        self._x_mean = torch.tensor(x_mean, dtype=torch.float32, device=device)
        self._x_std  = torch.tensor(x_std,  dtype=torch.float32, device=device)
        self._y_mean = torch.tensor(y_mean, dtype=torch.float32, device=device)
        self._y_std  = torch.tensor(y_std,  dtype=torch.float32, device=device)

    @classmethod
    def load(cls, path: str | Path, device: str = 'cpu') -> 'ResidualModelPredictor':
        ckpt = torch.load(path, map_location=device, weights_only=False)
        hidden  = ckpt['hidden']
        dropout = ckpt.get('dropout', 0.0)
        model   = ResidualMLP(hidden=tuple(hidden), dropout=dropout)
        model.load_state_dict(ckpt['model_state'])
        return cls(model,
                   ckpt['x_mean'], ckpt['x_std'],
                   ckpt['y_mean'], ckpt['y_std'],
                   device=device)

    @torch.no_grad()
    def predict(self, x_aug: np.ndarray, u_opt: np.ndarray) -> np.ndarray:
        """
        Args:
            x_aug: (19,) augmented state
            u_opt: (5,)  control input
        Returns:
            delta: (13,) additive correction for x_pred[1][0:13]
        """
        feat = np.concatenate([x_aug, u_opt]).astype(np.float32)
        t = torch.from_numpy(feat).to(self._device)
        t_norm = (t - self._x_mean) / self._x_std
        out_norm = self._model(t_norm.unsqueeze(0)).squeeze(0)
        out = out_norm * self._y_std + self._y_mean
        return out.cpu().numpy()
