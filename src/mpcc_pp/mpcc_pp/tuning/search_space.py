#!/usr/bin/env python
"""Search-space helpers for MPCC++ Bayesian optimization."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Mapping


@dataclass(frozen=True)
class ParameterSpec:
    """One scalar tuning parameter."""

    name: str
    low: float
    high: float
    scale: str = 'log'

    def decode(self, z_value: float) -> float:
        """Map a normalized value in [0, 1] to the physical parameter."""
        z = min(1.0, max(0.0, float(z_value)))
        if self.scale == 'log':
            lo = math.log10(self.low)
            hi = math.log10(self.high)
            return float(10.0 ** (lo + z * (hi - lo)))
        if self.scale == 'linear':
            return float(self.low + z * (self.high - self.low))
        raise ValueError(f"Unsupported scale for {self.name}: {self.scale}")

    def suggest(self, trial) -> float:
        """Ask an Optuna trial for this parameter."""
        if self.scale == 'log':
            return float(trial.suggest_float(
                self.name, self.low, self.high, log=True))
        if self.scale == 'linear':
            return float(trial.suggest_float(
                self.name, self.low, self.high))
        raise ValueError(f"Unsupported scale for {self.name}: {self.scale}")


class SearchSpace:
    """
    Default MPCC++ tuning space.

    qc_gate is represented as qc_nom * qc_gate_ratio so Optuna never proposes a
    gate tracking weight below the nominal contouring weight.
    """

    def __init__(self, specs: Iterable[ParameterSpec] | None = None):
        self.specs = tuple(specs or (
            ParameterSpec('ql', 1e-2, 1e2),
            ParameterSpec('qc_nom', 1e-3, 1e1),
            ParameterSpec('qc_gate_ratio', 1.0, 50.0),
            ParameterSpec('Qw', 1e-7, 1e-2),
            ParameterSpec('r_dvt', 1e-7, 1e-1),
            ParameterSpec('R_df', 1e-7, 1e-1),
            ParameterSpec('mu', 1e-2, 1e2),
            ParameterSpec('track_weight', 1.0, 1e3),
        ))

    @property
    def dim(self) -> int:
        """Number of optimized scalar variables."""
        return len(self.specs)

    def decode(self, z_values: Iterable[float]) -> dict[str, float]:
        """Decode normalized [0, 1] coordinates into MPCC++ parameters."""
        values = {
            spec.name: spec.decode(z)
            for spec, z in zip(self.specs, z_values)
        }
        return self.expand(values)

    def suggest(self, trial) -> dict[str, float]:
        """Suggest one MPCC++ parameter vector from an Optuna trial."""
        values = {spec.name: spec.suggest(trial) for spec in self.specs}
        return self.expand(values)

    def expand(self, values: Mapping[str, float]) -> dict[str, float]:
        """Add derived parameters such as qc_gate."""
        params = {key: float(value) for key, value in values.items()}
        qc_nom = params['qc_nom']
        qc_gate_ratio = params['qc_gate_ratio']
        params['qc_gate'] = qc_nom * qc_gate_ratio
        return params


def tuning_yaml_patch(params: Mapping[str, float]) -> dict:
    """Convert flat tuning parameters into a parameters.yaml patch."""
    return {
        'path': {
            'contouring': {
                'qc_nom': float(params['qc_nom']),
                'qc_gate': float(params['qc_gate']),
            },
        },
        'cost': {
            'ql': float(params['ql']),
            'Qw': float(params['Qw']),
            'r_dvt': float(params['r_dvt']),
            'R_df': float(params['R_df']),
            'mu': float(params['mu']),
            'track_weight': float(params['track_weight']),
        },
    }

