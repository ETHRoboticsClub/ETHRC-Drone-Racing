#!/usr/bin/env python
"""Load and merge MPCC++ tuning parameters."""

from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


PARAMETERS_ENV_VAR = 'MPCC_PP_PARAMETERS_FILE'
DEFAULT_PARAMETERS_PATH = Path(__file__).resolve().parent / 'parameters.yaml'


def _deep_merge(base: Mapping[str, Any], override: Mapping[str, Any]) -> dict:
    """Return base recursively updated with override values."""
    merged = copy.deepcopy(dict(base))
    for key, value in dict(override).items():
        if (
            isinstance(value, Mapping)
            and isinstance(merged.get(key), Mapping)
        ):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _set_nested(data: dict, keys: tuple[str, ...], value: Any) -> None:
    """Set a nested dictionary value, creating intermediate dictionaries."""
    cursor = data
    for key in keys[:-1]:
        cursor = cursor.setdefault(key, {})
    cursor[keys[-1]] = value


def _load_yaml(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"MPCC++ parameter file not found: {path}")
    with path.open('r') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, Mapping):
        raise ValueError(f"MPCC++ parameter file must contain a YAML mapping: {path}")
    return dict(data)


def load_mpcc_pp_parameters(parameter_file: str | os.PathLike | None = None) -> dict:
    """
    Load MPCC++ parameters.

    Priority:
      1. explicit parameter_file argument
      2. MPCC_PP_PARAMETERS_FILE environment variable
      3. bundled parameters.yaml next to this module
    """
    path_value = parameter_file or os.environ.get(PARAMETERS_ENV_VAR)
    path = Path(path_value).expanduser() if path_value else DEFAULT_PARAMETERS_PATH
    return _load_yaml(path)


def apply_track_profile(parameters: Mapping[str, Any],
                        track_name: str | None) -> dict:
    """Apply a track profile from parameters.yaml, if one exists."""
    merged = copy.deepcopy(dict(parameters))
    profiles = merged.get('track_profiles', {}) or {}
    if not isinstance(profiles, Mapping):
        return merged

    default_profile = profiles.get('default', {}) or {}
    if isinstance(default_profile, Mapping):
        merged = _deep_merge(merged, default_profile)

    if track_name:
        track_profile = profiles.get(track_name, {}) or {}
        if isinstance(track_profile, Mapping):
            merged = _deep_merge(merged, track_profile)

    return merged


_LEGACY_TRACK_MAP = {
    'SHOW_TUNNEL': ('path', 'tunnel', 'show_tunnel'),
    'W_nominal': ('path', 'tunnel', 'W_nominal'),
    'H_nominal': ('path', 'tunnel', 'H_nominal'),
    'sigma_gate': ('path', 'tunnel', 'sigma_gate'),
    'qc_nom': ('path', 'contouring', 'qc_nom'),
    'qc_gate': ('path', 'contouring', 'qc_gate'),
    'sigma_qc': ('path', 'contouring', 'sigma_qc'),
    'track_weight': ('cost', 'track_weight'),
}


def apply_legacy_track_overrides(parameters: Mapping[str, Any],
                                 track_params: Mapping[str, Any] | None) -> dict:
    """
    Optionally merge old gates_config.yaml track keys.

    The new parameters.yaml is authoritative by default. Legacy overrides are
    used only when compatibility.allow_legacy_track_overrides is true.
    """
    merged = copy.deepcopy(dict(parameters))
    compatibility = merged.get('compatibility', {}) or {}
    if not bool(compatibility.get('allow_legacy_track_overrides', False)):
        return merged
    if not track_params:
        return merged

    overrides: dict[str, Any] = {}
    for legacy_key, nested_key in _LEGACY_TRACK_MAP.items():
        if legacy_key in track_params:
            _set_nested(overrides, nested_key, track_params[legacy_key])

    weights = track_params.get('weights')
    if isinstance(weights, Mapping):
        for key, value in weights.items():
            _set_nested(overrides, ('cost', str(key)), value)

    return _deep_merge(merged, overrides)


def get_section(parameters: Mapping[str, Any], *keys: str) -> dict:
    """Return a nested section as a plain dict, or an empty dict if absent."""
    section: Any = parameters
    for key in keys:
        if not isinstance(section, Mapping):
            return {}
        section = section.get(key, {})
    return dict(section or {}) if isinstance(section, Mapping) else {}


def get_float(parameters: Mapping[str, Any], keys: tuple[str, ...],
              default: float) -> float:
    """Read a nested value as float."""
    value: Any = parameters
    for key in keys:
        if not isinstance(value, Mapping):
            return float(default)
        value = value.get(key, default)
    return float(value)


def get_int(parameters: Mapping[str, Any], keys: tuple[str, ...],
            default: int) -> int:
    """Read a nested value as int."""
    value: Any = parameters
    for key in keys:
        if not isinstance(value, Mapping):
            return int(default)
        value = value.get(key, default)
    return int(value)
