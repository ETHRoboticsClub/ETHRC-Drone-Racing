#!/usr/bin/env python
"""Optuna runner for MPCC++ parameter tuning.

The tuner owns the Optuna study, writes trial-specific parameter files, runs a
fresh simulator episode for each repeat, and reduces the collected lap/solver
metrics into one scalar reward.
"""

from __future__ import annotations

import argparse
import copy
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import re
import selectors
import shlex
import signal
import statistics
import subprocess
import sys
import time
from typing import Any, Mapping

import yaml

from ..parameters import DEFAULT_PARAMETERS_PATH
from .search_space import SearchSpace, tuning_yaml_patch


def _default_output_dir() -> Path:
    """Prefer the source package output directory when run from a workspace."""
    cwd = Path.cwd().resolve()
    for base in (cwd, *cwd.parents):
        source_dir = base / 'src' / 'mpcc_pp'
        if (source_dir / 'package.xml').is_file():
            return source_dir / 'mpcc_pp_optuna_runs'
    return Path(__file__).resolve().parents[2] / 'mpcc_pp_optuna_runs'


DEFAULT_OUTPUT_DIR = _default_output_dir()
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / 'optuna_config.yaml'

# Fixed evaluator plumbing. The YAML/CLI surface stays focused on tuning
# choices; less common runner details live here so stale config keys fail fast.
DEFAULT_TRIAL_RUNNER = 'ros2 run mpcc_pp mpcc_pp_gazebo_trial'
DEFAULT_PX4_TARGET = 'px4_sitl'
DEFAULT_ROS_PACKAGE = 'drone_control'
DEFAULT_ROS_LAUNCH_FILE = 'sim.launch.py'
DEFAULT_FORWARD_METRICS = True
DEFAULT_USE_SHELL = False
DEFAULT_CWD = None

LAP_RE = re.compile(
    r"\[Lap\].*?completato in\s+([0-9]+(?:\.[0-9]+)?)\s*s")
SOLVER_STATUS_RE = re.compile(r"\[mpcc_pp OCP\]\s+solver status\s+(-?\d+)")
SOLVER_STATS_RE = re.compile(
    r"\[SolverStats\].*?"
    r"solve_count=(?P<solve_count>\d+).*?"
    r"solver_fail_count=(?P<solver_fail_count>\d+).*?"
    r"solver_fail_rate=(?P<solver_fail_rate>[-+0-9.eE]+)"
)
PROGRESS_RE = re.compile(
    r"(?:total_)?(?:track_)?progress(?:_fraction|_percent|_pct)?\s*=\s*"
    r"(?P<progress>[-+0-9.eE]+)\s*(?P<percent>%?)",
    re.IGNORECASE,
)
LAP_FRACTION_RE = re.compile(
    r"(?:lap_fraction|lap_progress)\s*=\s*"
    r"(?P<lap_fraction>[-+0-9.eE]+)\s*(?P<percent>%?)",
    re.IGNORECASE,
)
COMPLETED_LAPS_RE = re.compile(
    r"completed_laps\s*=\s*(?P<completed_laps>[-+0-9.eE]+)",
    re.IGNORECASE,
)
CRASH_RE = re.compile(
    r"\[(?:Crash|Collision)\]|\b(?:drone\s+)?crashed\b|"
    r"\b(?:crash|crashed|collision)\s*=\s*(?:1|true|yes)\b",
    re.IGNORECASE,
)


@dataclass
class EpisodeMetrics:
    """Metrics returned by one simulation episode."""

    success: bool
    lap_times: list[float]
    solver_fail_rate: float
    solve_count: int = 0
    solver_fail_count: int = 0
    timeout: bool = False
    return_code: int | None = None
    failure_reason: str | None = None
    reward: float | None = None
    log_path: str | None = None
    progress: float = 0.0
    crash: bool = False


def _deep_merge(base: Mapping[str, Any], patch: Mapping[str, Any]) -> dict:
    """Return base recursively updated with patch."""
    merged = copy.deepcopy(dict(base))
    for key, value in dict(patch).items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _load_yaml(path: Path) -> dict:
    """Load a YAML mapping, rejecting scalar/list files early."""
    with path.open('r') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, Mapping):
        raise ValueError(f"YAML file must contain a mapping: {path}")
    return dict(data)


def _load_optional_yaml(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"Optuna config file not found: {path}")
    return _load_yaml(path)


def _write_yaml(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as f:
        yaml.safe_dump(dict(data), f, sort_keys=False)


def write_trial_parameters(base_parameters: Mapping[str, Any],
                           tuning_params: Mapping[str, float],
                           path: Path) -> dict:
    """Write a parameters.yaml variant for one Optuna trial."""
    patched = _deep_merge(base_parameters, tuning_yaml_patch(tuning_params))
    _write_yaml(path, patched)
    return patched


def initial_trial_from_parameters(parameters: Mapping[str, Any]) -> dict[str, float]:
    """Extract the first Optuna trial from a parameters.yaml mapping."""
    path_cfg = parameters.get('path', {})
    contouring_cfg = path_cfg.get('contouring', {})
    cost_cfg = parameters.get('cost', {})

    qc_nom = float(contouring_cfg['qc_nom'])
    qc_gate = float(contouring_cfg['qc_gate'])
    if qc_nom <= 0.0:
        raise ValueError('path.contouring.qc_nom must be positive')

    return {
        'ql': float(cost_cfg['ql']),
        'qc_nom': qc_nom,
        'qc_gate_ratio': qc_gate / qc_nom,
        'Qw': float(cost_cfg['Qw']),
        'r_dvt': float(cost_cfg['r_dvt']),
        'R_df': float(cost_cfg['R_df']),
        'mu': float(cost_cfg['mu']),
        'track_weight': float(cost_cfg['track_weight']),
    }


def _format_command(command_template: str, parameter_file: Path,
                    trial_dir: Path, metrics_file: Path) -> str:
    """Expand the small set of evaluator command placeholders."""
    return command_template.format(
        parameter_file=str(parameter_file),
        trial_dir=str(trial_dir),
        metrics_file=str(metrics_file),
    )


def _terminate_process_group(proc: subprocess.Popen, grace_s: float = 8.0) -> None:
    """Terminate a subprocess tree started with start_new_session=True."""
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=grace_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    proc.wait(timeout=grace_s)


def _clamp01(value: float) -> float:
    return float(min(1.0, max(0.0, value)))


def _normalise_progress(value: Any, *, is_percent: bool = False) -> float:
    progress = float(value)
    if is_percent or progress > 1.0:
        progress /= 100.0
    return _clamp01(progress)


def _lap_progress(lap_times: list[float], laps_required: int,
                  lap_fraction: float = 0.0) -> float:
    completed = min(len(lap_times), laps_required)
    if completed >= laps_required:
        return 1.0
    return _clamp01((completed + _clamp01(lap_fraction)) / laps_required)


def _bool_from_metrics(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'y')
    return bool(value)


def _metrics_progress(data: Mapping[str, Any], lap_times: list[float],
                      laps_required: int) -> float:
    """Read total-track progress from metrics JSON, falling back to laps."""
    for key in ('progress', 'track_progress', 'progress_fraction'):
        if data.get(key) is not None:
            return _normalise_progress(data[key])
    for key in ('progress_percent', 'progress_pct'):
        if data.get(key) is not None:
            return _normalise_progress(data[key], is_percent=True)
    for key in ('lap_fraction', 'lap_progress'):
        if data.get(key) is not None:
            return _lap_progress(lap_times, laps_required,
                                 _normalise_progress(data[key]))
    if data.get('completed_laps') is not None:
        completed_laps = min(float(data['completed_laps']), laps_required)
        return _clamp01(completed_laps / laps_required)
    return _lap_progress(lap_times, laps_required)


def _parse_progress_line(line: str, lap_times: list[float],
                         laps_required: int) -> float | None:
    """Parse either total progress or current-lap progress from stdout."""
    progress_match = PROGRESS_RE.search(line)
    if progress_match:
        return _normalise_progress(
            progress_match.group('progress'),
            is_percent=bool(progress_match.group('percent')),
        )
    lap_match = LAP_FRACTION_RE.search(line)
    if lap_match:
        lap_fraction = _normalise_progress(
            lap_match.group('lap_fraction'),
            is_percent=bool(lap_match.group('percent')),
        )
        return _lap_progress(lap_times, laps_required, lap_fraction)
    completed_match = COMPLETED_LAPS_RE.search(line)
    if completed_match:
        completed_laps = min(
            float(completed_match.group('completed_laps')),
            laps_required,
        )
        return _clamp01(completed_laps / laps_required)
    return None


def _load_metrics_file(path: Path, laps_required: int) -> EpisodeMetrics | None:
    """Load metrics written by an external evaluator, if present."""
    if not path.exists():
        return None
    with path.open('r') as f:
        data = json.load(f)
    lap_times = [float(v) for v in data.get('lap_times', [])]
    crash = any(
        _bool_from_metrics(data.get(key))
        for key in ('crash', 'crashed', 'collision')
        if data.get(key) is not None
    )
    return EpisodeMetrics(
        success=bool(data.get('success', len(lap_times) >= laps_required))
        and not crash,
        lap_times=lap_times,
        solver_fail_rate=float(data.get('solver_fail_rate', 0.0)),
        solve_count=int(data.get('solve_count', 0)),
        solver_fail_count=int(data.get('solver_fail_count', 0)),
        timeout=bool(data.get('timeout', False)),
        return_code=data.get('return_code'),
        failure_reason=data.get('failure_reason'),
        reward=(None if data.get('reward') is None
                else float(data.get('reward'))),
        log_path=data.get('log_path'),
        progress=_metrics_progress(data, lap_times, laps_required),
        crash=crash,
    )


def run_episode(command_template: str,
                parameter_file: Path,
                trial_dir: Path,
                metrics_file: Path,
                laps_required: int,
                timeout_s: float,
                stream_logs: bool,
                use_shell: bool,
                cwd: Path | None) -> EpisodeMetrics:
    """
    Run one simulator episode and collect lap/solver metrics from stdout.

    The evaluated command also receives MPCC_PP_PARAMETERS_FILE and
    MPCC_PP_METRICS_FILE in the environment.  If it writes the metrics JSON, the
    JSON takes precedence over stdout parsing.
    """
    command_text = _format_command(
        command_template, parameter_file, trial_dir, metrics_file)
    env = os.environ.copy()
    env['MPCC_PP_PARAMETERS_FILE'] = str(parameter_file)
    env['MPCC_PP_TRIAL_DIR'] = str(trial_dir)
    env['MPCC_PP_METRICS_FILE'] = str(metrics_file)

    popen_args: str | list[str]
    popen_args = command_text if use_shell else shlex.split(command_text)
    log_path = trial_dir / 'episode.log'
    lap_times: list[float] = []
    solver_status_fail_count = 0
    latest_solver_stats: dict[str, float | int] = {}
    progress = 0.0
    crash = False
    timed_out = False
    completed_at: float | None = None

    with log_path.open('w') as log_file:
        log_file.write(f"$ {command_text}\n")
        log_file.flush()
        proc = subprocess.Popen(
            popen_args,
            cwd=None if cwd is None else str(cwd),
            env=env,
            shell=use_shell,
            executable='/bin/bash' if use_shell else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors='replace',
            bufsize=1,
            start_new_session=True,
        )

        selector = selectors.DefaultSelector()
        assert proc.stdout is not None
        selector.register(proc.stdout, selectors.EVENT_READ)
        start = time.monotonic()

        try:
            while True:
                for key, _ in selector.select(timeout=0.2):
                    line = key.fileobj.readline()
                    if not line:
                        continue
                    log_file.write(line)
                    log_file.flush()
                    if stream_logs:
                        print(line, end='')

                    lap_match = LAP_RE.search(line)
                    if lap_match:
                        lap_times.append(float(lap_match.group(1)))
                        progress = max(
                            progress,
                            _lap_progress(lap_times, laps_required),
                        )
                        if (len(lap_times) >= laps_required
                                and completed_at is None):
                            completed_at = time.monotonic()

                    parsed_progress = _parse_progress_line(
                        line, lap_times, laps_required)
                    if parsed_progress is not None:
                        progress = max(progress, parsed_progress)

                    if CRASH_RE.search(line):
                        crash = True

                    status_match = SOLVER_STATUS_RE.search(line)
                    if status_match:
                        solver_status_fail_count += 1

                    stats_match = SOLVER_STATS_RE.search(line)
                    if stats_match:
                        latest_solver_stats = {
                            'solve_count': int(
                                stats_match.group('solve_count')),
                            'solver_fail_count': int(
                                stats_match.group('solver_fail_count')),
                            'solver_fail_rate': float(
                                stats_match.group('solver_fail_rate')),
                        }

                if crash:
                    _terminate_process_group(proc)
                    break

                if (completed_at is not None
                        and time.monotonic() - completed_at > 2.0):
                    # Give the controller a short window to flush final metrics
                    # after the last required lap, then stop the simulator tree.
                    _terminate_process_group(proc)
                    break

                if proc.poll() is not None:
                    break

                if time.monotonic() - start > timeout_s:
                    timed_out = True
                    _terminate_process_group(proc)
                    break
        except KeyboardInterrupt:
            print(
                '[mpcc_pp_optuna_tune] interrupted; terminating active trial',
                flush=True,
            )
            _terminate_process_group(proc, grace_s=2.0)
            raise
        finally:
            try:
                selector.unregister(proc.stdout)
            except Exception:
                pass
            if proc.poll() is None:
                _terminate_process_group(proc, grace_s=2.0)

        return_code = proc.poll()
        for line in proc.stdout:
            log_file.write(line)
            if stream_logs:
                print(line, end='')
            parsed_progress = _parse_progress_line(
                line, lap_times, laps_required)
            if parsed_progress is not None:
                progress = max(progress, parsed_progress)
            if CRASH_RE.search(line):
                crash = True

    external_metrics = _load_metrics_file(metrics_file, laps_required)
    if external_metrics is not None:
        external_metrics.log_path = str(log_path)
        external_metrics.progress = max(external_metrics.progress, progress)
        external_metrics.crash = external_metrics.crash or crash
        if external_metrics.crash:
            external_metrics.success = False
            external_metrics.failure_reason = (
                external_metrics.failure_reason or 'crash')
        return external_metrics

    solve_count = int(latest_solver_stats.get('solve_count', 0))
    solver_fail_count = int(latest_solver_stats.get(
        'solver_fail_count', solver_status_fail_count))
    if 'solver_fail_rate' in latest_solver_stats:
        solver_fail_rate = float(latest_solver_stats['solver_fail_rate'])
    elif solver_status_fail_count:
        solver_fail_rate = 1.0
    else:
        solver_fail_rate = 0.0

    progress = max(progress, _lap_progress(lap_times, laps_required))
    success = len(lap_times) >= laps_required and not timed_out and not crash
    failure_reason = None
    if crash:
        failure_reason = 'crash'
    elif timed_out:
        failure_reason = 'timeout'
    elif return_code not in (0, None) and len(lap_times) < laps_required:
        failure_reason = f'command_failed:{return_code}'
    elif len(lap_times) < laps_required:
        failure_reason = 'not_enough_laps'

    return EpisodeMetrics(
        success=success,
        lap_times=lap_times,
        solver_fail_rate=solver_fail_rate,
        solve_count=solve_count,
        solver_fail_count=solver_fail_count,
        timeout=timed_out,
        return_code=return_code,
        failure_reason=failure_reason,
        log_path=str(log_path),
        progress=progress,
        crash=crash,
    )


def compute_reward(metrics: EpisodeMetrics, laps_required: int,
                   timeout_s: float, gamma: float,
                   max_solver_fail_rate: float) -> float:
    """Compute the scalar reward maximized by Optuna.

    Higher is better. Completed non-crash episodes score from total lap time.
    Incomplete/crashed episodes score from total-track progress, so Optuna can
    distinguish an immediate crash from one that nearly finished the race.
    """
    if metrics.reward is not None:
        return float(metrics.reward)

    if metrics.solver_fail_rate > max_solver_fail_rate:
        metrics.success = False
        metrics.failure_reason = (
            f"solver_fail_rate>{max_solver_fail_rate:.3f}")

    metrics.progress = max(
        metrics.progress,
        _lap_progress(metrics.lap_times, laps_required),
    )
    complete = (
        metrics.success
        and not metrics.crash
        and len(metrics.lap_times) >= laps_required
    )
    if complete:
        base_reward = 1000.0 - sum(metrics.lap_times[:laps_required])
    else:
        base_reward = 1000.0 * metrics.progress - timeout_s
        if metrics.crash:
            metrics.success = False
            metrics.failure_reason = metrics.failure_reason or 'crash'

    return float(base_reward - gamma * metrics.solver_fail_rate)


def aggregate_rewards(rewards: list[float], method: str) -> float:
    """Aggregate repeated fresh-episode rewards for one Optuna trial."""
    if not rewards:
        raise ValueError('Cannot aggregate an empty reward list')
    if method == 'mean':
        return float(sum(rewards) / len(rewards))
    if method == 'median':
        return float(statistics.median(rewards))
    if method == 'min':
        return float(min(rewards))
    raise ValueError(f'Unsupported repeat aggregation method: {method}')


def _create_sampler(optuna_module, name: str, seed: int | None,
                    n_startup_trials: int | None,
                    n_trust_region: int | None):
    """Create the requested Optuna sampler, loading optional samplers lazily."""
    if name == 'random':
        return optuna_module.samplers.RandomSampler(seed=seed)
    if name == 'turbo':
        try:
            import optunahub
        except ImportError as exc:
            raise SystemExit(
                'The Turbo sampler requires OptunaHub. Install it with: '
                'pip install optunahub torch scipy') from exc
        try:
            turbo_module = optunahub.load_module(package='samplers/turbo')
            turbo_kwargs: dict[str, Any] = {'seed': seed}
            if n_startup_trials is not None:
                turbo_kwargs['n_startup_trials'] = n_startup_trials
            if n_trust_region is not None:
                turbo_kwargs['n_trust_region'] = n_trust_region
            return turbo_module.TuRBOSampler(**turbo_kwargs)
        except ImportError as exc:
            raise SystemExit(
                'The Turbo sampler requires its optional dependencies. '
                'Install them with: pip install optunahub torch scipy') from exc
    if name == 'gp':
        gp_sampler = getattr(optuna_module.samplers, 'GPSampler', None)
        if gp_sampler is not None:
            return gp_sampler(seed=seed)
        print('Optuna GPSampler is unavailable; falling back to TPESampler.',
              file=sys.stderr)
    return optuna_module.samplers.TPESampler(
        seed=seed,
        multivariate=True,
        group=True,
    )


def _default_storage(output_dir: Path) -> str:
    """Use one SQLite study database inside the run output directory."""
    return f"sqlite:///{(output_dir / 'study.db').resolve()}"


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as f:
        json.dump(data, f, indent=2, sort_keys=True)


def _cleanup_logs(
    directory: Path,
    metrics: EpisodeMetrics,
    log_retention: str,
) -> int:
    """Remove bulky per-episode logs according to the retention policy."""
    if log_retention == 'all':
        return 0
    if log_retention == 'failed' and not metrics.success:
        return 0

    removed_count = 0
    removed_metrics_log = False
    metrics_log_path = (
        Path(metrics.log_path).resolve()
        if metrics.log_path is not None
        else None
    )
    for log_path in directory.rglob('*.log'):
        try:
            if (metrics_log_path is not None
                    and log_path.resolve() == metrics_log_path):
                removed_metrics_log = True
            log_path.unlink()
            removed_count += 1
        except FileNotFoundError:
            pass
    if removed_metrics_log:
        metrics.log_path = None
    return removed_count


def _normalise_config_keys(config: Mapping[str, Any]) -> dict[str, Any]:
    """Accept either hyphenated CLI names or argparse's underscore names."""
    return {
        str(key).replace('-', '_'): value
        for key, value in dict(config).items()
    }


def _coerce_config_defaults(config: Mapping[str, Any],
                            parser: argparse.ArgumentParser) -> dict[str, Any]:
    """Validate and coerce YAML defaults so argparse can consume them."""
    config = _normalise_config_keys(config)
    actions = {
        action.dest: action
        for action in parser._actions
        if action.dest not in ('help', 'config', argparse.SUPPRESS)
    }
    unknown = sorted(set(config) - set(actions))
    if unknown:
        parser.error(
            'Unknown key(s) in --config file: ' + ', '.join(unknown))

    defaults: dict[str, Any] = {}
    for key, value in config.items():
        action = actions[key]
        if value is None:
            defaults[key] = None
        elif action.type is Path:
            defaults[key] = Path(value).expanduser()
        elif action.type is not None:
            defaults[key] = action.type(value)
        else:
            defaults[key] = value

        if action.choices is not None and defaults[key] is not None:
            if defaults[key] not in action.choices:
                parser.error(
                    f"Invalid value for {key!r} in --config file: "
                    f"{defaults[key]!r}. Choose from {list(action.choices)}")
    return defaults


def _bool_text(value: bool) -> str:
    return 'true' if bool(value) else 'false'


def _quote_command(parts: list[str]) -> str:
    return ' '.join(shlex.quote(str(part)) for part in parts)


def build_evaluation_command(args: argparse.Namespace) -> str:
    """Build the hard-coded gazebo-trial command with tunable essentials."""
    # Keep this builder deliberately narrow: these fields are expected to vary
    # between tuning runs, while the rest follows mpcc_pp_gazebo_trial defaults.
    ros_parts = [
        'ros2',
        'launch',
        DEFAULT_ROS_PACKAGE,
        DEFAULT_ROS_LAUNCH_FILE,
        f'controller_type:={args.controller_type}',
        f'rviz:={_bool_text(args.rviz)}',
        'plotjuggler:=false',
    ]
    ros_command = _quote_command(ros_parts)

    parts = shlex.split(DEFAULT_TRIAL_RUNNER)
    parts.extend(['--px4-target', DEFAULT_PX4_TARGET])
    if args.px4_model:
        parts.extend(['--px4-model', str(args.px4_model)])
    parts.extend(['--startup-timeout-s', str(args.startup_timeout_s)])
    parts.extend(['--startup-delay-s', str(args.startup_delay_s)])
    if args.sim_speed_factor is not None:
        parts.extend(['--sim-speed-factor', str(args.sim_speed_factor)])
    if not DEFAULT_FORWARD_METRICS:
        parts.append('--no-forward-metrics')
    parts.extend(['--ros-command', ros_command])
    return _quote_command(parts)


def _export_best(study, search_space: SearchSpace,
                 base_parameters: Mapping[str, Any],
                 output_dir: Path) -> None:
    """Write the current best parameter YAML and a compact JSON summary."""
    if study.best_trial is None:
        return
    best_params = search_space.expand(study.best_trial.params)
    best_yaml = output_dir / 'best_parameters.yaml'
    write_trial_parameters(base_parameters, best_params, best_yaml)
    _write_json(output_dir / 'best_summary.json', {
        'best_value': study.best_value,
        'best_trial_number': study.best_trial.number,
        'best_params': best_params,
        'best_user_attrs': study.best_trial.user_attrs,
        'best_parameters_file': str(best_yaml),
    })


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Tune MPCC++ parameters with Optuna.')
    parser.add_argument('--config', type=Path,
                        default=None,
                        help=('YAML file containing Optuna tuner options. '
                              f'Example: {DEFAULT_CONFIG_PATH}'))
    parser.add_argument('--base-parameters', type=Path,
                        default=DEFAULT_PARAMETERS_PATH,
                        help='Base MPCC++ parameters.yaml file.')
    parser.add_argument('--output-dir', type=Path,
                        default=DEFAULT_OUTPUT_DIR,
                        help='Directory for studies, trial YAMLs, and logs.')
    parser.add_argument('--study-name', default='mpcc_pp_optuna',
                        help='Optuna study name.')
    parser.add_argument('--n-trials', type=int, default=30,
                        help='Number of Optuna trials.')
    parser.add_argument('--sampler', choices=('tpe', 'gp', 'random', 'turbo'),
                        default='tpe',
                        help=('Optuna sampler. TPE is the robust default; '
                              'turbo uses OptunaHub TuRBOSampler.'))
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for the sampler.')
    parser.add_argument('--n-startup-trials', type=int, default=None,
                        help=('TuRBO only: initial trials per trust region. '
                              'null/omitted uses the OptunaHub default.'))
    parser.add_argument('--n-trust-region', type=int, default=None,
                        help=('TuRBO only: number of trust regions. '
                              'null/omitted uses the OptunaHub default.'))
    parser.add_argument('--px4-model', default='gz_zephyros',
                        help='PX4/Gazebo model for mpcc_pp_gazebo_trial.')
    parser.add_argument('--startup-timeout-s', type=float, default=60.0,
                        help='Seconds for mpcc_pp_gazebo_trial to wait for PX4 readiness.')
    parser.add_argument('--startup-delay-s', type=float, default=1.0,
                        help='Extra delay before starting ROS after PX4 readiness.')
    parser.add_argument('--sim-speed-factor', type=float, default=None,
                        help='Optional PX4_SIM_SPEED_FACTOR for the trial runner.')
    parser.add_argument('--controller-type', default='mpcc_pp',
                        help='controller_type launch argument.')
    parser.add_argument('--rviz', action=argparse.BooleanOptionalAction,
                        default=False,
                        help='rviz launch argument.')
    parser.add_argument('--laps-per-episode', type=int, default=3,
                        help='Number of completed laps required per episode.')
    parser.add_argument('--repeats-per-trial', type=int, default=3,
                        help=('Fresh simulator episodes to run for each '
                              'Optuna trial. Each repeat uses the same '
                              'candidate parameters.'))
    parser.add_argument('--repeat-aggregation',
                        choices=('mean', 'median', 'min'),
                        default='mean',
                        help=('How to aggregate repeated episode rewards for '
                              'one Optuna trial. Mean is the robust-tuning '
                              'default because failed repeats keep their '
                              'large negative reward.'))
    parser.add_argument('--timeout-s', type=float, default=90.0,
                        help='Maximum wall-clock seconds per episode repeat.')
    parser.add_argument('--gamma', type=float, default=100.0,
                        help='Solver-failure penalty in the reward.')
    parser.add_argument('--max-solver-fail-rate', type=float, default=0.30,
                        help='Hard failure threshold for solver failures.')
    parser.add_argument('--stream-logs', action=argparse.BooleanOptionalAction,
                        default=False,
                        help='Mirror simulator output to this terminal.')
    parser.add_argument('--log-retention', choices=('all', 'failed', 'none'),
                        default='all',
                        help=('Which per-episode *.log files to keep. "failed" '
                              'keeps logs only for failed repeats. "none" keeps '
                              'only metrics/parameter files.'))
    parser.add_argument('--dry-run', action=argparse.BooleanOptionalAction,
                        default=False,
                        help='Write a center-point trial YAML and exit.')
    return parser


def main(argv: list[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    parser = build_arg_parser()
    config_parser = argparse.ArgumentParser(add_help=False)
    config_parser.add_argument('--config', type=Path, default=None)
    config_args, _ = config_parser.parse_known_args(raw_argv)
    if config_args.config is not None:
        # Config values become argparse defaults, so explicit CLI flags still
        # win. Unknown keys are rejected by _coerce_config_defaults.
        config = _load_optional_yaml(config_args.config.expanduser())
        parser.set_defaults(
            **_coerce_config_defaults(config, parser),
            config=config_args.config.expanduser(),
        )

    args = parser.parse_args(raw_argv)
    args.output_dir = args.output_dir.expanduser()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if args.repeats_per_trial < 1:
        parser.error('--repeats-per-trial must be >= 1')
    if args.n_startup_trials is not None and args.n_startup_trials < 1:
        parser.error('--n-startup-trials must be >= 1')
    if args.n_trust_region is not None and args.n_trust_region < 1:
        parser.error('--n-trust-region must be >= 1')

    base_parameters = _load_yaml(args.base_parameters)
    search_space = SearchSpace()

    if args.dry_run:
        trial_dir = args.output_dir / 'dry_run'
        trial_dir.mkdir(parents=True, exist_ok=True)
        params = search_space.decode([0.5] * search_space.dim)
        parameter_file = trial_dir / 'parameters.yaml'
        write_trial_parameters(base_parameters, params, parameter_file)
        _write_json(trial_dir / 'trial_params.json', params)
        print(f"Wrote dry-run parameters to {parameter_file}")
        return 0

    evaluation_command = build_evaluation_command(args)

    try:
        import optuna
    except ImportError as exc:
        raise SystemExit(
            'Optuna is not installed. Install the mpcc_pp package with its '
            'Python dependencies, or run: pip install optuna') from exc

    sampler = _create_sampler(
        optuna,
        args.sampler,
        args.seed,
        args.n_startup_trials,
        args.n_trust_region,
    )
    storage = _default_storage(args.output_dir)
    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage,
        direction='maximize',
        sampler=sampler,
        load_if_exists=True,
    )
    if not study.trials:
        # Evaluate the checked-in/base parameters first for an interpretable
        # baseline and to make trial_00000 easy to compare against later runs.
        initial_params = initial_trial_from_parameters(base_parameters)
        study.enqueue_trial(initial_params)
        print(
            'Enqueued initial trial from '
            f'{args.base_parameters}: {initial_params}',
            flush=True,
        )
    else:
        print(
            f"Loaded existing study '{args.study_name}' with "
            f"{len(study.trials)} trial(s); the base parameters will not be "
            "inserted as trial_00000. Use a new --output-dir, a new "
            "--study-name, or archive the old study.db for a fresh study.",
            flush=True,
        )

    def objective(trial) -> float:
        """Evaluate one Optuna parameter vector across fresh episode repeats."""
        trial_dir = args.output_dir / f"trial_{trial.number:05d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        parameter_file = trial_dir / 'parameters.yaml'

        tuning_params = search_space.suggest(trial)
        write_trial_parameters(base_parameters, tuning_params, parameter_file)
        _write_json(trial_dir / 'trial_params.json', tuning_params)

        repeat_records: list[dict[str, Any]] = []
        repeat_rewards: list[float] = []
        for repeat_idx in range(args.repeats_per_trial):
            repeat_dir = (
                trial_dir
                if args.repeats_per_trial == 1
                else trial_dir / f"repeat_{repeat_idx:02d}"
            )
            repeat_dir.mkdir(parents=True, exist_ok=True)
            metrics_file = repeat_dir / 'metrics.json'

            metrics = run_episode(
                command_template=evaluation_command,
                parameter_file=parameter_file,
                trial_dir=repeat_dir,
                metrics_file=metrics_file,
                laps_required=args.laps_per_episode,
                timeout_s=args.timeout_s,
                stream_logs=args.stream_logs,
                use_shell=DEFAULT_USE_SHELL,
                cwd=DEFAULT_CWD,
            )
            reward = compute_reward(
                metrics=metrics,
                laps_required=args.laps_per_episode,
                timeout_s=args.timeout_s,
                gamma=args.gamma,
                max_solver_fail_rate=args.max_solver_fail_rate,
            )
            metrics.reward = reward
            repeat_metrics_path = repeat_dir / 'episode_metrics.json'
            removed_logs = _cleanup_logs(
                repeat_dir,
                metrics,
                args.log_retention,
            )
            metrics_dict = asdict(metrics)
            metrics_dict['removed_log_files'] = removed_logs
            metrics_dict['log_retention'] = args.log_retention
            _write_json(repeat_metrics_path, metrics_dict)
            repeat_records.append({
                'repeat': repeat_idx,
                'metrics_file': str(repeat_metrics_path),
                **metrics_dict,
            })
            repeat_rewards.append(reward)
            print(
                f"[trial {trial.number:05d} repeat "
                f"{repeat_idx + 1}/{args.repeats_per_trial}] "
                f"reward={reward:.6f} success={metrics.success} "
                f"laps={len(metrics.lap_times)}/{args.laps_per_episode} "
                f"progress={metrics.progress:.3f} "
                f"crash={metrics.crash} "
                f"solver_fail_rate={metrics.solver_fail_rate:.3f} "
                f"return_code={metrics.return_code} "
                f"reason={metrics.failure_reason or 'completed'} "
                f"log={metrics.log_path or 'discarded'}",
                flush=True,
            )

        reward = aggregate_rewards(repeat_rewards, args.repeat_aggregation)
        metrics_path = trial_dir / 'trial_metrics.json'
        success_count = sum(1 for record in repeat_records if record['success'])
        mean_solver_fail_rate = sum(
            float(record['solver_fail_rate'])
            for record in repeat_records
        ) / len(repeat_records)
        mean_progress = sum(
            float(record['progress'])
            for record in repeat_records
        ) / len(repeat_records)
        crash_count = sum(1 for record in repeat_records if record['crash'])
        summary = {
            'reward': reward,
            'repeat_count': args.repeats_per_trial,
            'repeat_aggregation': args.repeat_aggregation,
            'repeat_rewards': repeat_rewards,
            'mean_repeat_reward': aggregate_rewards(repeat_rewards, 'mean'),
            'median_repeat_reward': aggregate_rewards(repeat_rewards, 'median'),
            'min_repeat_reward': aggregate_rewards(repeat_rewards, 'min'),
            'success': success_count == len(repeat_records),
            'successful_repeats': success_count,
            'crashed_repeats': crash_count,
            'laps_per_episode': args.laps_per_episode,
            'mean_progress': mean_progress,
            'mean_solver_fail_rate': mean_solver_fail_rate,
            'repeats': repeat_records,
        }
        _write_json(metrics_path, summary)
        print(
            f"[trial {trial.number:05d}] reward={reward:.6f} "
            f"aggregation={args.repeat_aggregation} "
            f"successful_repeats={success_count}/{len(repeat_records)} "
            f"crashed_repeats={crash_count}/{len(repeat_records)} "
            f"mean_progress={mean_progress:.3f} "
            f"mean_solver_fail_rate={mean_solver_fail_rate:.3f} "
            f"metrics={metrics_path}",
            flush=True,
        )

        trial.set_user_attr('metrics_file', str(metrics_path))
        trial.set_user_attr('parameter_file', str(parameter_file))
        trial.set_user_attr('repeat_count', args.repeats_per_trial)
        trial.set_user_attr('repeat_aggregation', args.repeat_aggregation)
        trial.set_user_attr('repeat_rewards', repeat_rewards)
        trial.set_user_attr('successful_repeats', success_count)
        trial.set_user_attr('crashed_repeats', crash_count)
        trial.set_user_attr('mean_progress', mean_progress)
        trial.set_user_attr('mean_solver_fail_rate', mean_solver_fail_rate)
        trial.set_user_attr('success', summary['success'])
        return reward

    try:
        study.optimize(
            objective,
            n_trials=args.n_trials,
            gc_after_trial=True,
        )
    finally:
        try:
            _export_best(study, search_space, base_parameters, args.output_dir)
        except ValueError:
            pass

    print(f"Best reward: {study.best_value:.6f}")
    print(f"Best trial: {study.best_trial.number}")
    print(f"Best parameters written to {args.output_dir / 'best_parameters.yaml'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
