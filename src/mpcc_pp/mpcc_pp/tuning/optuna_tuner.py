#!/usr/bin/env python
"""Optuna runner for MPCC++ parameter tuning."""

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
import subprocess
import sys
import time
from typing import Any, Mapping

import yaml

from ..parameters import DEFAULT_PARAMETERS_PATH
from .search_space import SearchSpace, tuning_yaml_patch


LAP_RE = re.compile(
    r"\[Lap\].*?completato in\s+([0-9]+(?:\.[0-9]+)?)\s*s")
SOLVER_STATUS_RE = re.compile(r"\[mpcc_pp OCP\]\s+solver status\s+(-?\d+)")
SOLVER_STATS_RE = re.compile(
    r"\[SolverStats\].*?"
    r"solve_count=(?P<solve_count>\d+).*?"
    r"solver_fail_count=(?P<solver_fail_count>\d+).*?"
    r"solver_fail_rate=(?P<solver_fail_rate>[-+0-9.eE]+)"
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
    with path.open('r') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, Mapping):
        raise ValueError(f"YAML file must contain a mapping: {path}")
    return dict(data)


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


def _format_command(command_template: str, parameter_file: Path,
                    trial_dir: Path, metrics_file: Path) -> str:
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


def _load_metrics_file(path: Path) -> EpisodeMetrics | None:
    """Load metrics written by an external evaluator, if present."""
    if not path.exists():
        return None
    with path.open('r') as f:
        data = json.load(f)
    lap_times = [float(v) for v in data.get('lap_times', [])]
    return EpisodeMetrics(
        success=bool(data.get('success', bool(lap_times))),
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
                    if len(lap_times) >= laps_required and completed_at is None:
                        completed_at = time.monotonic()

                status_match = SOLVER_STATUS_RE.search(line)
                if status_match:
                    solver_status_fail_count += 1

                stats_match = SOLVER_STATS_RE.search(line)
                if stats_match:
                    latest_solver_stats = {
                        'solve_count': int(stats_match.group('solve_count')),
                        'solver_fail_count': int(
                            stats_match.group('solver_fail_count')),
                        'solver_fail_rate': float(
                            stats_match.group('solver_fail_rate')),
                    }

            if (completed_at is not None
                    and time.monotonic() - completed_at > 2.0):
                _terminate_process_group(proc)
                break

            if proc.poll() is not None:
                break

            if time.monotonic() - start > timeout_s:
                timed_out = True
                _terminate_process_group(proc)
                break

        return_code = proc.poll()
        selector.unregister(proc.stdout)
        for line in proc.stdout:
            log_file.write(line)
            if stream_logs:
                print(line, end='')

    external_metrics = _load_metrics_file(metrics_file)
    if external_metrics is not None:
        external_metrics.log_path = str(log_path)
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

    success = len(lap_times) >= laps_required and not timed_out
    failure_reason = None
    if timed_out:
        failure_reason = 'timeout'
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
    )


def compute_reward(metrics: EpisodeMetrics, laps_required: int,
                   gamma: float, failed_reward: float,
                   max_solver_fail_rate: float) -> float:
    """Compute the scalar reward maximized by Optuna."""
    if metrics.reward is not None:
        return float(metrics.reward)

    if metrics.solver_fail_rate > max_solver_fail_rate:
        metrics.success = False
        metrics.failure_reason = (
            f"solver_fail_rate>{max_solver_fail_rate:.3f}")

    if not metrics.success or len(metrics.lap_times) < laps_required:
        return float(failed_reward)

    lap_times = metrics.lap_times[:laps_required]
    mean_lap_time = sum(lap_times) / len(lap_times)
    return float(-mean_lap_time - gamma * metrics.solver_fail_rate)


def _create_sampler(optuna_module, name: str, seed: int | None):
    if name == 'random':
        return optuna_module.samplers.RandomSampler(seed=seed)
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
    return f"sqlite:///{(output_dir / 'study.db').resolve()}"


def _write_json(path: Path, data: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w') as f:
        json.dump(data, f, indent=2, sort_keys=True)


def _export_best(study, search_space: SearchSpace,
                 base_parameters: Mapping[str, Any],
                 output_dir: Path) -> None:
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
    parser.add_argument('--base-parameters', type=Path,
                        default=DEFAULT_PARAMETERS_PATH,
                        help='Base MPCC++ parameters.yaml file.')
    parser.add_argument('--output-dir', type=Path,
                        default=Path('mpcc_pp_optuna_runs'),
                        help='Directory for studies, trial YAMLs, and logs.')
    parser.add_argument('--study-name', default='mpcc_pp_optuna',
                        help='Optuna study name.')
    parser.add_argument('--storage', default=None,
                        help='Optuna storage URI. Defaults to SQLite in output-dir.')
    parser.add_argument('--n-trials', type=int, default=30,
                        help='Number of Optuna trials.')
    parser.add_argument('--sampler', choices=('tpe', 'gp', 'random'),
                        default='tpe',
                        help='Optuna sampler. TPE is the robust default.')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed for the sampler.')
    parser.add_argument('--command', default=None,
                        help=('Evaluation command. It receives '
                              'MPCC_PP_PARAMETERS_FILE and can use '
                              '{parameter_file}, {trial_dir}, {metrics_file}.'))
    parser.add_argument('--shell', action='store_true',
                        help='Run the command through /bin/bash.')
    parser.add_argument('--cwd', type=Path, default=None,
                        help='Working directory for the evaluation command.')
    parser.add_argument('--laps-per-episode', type=int, default=3,
                        help='Number of completed laps required per trial.')
    parser.add_argument('--timeout-s', type=float, default=90.0,
                        help='Maximum wall-clock seconds per trial.')
    parser.add_argument('--gamma', type=float, default=100.0,
                        help='Solver-failure penalty in the reward.')
    parser.add_argument('--failed-reward', type=float, default=-1e6,
                        help='Reward assigned to failed episodes.')
    parser.add_argument('--max-solver-fail-rate', type=float, default=0.30,
                        help='Hard failure threshold for solver failures.')
    parser.add_argument('--stream-logs', action='store_true',
                        help='Mirror simulator output to this terminal.')
    parser.add_argument('--dry-run', action='store_true',
                        help='Write a center-point trial YAML and exit.')
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)

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

    if not args.command:
        parser.error('--command is required unless --dry-run is used')

    try:
        import optuna
    except ImportError as exc:
        raise SystemExit(
            'Optuna is not installed. Install the mpcc_pp package with its '
            'Python dependencies, or run: pip install optuna') from exc

    sampler = _create_sampler(optuna, args.sampler, args.seed)
    storage = args.storage or _default_storage(args.output_dir)
    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage,
        direction='maximize',
        sampler=sampler,
        load_if_exists=True,
    )

    def objective(trial) -> float:
        trial_dir = args.output_dir / f"trial_{trial.number:05d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        parameter_file = trial_dir / 'parameters.yaml'
        metrics_file = trial_dir / 'metrics.json'

        tuning_params = search_space.suggest(trial)
        write_trial_parameters(base_parameters, tuning_params, parameter_file)
        _write_json(trial_dir / 'trial_params.json', tuning_params)

        metrics = run_episode(
            command_template=args.command,
            parameter_file=parameter_file,
            trial_dir=trial_dir,
            metrics_file=metrics_file,
            laps_required=args.laps_per_episode,
            timeout_s=args.timeout_s,
            stream_logs=args.stream_logs,
            use_shell=args.shell,
            cwd=args.cwd,
        )
        reward = compute_reward(
            metrics=metrics,
            laps_required=args.laps_per_episode,
            gamma=args.gamma,
            failed_reward=args.failed_reward,
            max_solver_fail_rate=args.max_solver_fail_rate,
        )
        metrics.reward = reward
        metrics_path = trial_dir / 'trial_metrics.json'
        _write_json(metrics_path, asdict(metrics))

        trial.set_user_attr('metrics_file', str(metrics_path))
        trial.set_user_attr('parameter_file', str(parameter_file))
        trial.set_user_attr('lap_times', metrics.lap_times)
        trial.set_user_attr('solver_fail_rate', metrics.solver_fail_rate)
        trial.set_user_attr('success', metrics.success)
        trial.set_user_attr('failure_reason', metrics.failure_reason)
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
