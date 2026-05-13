#!/usr/bin/env python
"""Run one Optuna trial with managed PX4 SITL, Gazebo, and ROS launch."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import sys
import threading
import time


DEFAULT_READY_PATTERNS = (
    r'Ready for takeoff',
    r'Startup script returned successfully',
    r'INFO\s+\[commander\].*ready',
)

DEFAULT_METRIC_PATTERNS = (
    r'\[Lap\]',
    r'\[Progress\]',
    r'\[Crash\]',
    r'\[Collision\]',
    r'\[MissedGate\]',
    r'\[SolverStats\]',
)


class ManagedProcess:
    """Small wrapper around a subprocess plus its streaming thread."""

    def __init__(self, name: str, args, cwd: Path | None,
                 shell: bool, log_path: Path, stream_logs: bool = False,
                 ready_patterns: tuple[re.Pattern[str], ...] = (),
                 forward_patterns: tuple[re.Pattern[str], ...] = ()):
        self.name = name
        self.args = args
        self.cwd = cwd
        self.shell = shell
        self.log_path = log_path
        self.stream_logs = stream_logs
        self.ready_patterns = ready_patterns
        self.forward_patterns = forward_patterns
        self.ready = threading.Event()
        self.lines: queue.Queue[str] = queue.Queue()
        self.proc: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None

    def start(self, env: dict[str, str]) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = self.log_path.open('w')
        log_file.write(f"$ {self.args}\n")
        log_file.flush()

        self.proc = subprocess.Popen(
            self.args,
            cwd=None if self.cwd is None else str(self.cwd),
            env=env,
            shell=self.shell,
            executable='/bin/bash' if self.shell else None,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors='replace',
            bufsize=1,
            start_new_session=True,
        )

        def _pump() -> None:
            assert self.proc is not None
            assert self.proc.stdout is not None
            with log_file:
                for line in self.proc.stdout:
                    prefixed = f"[{self.name}] {line}"
                    log_file.write(prefixed)
                    log_file.flush()
                    should_forward = any(
                        pattern.search(line)
                        for pattern in self.forward_patterns
                    )
                    if self.stream_logs or should_forward:
                        print(prefixed, end='', flush=True)
                    self.lines.put(line)
                    if any(pattern.search(line) for pattern in self.ready_patterns):
                        self.ready.set()

        self._thread = threading.Thread(
            target=_pump,
            name=f'{self.name}-stdout',
            daemon=True,
        )
        self._thread.start()

    def poll(self) -> int | None:
        if self.proc is None:
            return None
        return self.proc.poll()

    def terminate(self, grace_s: float = 8.0) -> None:
        if self.proc is None or self.proc.poll() is not None:
            return
        try:
            os.killpg(self.proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            self.proc.wait(timeout=grace_s)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        self.proc.wait(timeout=grace_s)


def _default_px4_dir() -> Path:
    this_file = Path(__file__).resolve()
    # Running from source: src/mpcc_pp/mpcc_pp/tuning/gazebo_trial_runner.py
    #   parents[4] = workspace root
    # Running installed: install/mpcc_pp/lib/python3.10/site-packages/mpcc_pp/tuning/...
    #   parents[7] = workspace root
    for depth in (4, 7):
        candidate = this_file.parents[depth] / 'px4' / 'PX4-Autopilot'
        if candidate.exists():
            return candidate
    return this_file.parents[4] / 'px4' / 'PX4-Autopilot'


def _compiled_patterns(patterns: list[str]) -> tuple[re.Pattern[str], ...]:
    return tuple(re.compile(pattern, re.IGNORECASE) for pattern in patterns)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Run one MPCC++ tuning episode with managed PX4/Gazebo.')
    parser.add_argument('--px4-dir', type=Path, default=_default_px4_dir(),
                        help='PX4-Autopilot directory.')
    parser.add_argument('--px4-target', default='px4_sitl',
                        help='First make target for PX4 SITL.')
    parser.add_argument('--px4-model', default='gz_zephyros',
                        help='Second make target/model, e.g. gz_zephyros.')
    parser.add_argument('--px4-command', default=None,
                        help='Override PX4 command. Uses bash when provided.')
    parser.add_argument('--ros-command',
                        default=('ros2 launch drone_control sim.launch.py '
                                 'controller_type:=mpcc_pp rviz:=false '
                                 'plotjuggler:=false'),
                        help='ROS command to run after PX4/Gazebo startup.')
    parser.add_argument('--startup-timeout-s', type=float, default=60.0,
                        help='Seconds to wait for PX4 readiness patterns.')
    parser.add_argument('--startup-delay-s', type=float, default=1.0,
                        help='Extra delay before starting ROS after PX4 is ready.')
    parser.add_argument('--headless', dest='headless', action='store_true',
                        default=True,
                        help='Run Gazebo server-only without the GUI.')
    parser.add_argument('--gui', dest='headless', action='store_false',
                        help='Launch the Gazebo GUI.')
    parser.add_argument('--sim-speed-factor', type=float, default=None,
                        help='Set PX4_SIM_SPEED_FACTOR for faster-than-real-time simulation.')
    parser.add_argument('--kill-existing', dest='kill_existing',
                        action='store_true', default=True,
                        help='Terminate stale PX4/Gazebo sim processes before starting.')
    parser.add_argument('--keep-existing', dest='kill_existing',
                        action='store_false',
                        help='Do not terminate existing sim processes before starting.')
    parser.add_argument('--strict-px4-ready', action='store_true',
                        help='Fail if readiness patterns are not observed.')
    parser.add_argument('--ready-pattern', action='append',
                        help='Extra regex that marks PX4/Gazebo ready.')
    parser.add_argument('--trial-timeout-s', type=float, default=None,
                        help='Optional hard timeout for this wrapper process.')
    parser.add_argument('--log-dir', type=Path, default=None,
                        help='Directory for px4_gazebo.log and ros_launch.log.')
    parser.add_argument('--stream-logs', action='store_true',
                        help='Mirror PX4 and ROS child logs to stdout.')
    parser.add_argument('--forward-metrics', dest='forward_metrics',
                        action='store_true', default=True,
                        help='Forward lap/stat metric lines to stdout for the tuner.')
    parser.add_argument('--no-forward-metrics', dest='forward_metrics',
                        action='store_false',
                        help='Do not forward lap/stat metric lines to stdout.')
    parser.add_argument('--dry-run', action='store_true',
                        help='Print the commands that would be launched.')
    return parser


def _terminate_all(processes: list[ManagedProcess]) -> None:
    for managed in reversed(processes):
        managed.terminate()


def _pkill_pattern(pattern: str, signal_name: str) -> None:
    subprocess.run(
        ['pkill', f'-{signal_name}', '-f', pattern],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def _terminate_existing_simulation(px4_dir: Path, grace_s: float = 2.0) -> None:
    """Clear stale PX4/Gazebo processes from this workspace before a trial."""
    px4_path = re.escape(str(px4_dir.resolve()))
    patterns = (
        rf'{px4_path}.*/bin/px4',
        rf'gz\s+sim .*{px4_path}',
    )
    for pattern in patterns:
        _pkill_pattern(pattern, 'TERM')
    time.sleep(grace_s)
    for pattern in patterns:
        _pkill_pattern(pattern, 'KILL')


def main(argv: list[str] | None = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    trial_dir = Path(os.environ.get('MPCC_PP_TRIAL_DIR', '.')).resolve()
    log_dir = (args.log_dir.resolve() if args.log_dir is not None
               else trial_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    if args.px4_command:
        px4_args = args.px4_command
        px4_shell = True
    else:
        px4_args = ['make', args.px4_target, args.px4_model]
        px4_shell = False

    if args.dry_run:
        print(f"PX4 dir: {args.px4_dir}")
        print(f"PX4 command: {px4_args}")
        print(f"ROS command: {args.ros_command}")
        print(f"Headless: {args.headless}")
        print(f"Startup delay: {args.startup_delay_s}")
        print(f"Simulation speed factor: {args.sim_speed_factor}")
        print(f"Kill existing sim processes: {args.kill_existing}")
        print(f"Stream child logs: {args.stream_logs}")
        print(f"Forward metric lines: {args.forward_metrics}")
        print(f"Log dir: {log_dir}")
        return 0

    if not args.px4_dir.exists():
        raise SystemExit(f"PX4 directory does not exist: {args.px4_dir}")

    if args.kill_existing:
        print('[mpcc_pp_gazebo_trial] clearing existing PX4/Gazebo processes',
              flush=True)
        _terminate_existing_simulation(args.px4_dir)

    env = os.environ.copy()
    if args.headless:
        env['HEADLESS'] = '1'
    else:
        env.pop('HEADLESS', None)
    if args.sim_speed_factor is not None:
        env['PX4_SIM_SPEED_FACTOR'] = str(args.sim_speed_factor)
    else:
        env.pop('PX4_SIM_SPEED_FACTOR', None)
    processes: list[ManagedProcess] = []
    stop_requested = threading.Event()

    def _handle_signal(signum, _frame) -> None:
        print(f"[mpcc_pp_gazebo_trial] received signal {signum}; cleaning up",
              flush=True)
        stop_requested.set()
        _terminate_all(processes)

    previous_handlers = {}
    for sig in (signal.SIGTERM, signal.SIGINT):
        previous_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, _handle_signal)

    ready_patterns = list(DEFAULT_READY_PATTERNS)
    if args.ready_pattern:
        ready_patterns.extend(args.ready_pattern)
    metric_patterns = (
        _compiled_patterns(list(DEFAULT_METRIC_PATTERNS))
        if args.forward_metrics
        else ()
    )

    px4 = ManagedProcess(
        name='PX4',
        args=px4_args,
        cwd=args.px4_dir,
        shell=px4_shell,
        log_path=log_dir / 'px4_gazebo.log',
        stream_logs=args.stream_logs,
        ready_patterns=_compiled_patterns(ready_patterns),
    )
    processes.append(px4)

    ros = ManagedProcess(
        name='ROS',
        args=args.ros_command,
        cwd=Path.cwd(),
        shell=True,
        log_path=log_dir / 'ros_launch.log',
        stream_logs=args.stream_logs,
        forward_patterns=metric_patterns,
    )

    start_time = time.monotonic()
    try:
        print('[mpcc_pp_gazebo_trial] starting PX4/Gazebo', flush=True)
        px4.start(env)

        ready = px4.ready.wait(timeout=args.startup_timeout_s)
        if px4.poll() is not None:
            print('[mpcc_pp_gazebo_trial] PX4/Gazebo exited during startup',
                  flush=True)
            return px4.poll() or 1
        if not ready:
            message = (
                '[mpcc_pp_gazebo_trial] PX4 readiness pattern not observed '
                f'within {args.startup_timeout_s:.1f}s')
            if args.strict_px4_ready:
                print(message, flush=True)
                return 1
            print(message + '; continuing because --strict-px4-ready is off',
                  flush=True)

        if args.startup_delay_s > 0.0:
            time.sleep(args.startup_delay_s)

        print('[mpcc_pp_gazebo_trial] starting ROS controller launch',
              flush=True)
        processes.append(ros)
        ros.start(env)

        while not stop_requested.is_set():
            if args.trial_timeout_s is not None:
                elapsed = time.monotonic() - start_time
                if elapsed > args.trial_timeout_s:
                    print('[mpcc_pp_gazebo_trial] trial timeout reached',
                          flush=True)
                    return 124
            ros_code = ros.poll()
            if ros_code is not None:
                return int(ros_code)
            px4_code = px4.poll()
            if px4_code is not None:
                print('[mpcc_pp_gazebo_trial] PX4/Gazebo exited unexpectedly',
                      flush=True)
                return int(px4_code or 1)
            time.sleep(0.5)
        return 130
    finally:
        _terminate_all(processes)
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
