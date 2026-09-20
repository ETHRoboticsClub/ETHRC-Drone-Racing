from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R


FRAME_FLIP = np.array([1.0, -1.0, -1.0], dtype=np.float64)
GRAVITY_NWU = np.array([0.0, 0.0, -1.0], dtype=np.float64)
CORNER_SIGNS = (
    (-1.0, 1.0),   # NW
    (1.0, 1.0),    # NE
    (1.0, -1.0),   # SE
    (-1.0, -1.0),  # SW
)

LEGACY_CORNERS_25 = "legacy_corners_25"
AIGP_OURS_105 = "aigp_ours_105"
AIGP_OURS_HISTORY_LENGTH = 3
AIGP_OURS_GATE_COUNT = 3
AIGP_OURS_REL_POSITION_SCALE_M = 10.0

# Exact flattened layout produced by Isaac Lab's ObservationManager. Each
# history is oldest -> newest, and histories are concatenated term-by-term.
AIGP_OURS_105_MANIFEST: tuple[tuple[str, int, int], ...] = (
    ("base_lin_vel_body_flu_history", 0, 9),
    ("base_ang_vel_body_flu_history", 9, 18),
    ("projected_gravity_body_history", 18, 27),
    ("last_action_history", 27, 39),
    ("next_3_gates_rel_forward_half_width_history", 39, 102),
    ("velocity_limit_history", 102, 105),
)
OBSERVATION_CONTRACT_DIMS = {LEGACY_CORNERS_25: 25, AIGP_OURS_105: 105}


@dataclass(frozen=True)
class Gate:
    position: np.ndarray
    rotation: R
    inner_size_m: float


@dataclass(frozen=True)
class TrainingState:
    position_w: np.ndarray
    rotation_wb: R
    body_lin_vel: np.ndarray
    body_ang_vel: np.ndarray
    gravity_body: np.ndarray


def _as_vec3(values: Any, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"{name} must have exactly 3 elements, got shape {arr.shape}")
    return arr


def _parse_rotation(raw: Any) -> tuple[float, float, float]:
    if raw is None:
        return 0.0, 0.0, 0.0
    if not isinstance(raw, (list, tuple)):
        raise ValueError(f"rotation must be a list/tuple, got {raw!r}")
    if len(raw) == 1:
        return 0.0, 0.0, float(raw[0])
    if len(raw) == 3:
        return float(raw[0]), float(raw[1]), float(raw[2])
    raise ValueError(f"rotation must have 1 or 3 elements, got {len(raw)}")


def gate_rotation_from_rpy_deg(roll_deg: float, pitch_deg: float, yaw_deg: float) -> R:
    """Return world-from-gate rotation using the training env's gate convention.

    The training env stores gate yaw as clockwise from +Y, while scipy uses
    counter-clockwise yaw around +Z. Negating yaw reproduces the IsaacLab loader.
    """

    return R.from_euler("xyz", [roll_deg, pitch_deg, -yaw_deg], degrees=True)


def load_track(path: str | Path) -> list[Gate]:
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise RuntimeError("PyYAML is required to load AIGP track YAML files.") from exc

    track_path = Path(path)
    if not track_path.exists():
        raise FileNotFoundError(f"AIGP track YAML not found: {track_path}")

    data = yaml.safe_load(track_path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"AIGP track YAML must be a mapping: {track_path}")

    entries = data.get("gate_track", data.get("gates"))
    if not entries:
        raise ValueError(f"AIGP track YAML contains no gates: {track_path}")

    default_inner_size = float(data.get("inner_size_m", 1.5))
    track_offset = np.asarray(data.get("track_offset", [0.0, 0.0, 0.0]), dtype=np.float64)
    subtract_offset = data.get("frame", "nwu") == "nwu_offset"

    gates: list[Gate] = []
    for idx, entry in enumerate(entries):
        if not isinstance(entry, dict):
            raise ValueError(f"Gate {idx} must be a mapping, got {entry!r}")
        pos = _as_vec3(entry.get("position"), f"gate {idx} position")
        if subtract_offset:
            pos = pos - track_offset
        roll, pitch, yaw = _parse_rotation(entry.get("rotation"))
        gates.append(
            Gate(
                position=pos,
                rotation=gate_rotation_from_rpy_deg(roll, pitch, yaw),
                inner_size_m=float(entry.get("inner_size_m", default_inner_size)),
            )
        )
    return gates


def ned_frd_state_to_training(state: np.ndarray, velocity_frame: str = "world_ned") -> TrainingState:
    """Convert AIGP NED/FRD state to training NWU/FLU quantities."""

    state = np.asarray(state, dtype=np.float64)
    if state.shape[0] < 13:
        raise ValueError(f"Expected at least 13 state values, got {state.shape[0]}")

    pos_ned = state[0:3]
    lin_vel = state[3:6]
    quat_wxyz = state[6:10]
    ang_vel_frd = state[10:13]

    rot_ned_from_frd = R.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
    flip_matrix = np.diag(FRAME_FLIP)
    rot_nwu_from_flu = R.from_matrix(flip_matrix @ rot_ned_from_frd.as_matrix() @ flip_matrix)

    pos_nwu = FRAME_FLIP * pos_ned

    if velocity_frame == "world_ned":
        lin_vel_world = FRAME_FLIP * lin_vel
        body_lin_vel = rot_nwu_from_flu.inv().apply(lin_vel_world)
    elif velocity_frame == "world_nwu":
        body_lin_vel = rot_nwu_from_flu.inv().apply(lin_vel)
    elif velocity_frame == "body_frd":
        body_lin_vel = FRAME_FLIP * lin_vel
    elif velocity_frame == "body_flu":
        body_lin_vel = lin_vel
    else:
        raise ValueError(
            "velocity_frame must be one of: world_ned, world_nwu, body_frd, body_flu"
        )

    return TrainingState(
        position_w=pos_nwu,
        rotation_wb=rot_nwu_from_flu,
        body_lin_vel=np.asarray(body_lin_vel, dtype=np.float64),
        body_ang_vel=FRAME_FLIP * ang_vel_frd,
        gravity_body=rot_nwu_from_flu.inv().apply(GRAVITY_NWU),
    )


def gate_inner_corners_world(gate: Gate) -> np.ndarray:
    half = 0.5 * gate.inner_size_m
    corners_local = np.array(
        [[x_sign * half, 0.0, z_sign * half] for x_sign, z_sign in CORNER_SIGNS],
        dtype=np.float64,
    )
    return gate.position.reshape(1, 3) + gate.rotation.apply(corners_local)


def build_observation(
    state: np.ndarray,
    gates: list[Gate],
    active_gate_index: int,
    previous_action: np.ndarray,
    velocity_frame: str = "world_ned",
) -> np.ndarray:
    if not gates:
        raise ValueError("At least one gate is required to build an AIGP policy observation.")

    prev = np.asarray(previous_action, dtype=np.float64)
    if prev.shape != (4,):
        raise ValueError(f"previous_action must have shape (4,), got {prev.shape}")

    idx = int(np.clip(active_gate_index, 0, len(gates) - 1))
    training_state = ned_frd_state_to_training(state, velocity_frame=velocity_frame)
    rel_corners_w = gate_inner_corners_world(gates[idx]) - training_state.position_w.reshape(1, 3)
    corners_body = training_state.rotation_wb.inv().apply(rel_corners_w).reshape(12)

    obs = np.concatenate(
        [
            training_state.body_lin_vel,
            training_state.body_ang_vel,
            training_state.gravity_body,
            corners_body,
            prev,
        ]
    ).astype(np.float32)
    if obs.shape != (25,):
        raise RuntimeError(f"AIGP policy observation must be 25D, got {obs.shape}")
    return obs


def _ours_observation_terms(
    state: np.ndarray,
    gates: list[Gate],
    active_gate_index: int,
    previous_action: np.ndarray,
    velocity_limit: float,
    velocity_frame: str,
) -> tuple[np.ndarray, ...]:
    """Build one unstacked sample for every term in the Isaac ``ours`` group."""

    if not gates:
        raise ValueError("At least one gate is required to build an AIGP policy observation.")
    previous_action = np.asarray(previous_action, dtype=np.float64)
    if previous_action.shape != (4,):
        raise ValueError(f"previous_action must have shape (4,), got {previous_action.shape}")
    if not np.isfinite(velocity_limit) or velocity_limit <= 0.0:
        raise ValueError(f"velocity_limit must be finite and positive, got {velocity_limit}")

    training_state = ned_frd_state_to_training(state, velocity_frame=velocity_frame)
    active = int(np.clip(active_gate_index, 0, len(gates) - 1))
    gate_features: list[np.ndarray] = []
    for offset in range(AIGP_OURS_GATE_COUNT):
        gate = gates[(active + offset) % len(gates)]
        relative_world = gate.position - training_state.position_w
        relative_body = training_state.rotation_wb.inv().apply(relative_world)
        forward_world = gate.rotation.apply(np.array([0.0, 1.0, 0.0]))
        forward_body = training_state.rotation_wb.inv().apply(forward_world)
        gate_features.append(
            np.concatenate(
                (
                    relative_body / AIGP_OURS_REL_POSITION_SCALE_M,
                    forward_body,
                    np.array([0.5 * gate.inner_size_m]),
                )
            )
        )

    return (
        training_state.body_lin_vel.astype(np.float32),
        training_state.body_ang_vel.astype(np.float32),
        training_state.gravity_body.astype(np.float32),
        previous_action.astype(np.float32),
        np.concatenate(gate_features).astype(np.float32),
        np.array([velocity_limit], dtype=np.float32),
    )


class AigpOursObservationBuilder:
    """Stateful 105D observation builder matching Isaac Lab ``ours`` exactly.

    Isaac Lab initializes an empty term history by repeating its first sample.
    ``reset`` follows the same behavior on the next call to :meth:`build`.
    """

    contract = AIGP_OURS_105
    observation_dim = 105

    def __init__(self, velocity_frame: str = "world_ned") -> None:
        self.velocity_frame = velocity_frame
        self._history: list[list[np.ndarray]] = []

    def reset(self) -> None:
        self._history.clear()

    def build(
        self,
        state: np.ndarray,
        gates: list[Gate],
        active_gate_index: int,
        previous_action: np.ndarray,
        velocity_limit: float,
    ) -> np.ndarray:
        terms = _ours_observation_terms(
            state,
            gates,
            active_gate_index,
            previous_action,
            velocity_limit,
            self.velocity_frame,
        )
        if not self._history:
            self._history = [
                [term.copy() for _ in range(AIGP_OURS_HISTORY_LENGTH)] for term in terms
            ]
        else:
            if len(self._history) != len(terms):
                raise RuntimeError("Observation history term count changed unexpectedly")
            for history, term in zip(self._history, terms):
                if term.shape != history[-1].shape:
                    raise RuntimeError(
                        f"Observation term shape changed from {history[-1].shape} to {term.shape}"
                    )
                history.pop(0)
                history.append(term.copy())

        observation = np.concatenate(
            [np.concatenate(history) for history in self._history]
        ).astype(np.float32)
        if observation.shape != (self.observation_dim,):
            raise RuntimeError(
                f"{self.contract} observation must be {self.observation_dim}D, got {observation.shape}"
            )
        return observation


def policy_action_to_attitude_cmd(
    action: np.ndarray,
    rate_command_scale_rad_s: float = 30.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Map policy action to /drone_attitude_cmd and clamped previous action.

    Returns:
        (cmd, clamped_action)
        cmd is [roll_deg_s, pitch_deg_s, yaw_deg_s, thrust] in AIGP FRD signs.
    """

    raw = np.asarray(action, dtype=np.float64)
    if raw.shape != (4,):
        raise ValueError(f"Policy action must have shape (4,), got {raw.shape}")

    clamped = np.clip(raw, -1.0, 1.0)
    thrust = float(np.clip((clamped[0] + 1.0) * 0.5, 0.0, 1.0))
    rates_flu = clamped[1:4] * float(rate_command_scale_rad_s)
    rates_frd = FRAME_FLIP * rates_flu
    cmd = np.array(
        [
            np.degrees(rates_frd[0]),
            np.degrees(rates_frd[1]),
            np.degrees(rates_frd[2]),
            thrust,
        ],
        dtype=np.float64,
    )
    return cmd, clamped.astype(np.float32)


class TorchScriptPolicy:
    def __init__(
        self,
        path: str | Path,
        device: str = "cpu",
        observation_contract: str | None = None,
    ):
        try:
            import torch
        except ModuleNotFoundError as exc:
            raise RuntimeError("PyTorch is required to run Nicola's TorchScript policy.") from exc

        self._torch = torch
        self.device = torch.device(device)
        self.model = torch.jit.load(str(path), map_location=self.device)
        self.model.eval()
        self.observation_dim = self._infer_observation_dim()
        inferred_contracts = [
            name for name, dimension in OBSERVATION_CONTRACT_DIMS.items()
            if dimension == self.observation_dim
        ]
        self.observation_contract = inferred_contracts[0] if len(inferred_contracts) == 1 else None
        if observation_contract is not None:
            if observation_contract not in OBSERVATION_CONTRACT_DIMS:
                raise ValueError(
                    f"Unknown observation contract {observation_contract!r}; "
                    f"expected one of {sorted(OBSERVATION_CONTRACT_DIMS)}"
                )
            expected_dim = OBSERVATION_CONTRACT_DIMS[observation_contract]
            if expected_dim != self.observation_dim:
                raise ValueError(
                    f"Policy input is {self.observation_dim}D but contract "
                    f"{observation_contract!r} requires {expected_dim}D"
                )
            self.observation_contract = observation_contract
        self.reset()

    def _infer_observation_dim(self) -> int:
        # Exported actors contain a running_mean buffer. Legacy TorchScript
        # artifacts can be inferred from the first matrix-shaped parameter.
        for name, value in self.model.named_buffers():
            if name.endswith("running_mean") and value.ndim == 1:
                return int(value.shape[0])
        for _, value in self.model.named_parameters():
            if value.ndim == 2:
                return int(value.shape[1])
        raise ValueError("Unable to infer TorchScript policy observation dimension")

    def reset(self) -> None:
        if hasattr(self.model, "reset"):
            with self._torch.no_grad():
                self.model.reset()

    def __call__(self, observation: np.ndarray) -> np.ndarray:
        obs = np.asarray(observation, dtype=np.float32)
        if obs.shape != (self.observation_dim,):
            raise ValueError(
                f"AIGP policy expects a {self.observation_dim}D observation, got {obs.shape}"
            )
        tensor = self._torch.as_tensor(obs, dtype=self._torch.float32, device=self.device).unsqueeze(0)
        with self._torch.no_grad():
            action = self.model(tensor)
        return action.squeeze(0).detach().cpu().numpy().astype(np.float32)
