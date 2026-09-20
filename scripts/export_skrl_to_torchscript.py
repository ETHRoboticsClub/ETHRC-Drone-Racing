#!/usr/bin/env python3
"""Export an SKRL PPO actor checkpoint as a self-contained TorchScript model.

The exporter intentionally understands the state-dict layout produced by the
SKRL model instantiator used in this repository.  It does not use
``strict=False``: an incompatible checkpoint must fail at export time rather
than produce a randomly initialized policy that happens to run.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
from typing import Mapping

import torch
from torch import nn
from torch.nn import functional as F


_HIDDEN_PARAMETER = re.compile(r"^net_container\.(\d+)\.(weight|bias)$")
_KNOWN_NON_ACTOR_POLICY_KEYS = {
    "log_std_parameter",
    "value_layer.weight",
    "value_layer.bias",
}
_PREPROCESSOR_KEYS = {"running_mean", "running_variance", "current_count"}


class ExportedSkrlActor(nn.Module):
    """Deterministic mean actor, including SKRL observation preprocessing."""

    def __init__(
        self,
        obs_dim: int,
        action_dim: int,
        hidden_dims: tuple[int, ...],
        clip_threshold: float = 5.0,
        epsilon: float = 1.0e-8,
    ) -> None:
        super().__init__()
        self.obs_dim = int(obs_dim)
        self.action_dim = int(action_dim)
        self.clip_threshold = float(clip_threshold)
        self.epsilon = float(epsilon)

        self.register_buffer("running_mean", torch.zeros(obs_dim, dtype=torch.float32))
        self.register_buffer("running_variance", torch.ones(obs_dim, dtype=torch.float32))
        # Retained in the artifact for auditability and future scaler updates.
        self.register_buffer("current_count", torch.zeros((), dtype=torch.float32))

        layers: list[nn.Module] = []
        in_features = obs_dim
        for out_features in hidden_dims:
            layers.extend((nn.Linear(in_features, out_features), nn.ELU()))
            in_features = out_features
        layers.append(nn.Linear(in_features, action_dim))
        self.net = nn.Sequential(*layers)

    def forward(self, observations: torch.Tensor) -> torch.Tensor:
        normalized = (observations - self.running_mean) / torch.sqrt(
            self.running_variance + self.epsilon
        )
        normalized = torch.clamp(
            normalized, -self.clip_threshold, self.clip_threshold
        )
        # SKRL's Gaussian actor uses this as the deterministic mean. The agent
        # config has clip_actions=False, so output clipping does not belong here.
        return self.net(normalized)


def _as_tensor_mapping(value: object, name: str) -> Mapping[str, torch.Tensor]:
    if not isinstance(value, Mapping):
        raise TypeError(f"Checkpoint {name!r} must be a mapping, got {type(value).__name__}")
    bad = [key for key, item in value.items() if not isinstance(key, str) or not torch.is_tensor(item)]
    if bad:
        raise TypeError(f"Checkpoint {name!r} contains non-tensor entries: {bad}")
    return value


def _actor_layout(
    policy_state: Mapping[str, torch.Tensor],
) -> tuple[list[tuple[str, str]], int, int, tuple[int, ...]]:
    """Validate actor keys/shapes and return source-key to destination-key pairs."""

    hidden_indices: list[int] = []
    actor_keys: set[str] = set()
    for key in policy_state:
        match = _HIDDEN_PARAMETER.fullmatch(key)
        if match:
            actor_keys.add(key)
            if match.group(2) == "weight":
                hidden_indices.append(int(match.group(1)))
        elif key.startswith("net_container.") or key.startswith("policy_layer."):
            actor_keys.add(key)

    unexpected = set(policy_state) - actor_keys - _KNOWN_NON_ACTOR_POLICY_KEYS
    if unexpected:
        raise ValueError(f"Unexpected policy checkpoint keys: {sorted(unexpected)}")
    if not hidden_indices:
        raise ValueError("No net_container.<index>.weight actor layers found")

    hidden_indices = sorted(set(hidden_indices))
    expected_actor = {"policy_layer.weight", "policy_layer.bias"}
    for index in hidden_indices:
        expected_actor.update(
            {f"net_container.{index}.weight", f"net_container.{index}.bias"}
        )
    missing = expected_actor - actor_keys
    extra = actor_keys - expected_actor
    if missing or extra:
        raise ValueError(
            f"Actor parameter mismatch; missing={sorted(missing)}, unexpected={sorted(extra)}"
        )

    mappings: list[tuple[str, str]] = []
    hidden_dims: list[int] = []
    previous_out: int | None = None
    obs_dim: int | None = None
    for layer_number, source_index in enumerate(hidden_indices):
        weight_key = f"net_container.{source_index}.weight"
        bias_key = f"net_container.{source_index}.bias"
        weight = policy_state[weight_key]
        bias = policy_state[bias_key]
        if weight.ndim != 2 or bias.shape != (weight.shape[0],):
            raise ValueError(
                f"Invalid shapes for {weight_key}/{bias_key}: {tuple(weight.shape)}, {tuple(bias.shape)}"
            )
        out_features, in_features = map(int, weight.shape)
        if previous_out is not None and in_features != previous_out:
            raise ValueError(
                f"Actor layers do not connect: {weight_key} expects {in_features}, previous emits {previous_out}"
            )
        if obs_dim is None:
            obs_dim = in_features
        previous_out = out_features
        hidden_dims.append(out_features)
        destination_index = 2 * layer_number
        mappings.extend(
            ((weight_key, f"net.{destination_index}.weight"), (bias_key, f"net.{destination_index}.bias"))
        )

    output_weight = policy_state["policy_layer.weight"]
    output_bias = policy_state["policy_layer.bias"]
    if output_weight.ndim != 2 or output_bias.shape != (output_weight.shape[0],):
        raise ValueError(
            "Invalid policy_layer shapes: "
            f"{tuple(output_weight.shape)}, {tuple(output_bias.shape)}"
        )
    action_dim, output_in = map(int, output_weight.shape)
    if output_in != previous_out:
        raise ValueError(
            f"policy_layer expects {output_in} values, final hidden layer emits {previous_out}"
        )
    output_index = 2 * len(hidden_indices)
    mappings.extend(
        (("policy_layer.weight", f"net.{output_index}.weight"), ("policy_layer.bias", f"net.{output_index}.bias"))
    )
    assert obs_dim is not None
    return mappings, obs_dim, action_dim, tuple(hidden_dims)


def actor_from_checkpoint(
    checkpoint: Mapping[str, object],
    *,
    clip_threshold: float = 5.0,
    require_preprocessor: bool = True,
) -> tuple[ExportedSkrlActor, Mapping[str, torch.Tensor]]:
    """Build and strictly populate an actor from a loaded SKRL checkpoint."""

    if "policy" not in checkpoint:
        raise KeyError("SKRL checkpoint is missing the top-level 'policy' state dict")
    policy_state = _as_tensor_mapping(checkpoint["policy"], "policy")
    mappings, obs_dim, action_dim, hidden_dims = _actor_layout(policy_state)
    actor = ExportedSkrlActor(
        obs_dim=obs_dim,
        action_dim=action_dim,
        hidden_dims=hidden_dims,
        clip_threshold=clip_threshold,
    )

    destination = actor.state_dict()
    for source_key, destination_key in mappings:
        destination[destination_key] = policy_state[source_key].detach().to(dtype=torch.float32)

    preprocessor_raw = checkpoint.get("observation_preprocessor")
    if preprocessor_raw is None:
        if require_preprocessor:
            raise KeyError("SKRL checkpoint is missing 'observation_preprocessor'")
    else:
        preprocessor = _as_tensor_mapping(preprocessor_raw, "observation_preprocessor")
        missing = _PREPROCESSOR_KEYS - set(preprocessor)
        unexpected = set(preprocessor) - _PREPROCESSOR_KEYS
        if missing or unexpected:
            raise ValueError(
                "Observation preprocessor mismatch; "
                f"missing={sorted(missing)}, unexpected={sorted(unexpected)}"
            )
        for source_key in ("running_mean", "running_variance"):
            if preprocessor[source_key].shape != (obs_dim,):
                raise ValueError(
                    f"{source_key} must have shape ({obs_dim},), got {tuple(preprocessor[source_key].shape)}"
                )
        if preprocessor["current_count"].numel() != 1:
            raise ValueError("current_count must be scalar")
        destination["running_mean"] = preprocessor["running_mean"].detach().to(torch.float32)
        destination["running_variance"] = preprocessor["running_variance"].detach().to(torch.float32)
        destination["current_count"] = preprocessor["current_count"].detach().reshape(()).to(torch.float32)

    actor.load_state_dict(destination, strict=True)
    actor.eval()
    return actor, policy_state


def checkpoint_actor_forward(
    observations: torch.Tensor,
    policy_state: Mapping[str, torch.Tensor],
    preprocessor_state: Mapping[str, torch.Tensor],
    *,
    clip_threshold: float = 5.0,
    epsilon: float = 1.0e-8,
) -> torch.Tensor:
    """Independent reference for the deterministic SKRL actor computation."""

    hidden_indices = sorted(
        int(match.group(1))
        for key in policy_state
        if (match := re.fullmatch(r"net_container\.(\d+)\.weight", key))
    )
    x = observations.to(torch.float32)
    mean = preprocessor_state["running_mean"].to(dtype=x.dtype, device=x.device)
    variance = preprocessor_state["running_variance"].to(dtype=x.dtype, device=x.device)
    x = torch.clamp((x - mean) / torch.sqrt(variance + epsilon), -clip_threshold, clip_threshold)
    for index in hidden_indices:
        x = F.elu(
            F.linear(
                x,
                policy_state[f"net_container.{index}.weight"].to(x),
                policy_state[f"net_container.{index}.bias"].to(x),
            )
        )
    return F.linear(
        x,
        policy_state["policy_layer.weight"].to(x),
        policy_state["policy_layer.bias"].to(x),
    )


def check_parity(
    actor: ExportedSkrlActor,
    checkpoint: Mapping[str, object],
    *,
    samples: int = 32,
    tolerance: float = 1.0e-5,
) -> float:
    """Compare mapped actor output with independent checkpoint actor math."""

    policy_state = _as_tensor_mapping(checkpoint["policy"], "policy")
    preprocessor = _as_tensor_mapping(
        checkpoint["observation_preprocessor"], "observation_preprocessor"
    )
    generator = torch.Generator(device="cpu").manual_seed(0)
    observations = torch.randn(samples, actor.obs_dim, generator=generator)
    observations[0].zero_()
    with torch.no_grad():
        expected = checkpoint_actor_forward(
            observations,
            policy_state,
            preprocessor,
            clip_threshold=actor.clip_threshold,
            epsilon=actor.epsilon,
        )
        actual = actor(observations)
    max_error = float(torch.max(torch.abs(expected - actual)).item())
    if max_error > tolerance:
        raise RuntimeError(
            f"Checkpoint parity failed: max |actor-reference|={max_error:.3e} > {tolerance:.3e}"
        )
    return max_error


def export_checkpoint(
    checkpoint_path: str | Path,
    output_path: str | Path,
    *,
    clip_threshold: float = 5.0,
    parity_tolerance: float = 1.0e-5,
) -> ExportedSkrlActor:
    checkpoint_path = Path(checkpoint_path)
    output_path = Path(output_path)
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, Mapping):
        raise TypeError(f"Checkpoint must be a mapping, got {type(checkpoint).__name__}")

    actor, _ = actor_from_checkpoint(checkpoint, clip_threshold=clip_threshold)
    max_error = check_parity(actor, checkpoint, tolerance=parity_tolerance)
    scripted = torch.jit.script(actor)

    # Include the serialization boundary in parity validation.
    example = torch.randn(8, actor.obs_dim)
    with torch.no_grad():
        serialization_error = float(torch.max(torch.abs(scripted(example) - actor(example))).item())
    if serialization_error > parity_tolerance:
        raise RuntimeError(
            "TorchScript parity failed: "
            f"max |scripted-actor|={serialization_error:.3e} > {parity_tolerance:.3e}"
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    scripted.save(str(output_path))
    print(
        f"Exported {actor.obs_dim}D -> {actor.action_dim}D actor to {output_path} "
        f"(checkpoint max error {max_error:.3e}, TorchScript {serialization_error:.3e})"
    )
    return actor


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, help="SKRL .pt checkpoint")
    parser.add_argument("--output", required=True, help="Output TorchScript .pt")
    parser.add_argument(
        "--normalization-clip",
        type=float,
        default=5.0,
        help="SKRL RunningStandardScaler clip threshold (default: 5)",
    )
    parser.add_argument("--parity-tolerance", type=float, default=1.0e-5)
    args = parser.parse_args()
    export_checkpoint(
        args.checkpoint,
        args.output,
        clip_threshold=args.normalization_clip,
        parity_tolerance=args.parity_tolerance,
    )


if __name__ == "__main__":
    main()
