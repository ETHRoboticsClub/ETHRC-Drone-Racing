from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


torch = pytest.importorskip("torch")

REPO_ROOT = Path(__file__).resolve().parents[3]
EXPORTER_PATH = REPO_ROOT / "scripts" / "export_skrl_to_torchscript.py"
SPEC = importlib.util.spec_from_file_location("export_skrl_to_torchscript", EXPORTER_PATH)
assert SPEC is not None and SPEC.loader is not None
exporter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(exporter)


def _checkpoint(obs_dim: int = 105):
    generator = torch.Generator().manual_seed(4)

    def randn(*shape):
        return torch.randn(*shape, generator=generator)

    return {
        "policy": {
            "log_std_parameter": randn(4),
            "net_container.0.weight": randn(7, obs_dim),
            "net_container.0.bias": randn(7),
            "net_container.2.weight": randn(5, 7),
            "net_container.2.bias": randn(5),
            "policy_layer.weight": randn(4, 5),
            "policy_layer.bias": randn(4),
            "value_layer.weight": randn(1, 5),
            "value_layer.bias": randn(1),
        },
        "observation_preprocessor": {
            "running_mean": randn(obs_dim).to(torch.float64),
            "running_variance": (torch.rand(obs_dim, generator=generator) + 0.1).to(torch.float64),
            "current_count": torch.tensor(1234.0, dtype=torch.float64),
        },
    }


def test_export_infers_105d_maps_scaler_and_matches_checkpoint(tmp_path):
    checkpoint = _checkpoint()
    checkpoint_path = tmp_path / "checkpoint.pt"
    output_path = tmp_path / "policy.pt"
    torch.save(checkpoint, checkpoint_path)

    actor = exporter.export_checkpoint(checkpoint_path, output_path)
    scripted = torch.jit.load(str(output_path))

    assert actor.obs_dim == 105
    assert actor.action_dim == 4
    assert tuple(actor.net[0].weight.shape) == (7, 105)
    torch.testing.assert_close(
        actor.running_mean,
        checkpoint["observation_preprocessor"]["running_mean"].float(),
    )
    torch.testing.assert_close(
        actor.running_variance,
        checkpoint["observation_preprocessor"]["running_variance"].float(),
    )
    assert actor.current_count.item() == pytest.approx(1234.0)

    observations = torch.randn(11, 105)
    expected = exporter.checkpoint_actor_forward(
        observations,
        checkpoint["policy"],
        checkpoint["observation_preprocessor"],
    )
    torch.testing.assert_close(scripted(observations), expected, atol=1e-5, rtol=1e-5)


@pytest.mark.parametrize(
    "mutation, error",
    [
        (lambda checkpoint: checkpoint["policy"].pop("policy_layer.bias"), "missing"),
        (
            lambda checkpoint: checkpoint["policy"].__setitem__("mystery.weight", torch.ones(1)),
            "Unexpected policy checkpoint keys",
        ),
        (
            lambda checkpoint: checkpoint["observation_preprocessor"].pop("current_count"),
            "preprocessor mismatch",
        ),
    ],
)
def test_export_fails_loudly_on_incomplete_or_unknown_state(mutation, error):
    checkpoint = _checkpoint()
    mutation(checkpoint)
    with pytest.raises((KeyError, ValueError), match=error):
        exporter.actor_from_checkpoint(checkpoint)

