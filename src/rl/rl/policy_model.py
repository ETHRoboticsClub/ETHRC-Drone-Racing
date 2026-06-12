import torch
import torch.nn as nn



class PolicyModel():
    def __init__(self, model_path: str, model_type: str = "pytorch"):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.running_mean = None
        self.running_var = None
        self._ort_session = None
        self.policy = None

        if model_type == "onnx":
            self._load_onnx_policy(model_path)
        elif model_type == "pytorch":
            self.policy = self._load_pytorch_policy(model_path)
            self.policy.to(self.device)
            self.policy.eval()
        else:
            raise ValueError(f"Unknown model_type: {model_type!r}")

    def to(self, device):
        self.device = device
        if self.policy is not None:
            self.policy.to(device)
        if self.running_mean is not None:
            self.running_mean = self.running_mean.to(device)
            self.running_var = self.running_var.to(device)

    def inference(self, observations: torch.Tensor) -> torch.Tensor:
        if self._ort_session is not None:
            return self._inference_onnx(observations)
        return self._inference_pytorch(observations)

    def _inference_pytorch(self, observations: torch.Tensor) -> torch.Tensor:
        observations = observations.to(self.device)
        if self.running_mean is not None and self.running_var is not None:
            std = torch.sqrt(self.running_var)
            std[std < 1e-8] = 1.0
            observations = (observations - self.running_mean) / std
        with torch.no_grad():
            return self.policy(observations)

    def _inference_onnx(self, observations: torch.Tensor) -> torch.Tensor:
        obs_np = observations.float().cpu().numpy()
        out = self._ort_session.run(["actions"], {"obs": obs_np})[0]
        return torch.tensor(out)

    def _load_onnx_policy(self, model_path: str) -> None:
        try:
            import onnxruntime as ort
        except (ImportError, AttributeError) as e:
            raise RuntimeError(f"onnxruntime unavailable: {e}") from e
        self._ort_session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        print(f"Loaded ONNX model: {model_path}")

    def _load_pytorch_policy(self, checkpoint_path: str) -> nn.Module:
        checkpoint = torch.load(checkpoint_path, map_location=self.device, weights_only=True)
        policy_state = checkpoint.get("policy", checkpoint)

        prep = checkpoint.get("observation_preprocessor", checkpoint.get("state_preprocessor", None))
        if prep is not None:
            self.running_mean = prep.get("running_mean", None)
            self.running_var = prep.get("running_variance", None)
            if self.running_mean is not None:
                self.running_mean = self.running_mean.float()
                self.running_var = self.running_var.float()
                print("Loaded SKRL RunningStandardScaler weights.")

        weight_keys = sorted(
            k for k in policy_state
            if ("net_container" in k or "policy_layer" in k) and k.endswith(".weight")
        )

        layers = []
        for i, wk in enumerate(weight_keys):
            out_f, in_f = policy_state[wk].shape
            layers.append(nn.Linear(in_f, out_f))
            if i < len(weight_keys) - 1:
                layers.append(nn.ELU())

        policy = nn.Sequential(*layers)

        linear_indices = [i * 2 for i in range(len(weight_keys) - 1)] + [2 * (len(weight_keys) - 1)]
        old_weight_keys = sorted(
            k for k in policy_state
            if ("net_container" in k or "policy_layer" in k)
        )

        key_map = {}
        for old_key in old_weight_keys:
            wk = old_key.rsplit(".", 1)[0] + ".weight"
            idx = weight_keys.index(wk)
            param = old_key.rsplit(".", 1)[1]
            key_map[f"{linear_indices[idx]}.{param}"] = policy_state[old_key]

        policy.load_state_dict(key_map)
        return policy
