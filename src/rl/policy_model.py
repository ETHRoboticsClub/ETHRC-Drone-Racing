import torch
import torch.nn as nn


class PolicyModel():
    def __init__(self, model_path: str, model_type: str = "pytorch"):
        self.policy = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.running_mean = None
        self.running_var = None

        if model_type == "onnx":
            print("ONNX model loading not implemented yet. Switching to pytorch.")
            model_type = "pytorch"
        
        if model_type == "pytorch":
            self.policy = self._load_pytorch_policy(model_path)
            self.policy.to(self.device)
            self.policy.eval()
        

    def to(self, device):
        self.device = device
        self.policy.to(device)
        if self.running_mean is not None:
            self.running_mean = self.running_mean.to(device)
            self.running_var = self.running_var.to(device)

    def inference(self, observations: torch.Tensor) -> torch.Tensor:
        observations = observations.to(self.device)
        
        # Apply RunningStandardScaler if loaded
        if self.running_mean is not None and self.running_var is not None:
            # Prevent division by zero
            std = torch.sqrt(self.running_var)
            std[std < 1e-8] = 1.0
            observations = (observations - self.running_mean) / std
            
        with torch.no_grad():
            actions = self.policy(observations)
        return actions    

    def _load_pytorch_policy(self, checkpoint_path: str) -> nn.Module:
        checkpoint = torch.load(checkpoint_path, map_location=self.device)
        policy_state = checkpoint.get("policy", checkpoint)

        # Attempt to load state preprocessor (RunningStandardScaler)
        if "state_preprocessor" in checkpoint:
            prep = checkpoint["state_preprocessor"]
            self.running_mean = prep.get("running_mean", None)
            self.running_var = prep.get("running_var", None)
            if self.running_mean is not None:
                print("Loaded SKRL RunningStandardScaler weights.")

        # Collect weight tensors in order to infer layer sizes
        weight_keys = sorted(
            k for k in policy_state
            if ("net_container" in k or "policy_layer" in k) and k.endswith(".weight")
        )

        layers = []
        for i, wk in enumerate(weight_keys):
            out_f, in_f = policy_state[wk].shape
            layers.append(nn.Linear(in_f, out_f))
            if i < len(weight_keys) - 1:  # activation after all but last
                layers.append(nn.ELU())

        policy = nn.Sequential(*layers)

        # Map old keys → new sequential keys
        # Linear layers are at indices 0, 2, 4, ... (interleaved with activations), last has no activation
        linear_indices = [i * 2 for i in range(len(weight_keys) - 1)] + [2 * (len(weight_keys) - 1)]
        old_weight_keys = sorted(
            k for k in policy_state
            if ("net_container" in k or "policy_layer" in k)
        )

        key_map = {}
        for old_key in old_weight_keys:
            # Determine which linear layer this belongs to
            wk = old_key.rsplit(".", 1)[0] + ".weight"
            idx = weight_keys.index(wk)
            param = old_key.rsplit(".", 1)[1]  # "weight" or "bias"
            key_map[f"{linear_indices[idx]}.{param}"] = policy_state[old_key]

        policy.load_state_dict(key_map)
        return policy









