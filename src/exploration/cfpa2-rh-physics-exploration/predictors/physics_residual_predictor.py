from __future__ import annotations

import importlib
import math
import time
from pathlib import Path

import numpy as np

from core.predictor_features import feature_dimension
from core.types import Pose2D, PredictorInput, PredictorOutput

from .base_predictor import BasePredictor
from .path_follow_predictor import PathFollowPredictor


def _lazy_import_torch():
    try:
        torch = importlib.import_module("torch")
        nn = importlib.import_module("torch.nn")
        return torch, nn
    except Exception:
        return None, None


def _build_torch_mlp(nn_module, input_dim: int, hidden_dims: list[int]):
    dims = [int(input_dim)] + [int(h) for h in hidden_dims] + [2]
    layers = []
    for i in range(len(dims) - 1):
        layers.append(nn_module.Linear(dims[i], dims[i + 1]))
        if i < len(dims) - 2:
            layers.append(nn_module.ReLU())
    return nn_module.Sequential(*layers)


def _torch_load_compat(torch_module, path: Path):
    # PyTorch 2.6 defaulted to weights_only=True in torch.load.
    # Predictor checkpoints include metadata arrays, so require full load.
    try:
        return torch_module.load(path, map_location="cpu", weights_only=False)
    except TypeError:
        # Older torch versions do not support the weights_only argument.
        return torch_module.load(path, map_location="cpu")


def _choose_torch_device(torch_module, device_flag: str):
    flag = str(device_flag).strip().lower()
    if flag == "cpu":
        return torch_module.device("cpu")
    if flag == "cuda":
        if torch_module.cuda.is_available():
            return torch_module.device("cuda")
        return torch_module.device("cpu")
    if torch_module.cuda.is_available():
        return torch_module.device("cuda")
    return torch_module.device("cpu")


class PhysicsResidualPredictor(BasePredictor):
    """Physics-informed residual predictor with pluggable backends.

    Backends:
    - `npz_linear`: lightweight linear residual head (legacy-compatible)
    - `torch_mlp`: learned MLP residual model from large-scale dataset
    """

    name = "physics_residual"

    def __init__(
        self,
        enabled: bool = True,
        weight_file: str | None = None,
        residual_scale: float = 0.35,
        occupancy_patch_radius: int = 4,
        hidden_dims: tuple[int, ...] = (128, 128),
        device: str = "auto",
        enable_uncertainty: bool = True,
        uncertainty_ensemble_samples: int = 5,
        uncertainty_feature_noise_std: float = 0.03,
        uncertainty_base: float = 0.03,
        uncertainty_scale: float = 1.0,
        uncertainty_max: float = 2.0,
    ):
        self.enabled = bool(enabled)
        self.weight_file = weight_file
        self.residual_scale = float(residual_scale)
        self.patch_radius = int(occupancy_patch_radius)
        self.hidden_dims = tuple(int(h) for h in hidden_dims)
        self.device_flag = str(device)
        self.enable_uncertainty = bool(enable_uncertainty)
        self.uncertainty_ensemble_samples = max(1, int(uncertainty_ensemble_samples))
        self.uncertainty_feature_noise_std = float(uncertainty_feature_noise_std)
        self.uncertainty_base = float(uncertainty_base)
        self.uncertainty_scale = float(uncertainty_scale)
        self.uncertainty_max = float(uncertainty_max)
        self.fallback = PathFollowPredictor()

        input_dim = feature_dimension(self.patch_radius)
        self.w = np.zeros((2, input_dim), dtype=np.float32)
        self.b = np.zeros((2,), dtype=np.float32)

        self.loaded = False
        self.backend = "analytic"
        self.expected_input_dim = int(input_dim)

        self._torch = None
        self._nn = None
        self.torch_device = None
        self.torch_model = None
        self.torch_feature_mean: np.ndarray | None = None
        self.torch_feature_std: np.ndarray | None = None

        if weight_file:
            self._try_load_weights(weight_file)

    def _try_load_weights(self, weight_file: str) -> None:
        p = Path(weight_file)
        if not p.exists():
            return

        suffix = p.suffix.lower()
        if suffix == ".npz":
            self._load_npz(p)
            return
        if suffix in (".pt", ".pth"):
            self._load_torch(p)
            return

    def _load_npz(self, path: Path) -> None:
        try:
            data = np.load(path)
            w = data.get("w")
            b = data.get("b")
            if w is None or b is None:
                return
            w = np.asarray(w, dtype=np.float32)
            b = np.asarray(b, dtype=np.float32)
            if w.ndim == 2 and w.shape[0] == 2 and b.shape == (2,):
                self.w = w
                self.b = b
                self.loaded = True
                self.backend = "npz_linear"
                self.expected_input_dim = int(w.shape[1])
        except Exception:
            self.loaded = False

    def _load_torch(self, path: Path) -> None:
        torch, nn = _lazy_import_torch()
        if torch is None or nn is None:
            self.loaded = False
            return
        try:
            ckpt = _torch_load_compat(torch, path)
            if not isinstance(ckpt, dict) or "model_state" not in ckpt:
                return

            input_dim = int(ckpt.get("input_dim", feature_dimension(self.patch_radius)))
            hidden_dims = [int(v) for v in ckpt.get("hidden_dims", list(self.hidden_dims))]
            model = _build_torch_mlp(nn, input_dim=input_dim, hidden_dims=hidden_dims)
            model.load_state_dict(ckpt["model_state"])
            self.torch_device = _choose_torch_device(torch, self.device_flag)
            model = model.to(self.torch_device)
            model.eval()

            self._torch = torch
            self._nn = nn
            self.torch_model = model

            fm = ckpt.get("feature_mean")
            fs = ckpt.get("feature_std")
            if fm is not None and fs is not None:
                self.torch_feature_mean = np.asarray(fm, dtype=np.float32).reshape(-1)
                self.torch_feature_std = np.asarray(fs, dtype=np.float32).reshape(-1)

            self.backend = "torch_mlp"
            self.expected_input_dim = input_dim
            self.loaded = True
        except Exception:
            self.loaded = False

    def _numeric_feature(self, pred_input: PredictorInput, x: float, y: float, heading_deg: float, velocity: tuple[float, float]) -> np.ndarray:
        goal_dx = 0.0
        goal_dy = 0.0
        if pred_input.goal is not None:
            goal_dx = float(pred_input.goal[0]) - x
            goal_dy = float(pred_input.goal[1]) - y

        goal_dist = math.hypot(goal_dx, goal_dy)
        if goal_dist > 1e-6:
            cos_goal = goal_dx / goal_dist
            sin_goal = goal_dy / goal_dist
        else:
            cos_goal = 0.0
            sin_goal = 0.0

        heading_rad = math.radians(heading_deg)

        local_obstacle_density = float(pred_input.local_context.get("local_obstacle_density", 0.0))
        teammate_distance = float(pred_input.local_context.get("teammate_distance", 10.0))
        teammate_velocity = tuple(pred_input.local_context.get("teammate_velocity", (0.0, 0.0)))

        max_speed = float(pred_input.local_context.get("max_speed_cells_per_step", 1.0))
        vx, vy = velocity
        speed = math.hypot(vx, vy)

        feat = np.array(
            [
                math.cos(heading_rad),
                math.sin(heading_rad),
                cos_goal,
                sin_goal,
                min(1.0, goal_dist / 20.0),
                min(1.0, speed / max(1e-6, max_speed)),
                float(vx),
                float(vy),
                min(1.0, local_obstacle_density),
                min(1.0, teammate_distance / 10.0),
                float(teammate_velocity[0]),
                float(teammate_velocity[1]),
            ],
            dtype=np.float32,
        )
        return feat

    def _build_feature_vector(self, pred_input: PredictorInput, x: float, y: float, heading_deg: float, velocity: tuple[float, float]) -> np.ndarray:
        numeric = self._numeric_feature(pred_input, x, y, heading_deg, velocity)
        patch_flat = np.asarray(pred_input.local_context.get("occupancy_patch_flat", []), dtype=np.float32)
        feat = np.concatenate([numeric, patch_flat], axis=0)

        if feat.shape[0] < self.expected_input_dim:
            pad = np.zeros((self.expected_input_dim - feat.shape[0],), dtype=np.float32)
            feat = np.concatenate([feat, pad], axis=0)
        elif feat.shape[0] > self.expected_input_dim:
            feat = feat[: self.expected_input_dim]

        return feat

    def _infer_residual(self, feat: np.ndarray) -> np.ndarray:
        if self.backend == "torch_mlp" and self.torch_model is not None and self._torch is not None:
            with self._torch.no_grad():
                x_np = feat.astype(np.float32)
                if (
                    self.torch_feature_mean is not None
                    and self.torch_feature_std is not None
                    and self.torch_feature_mean.shape[0] == x_np.shape[0]
                    and self.torch_feature_std.shape[0] == x_np.shape[0]
                ):
                    x_np = (x_np - self.torch_feature_mean) / np.maximum(self.torch_feature_std, 1e-6)
                x = self._torch.from_numpy(x_np).unsqueeze(0)
                if self.torch_device is not None:
                    x = x.to(self.torch_device)
                out = self.torch_model(x).squeeze(0).cpu().numpy().astype(np.float32)
            return np.tanh(out)

        # default linear residual head
        out = self.w @ feat + self.b
        return np.tanh(out.astype(np.float32))

    def _deterministic_feature_perturbation(self, feat: np.ndarray, sample_index: int) -> np.ndarray:
        if self.uncertainty_feature_noise_std <= 0.0:
            return feat
        idx = np.arange(feat.shape[0], dtype=np.float32)
        phase = float(sample_index + 1)
        noise = self.uncertainty_feature_noise_std * np.sin(idx * (0.173 * phase) + (0.379 * phase))
        return feat + noise.astype(np.float32)

    def _infer_residual_with_uncertainty(self, feat: np.ndarray) -> tuple[np.ndarray, float]:
        mean_residual = self._infer_residual(feat)
        if not self.enable_uncertainty or self.uncertainty_ensemble_samples <= 1:
            uncertainty = self.uncertainty_base
            if not self.loaded:
                uncertainty += 0.05
            return mean_residual, min(self.uncertainty_max, max(0.0, uncertainty))

        residuals = [mean_residual]
        for i in range(1, self.uncertainty_ensemble_samples):
            feat_i = self._deterministic_feature_perturbation(feat, sample_index=i)
            residuals.append(self._infer_residual(feat_i))

        arr = np.stack(residuals, axis=0).astype(np.float32)
        mean = np.mean(arr, axis=0)
        std_xy = np.std(arr, axis=0)
        sigma = float(np.sqrt(float(std_xy[0] ** 2 + std_xy[1] ** 2)))
        uncertainty = self.uncertainty_base + self.uncertainty_scale * sigma
        if not self.loaded:
            uncertainty += 0.05
        uncertainty = min(self.uncertainty_max, max(0.0, uncertainty))
        return np.tanh(mean), uncertainty

    def predict(self, pred_input: PredictorInput) -> PredictorOutput:
        t0 = time.perf_counter()
        base = self.fallback.predict(pred_input)

        if not self.enabled:
            t1 = time.perf_counter()
            return PredictorOutput(
                trajectory=base.trajectory,
                inference_time_sec=t1 - t0,
                uncertainty=[0.0 for _ in base.trajectory],
                debug={"model": self.name, "backend": "disabled", "loaded": self.loaded},
            )

        out: list[Pose2D] = []
        uncertainty_out: list[float] = []
        prev_x = float(pred_input.robot_state.pose[0])
        prev_y = float(pred_input.robot_state.pose[1])
        prev_heading = float(pred_input.robot_state.heading_deg)
        cur_vel = tuple(pred_input.local_context.get("robot_velocity", pred_input.robot_state.velocity))

        for p in base.trajectory:
            feat = self._build_feature_vector(pred_input, prev_x, prev_y, prev_heading, cur_vel)
            residual_mean, uncertainty = self._infer_residual_with_uncertainty(feat)
            residual = residual_mean * self.residual_scale

            base_dx = float(p.x - prev_x)
            base_dy = float(p.y - prev_y)
            x = prev_x + base_dx + float(residual[0])
            y = prev_y + base_dy + float(residual[1])

            dx = x - prev_x
            dy = y - prev_y
            heading = math.degrees(math.atan2(dy, dx)) if (abs(dx) > 1e-8 or abs(dy) > 1e-8) else prev_heading
            speed = math.hypot(dx, dy) / max(1e-6, pred_input.step_dt)
            out.append(Pose2D(x=x, y=y, heading_deg=heading, speed=speed))
            uncertainty_out.append(float(uncertainty))

            cur_vel = (dx / max(1e-6, pred_input.step_dt), dy / max(1e-6, pred_input.step_dt))
            prev_x, prev_y, prev_heading = x, y, heading

        t1 = time.perf_counter()
        unc_mean = float(sum(uncertainty_out) / len(uncertainty_out)) if uncertainty_out else 0.0
        unc_max = float(max(uncertainty_out)) if uncertainty_out else 0.0
        return PredictorOutput(
            trajectory=out,
            inference_time_sec=t1 - t0,
            uncertainty=uncertainty_out,
            debug={
                "model": self.name,
                "backend": self.backend,
                "loaded": self.loaded,
                "weight_file": self.weight_file,
                "feature_dim": self.expected_input_dim,
                "device": str(self.torch_device) if self.torch_device is not None else "cpu",
                "uncertainty_enabled": self.enable_uncertainty,
                "uncertainty_samples": self.uncertainty_ensemble_samples,
                "uncertainty_mean": unc_mean,
                "uncertainty_max": unc_max,
            },
        )
