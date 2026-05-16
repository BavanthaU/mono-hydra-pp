#!/usr/bin/env python3
"""Standalone loader for the large NYUD/ITC/ScanNet models bundled in this package."""
from __future__ import annotations

import sys
from contextlib import nullcontext
from pathlib import Path
from typing import Dict, Tuple, Iterable

import numpy as np
from PIL import Image
import torch

try:
    import rospkg
except ImportError:  # pragma: no cover - optional when running outside ROS
    rospkg = None  # type: ignore
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - optional outside ROS 2
    get_package_share_directory = None  # type: ignore[assignment]

from m2h_hmx_large.config import load_config  # noqa: E402
from m2h_hmx_large.models.m2h_hmx_v3 import HMXV3ModelConfig, build_model  # noqa: E402


def _resolve_pkg_root() -> Path:
    if get_package_share_directory is not None:
        try:
            return Path(get_package_share_directory("mono_hydra_perception"))
        except Exception:
            pass
    if rospkg is not None:
        try:
            return Path(rospkg.RosPack().get_path("m2h_hmx_large"))
        except Exception:
            pass
    script_path = Path(__file__).resolve()
    for candidate in [script_path.parent, *script_path.parents]:
        if (candidate / "config").exists() and (candidate / "weights").exists():
            return candidate
    return script_path.parents[1]


PKG_ROOT = _resolve_pkg_root()
NYUD_CONFIG = PKG_ROOT / "config" / "m2h_hmx_v3_nyudv2_large.yml"
NYUD_CHECKPOINT = PKG_ROOT / "weights" / "nyudv2_large__miou_0.656_rmse_0.380_weights.pt"
ITC_CONFIG = PKG_ROOT / "config" / "m2h_hmx_v3_1_large_itc_mt_hr.yml"
ITC_CHECKPOINT = PKG_ROOT / "weights" / "itc_large__miou_0.393_rmse_0.523_weights.pt"
SCANNET_CONFIG = PKG_ROOT / "config" / "m2h_hmx_v3_1_large_scannet_ft.yml"
SCANNET_CHECKPOINT = PKG_ROOT / "weights" / "scannet_large__miou_0.761_rmse_0.221_weights.pt"
DATASET_DEFAULTS: Dict[str, Tuple[Path, Path]] = {
    "nyud": (NYUD_CONFIG, NYUD_CHECKPOINT),
    "itc": (ITC_CONFIG, ITC_CHECKPOINT),
    "scannet": (SCANNET_CONFIG, SCANNET_CHECKPOINT),
}
DATASET_TASKS: Dict[str, Tuple[str, ...]] = {
    "nyud": ("semseg", "depth", "normals", "edge"),
    "itc": ("semseg", "depth"),
    "scannet": ("semseg", "depth"),
}
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def _pascal_palette(num_classes: int) -> np.ndarray:
    palette = np.zeros((num_classes, 3), dtype=np.uint8)
    for j in range(num_classes):
        lab = j
        for i in range(8):
            palette[j, 0] |= (((lab >> 0) & 1) << (7 - i))
            palette[j, 1] |= (((lab >> 1) & 1) << (7 - i))
            palette[j, 2] |= (((lab >> 2) & 1) << (7 - i))
            lab >>= 3
    return palette


def _colorize_semseg(mask: np.ndarray, num_classes: int) -> np.ndarray:
    palette = _pascal_palette(max(num_classes, int(mask.max()) + 1))
    colored = palette[mask % len(palette)]
    return colored.astype(np.uint8)


def _colorize_depth(depth: np.ndarray, min_depth: float, max_depth: float) -> np.ndarray:
    finite = np.nan_to_num(depth, nan=min_depth, posinf=max_depth, neginf=min_depth)
    norm = (finite - min_depth) / max(max_depth - min_depth, 1e-6)
    norm = np.clip(norm, 0.0, 1.0)
    cmap = np.stack([norm, np.sqrt(norm), 1.0 - norm], axis=-1)
    return (cmap * 255.0).astype(np.uint8)


def _load_image_tensor(path: Path, size: Tuple[int, int]) -> torch.Tensor:
    from PIL import Image

    image = Image.open(path).convert("RGB")
    resized = image.resize((size[1], size[0]), Image.BILINEAR)
    array = np.array(resized, dtype=np.float32) / 255.0
    array = (array - MEAN) / STD
    tensor = torch.from_numpy(array).permute(2, 0, 1).unsqueeze(0)
    return tensor


class M2HHMXV3Loader:
    """Utility that owns the model + checkpoint for downstream ROS nodes."""

    def __init__(
        self,
        config_path: Path,
        checkpoint: Path,
        device: str = "cuda",
        half: bool = False,
        tasks: Iterable[str] | None = None,
    ) -> None:
        self.config_path = Path(config_path)
        self.checkpoint_path = Path(checkpoint)
        self.device = torch.device(device)
        self.cfg = load_config(self.config_path)
        self.model_cfg = self._build_model_cfg(self.cfg, tasks)
        self.model = build_model(self.model_cfg).to(self.device)
        self.model.eval()
        self._load_weights(self.checkpoint_path)
        self.image_size = self.cfg.dataset.image_size
        self.num_classes = self.cfg.dataset.num_classes
        self.min_depth = self.cfg.model.min_depth
        self.max_depth = self.cfg.model.max_depth
        self._autocast = bool(half and self.device.type == "cuda")
        self._dtype = torch.float16 if self._autocast else torch.float32

    def _build_model_cfg(self, exp_cfg, tasks_override: Iterable[str] | None = None) -> HMXV3ModelConfig:
        hmx_cfg = exp_cfg.hmx
        tasks = tuple(tasks_override) if tasks_override is not None else tuple(getattr(exp_cfg, "tasks", ()))
        if not tasks:
            tasks = ("semseg", "depth", "normals", "edge")
        return HMXV3ModelConfig(
            backbone_name=hmx_cfg.backbone_name,
            decoder_dim=hmx_cfg.decoder_dim,
            num_seg_classes=exp_cfg.model.num_classes,
            ltc_window_size=hmx_cfg.ltc_window_size,
            min_depth=exp_cfg.model.min_depth,
            max_depth=exp_cfg.model.max_depth,
            depth_bins=hmx_cfg.depth_bins,
            hm_d_state=hmx_cfg.hm_d_state,
            hm_drop_path=hmx_cfg.hm_drop_path,
            gtf_extra_levels=hmx_cfg.gtf_extra_levels,
            train_last_n_blocks=hmx_cfg.train_last_n_blocks,
            intermediate_layer_indices=hmx_cfg.intermediate_layer_indices,
            num_register_tokens=hmx_cfg.num_register_tokens,
            use_lora=hmx_cfg.use_lora,
            lora_rank=hmx_cfg.lora_rank,
            lora_alpha=hmx_cfg.lora_alpha,
            lora_dropout=hmx_cfg.lora_dropout,
            edge_refine_threshold=hmx_cfg.edge_refine_threshold,
            edge_refine_strength=hmx_cfg.edge_refine_strength,
            tasks=tasks,
        )

    def _load_weights(self, checkpoint: Path) -> None:
        if not checkpoint.exists():
            raise FileNotFoundError(f"Checkpoint '{checkpoint}' not found.")
        state_dict = torch.load(checkpoint, map_location=self.device)
        if isinstance(state_dict, dict) and "model" in state_dict and not any(k.startswith("module.") for k in state_dict):
            state_dict = state_dict["model"]
        if isinstance(state_dict, dict) and "state_dict" in state_dict:
            state_dict = state_dict["state_dict"]
        cleaned = {}
        if isinstance(state_dict, dict):
            for key, value in state_dict.items():
                new_key = key[7:] if key.startswith("module.") else key
                cleaned[new_key] = value
            state_dict = cleaned
        if isinstance(state_dict, dict):
            for drop_key in [k for k in list(state_dict.keys()) if k.startswith("loss_balancer.")]:
                state_dict.pop(drop_key, None)
        if isinstance(state_dict, dict):
            model_keys = set(self.model.state_dict().keys())
            pruned = [k for k in list(state_dict.keys()) if k not in model_keys]
            for k in pruned:
                state_dict.pop(k, None)
        missing, unexpected = self.model.load_state_dict(state_dict, strict=False)
        if missing:
            print(f"[m2h_hmx_large_loader] Missing params when loading {checkpoint.name}: {missing}")
        if unexpected:
            print(f"[m2h_hmx_large_loader] Unexpected params when loading {checkpoint.name}: {unexpected}")
        # Sanity-check class count to catch config/weight mismatches early.
        seg_w = self.model.segmentation_head.pred_conv.conv2.weight
        ckpt_classes = seg_w.shape[0]
        if ckpt_classes != self.model.cfg.num_seg_classes:
            raise ValueError(
                f"Checkpoint classes ({ckpt_classes}) do not match config num_seg_classes ({self.model.cfg.num_seg_classes}). "
                f"Config path: {self.config_path}, checkpoint: {checkpoint}"
            )

    def predict(self, batch: torch.Tensor | Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        if isinstance(batch, torch.Tensor):
            batch = {"images": batch}
        batch.setdefault("compute_edges", True)
        batch.setdefault("compute_normals", True)
        autocast_ctx = torch.amp.autocast("cuda", enabled=self._autocast) if self.device.type == "cuda" else nullcontext()
        with torch.no_grad(), autocast_ctx:
            raw_outputs = self.model(batch)
        outputs = raw_outputs.get("pred") if isinstance(raw_outputs, dict) else raw_outputs
        if isinstance(outputs, dict) and "pred" in outputs:
            outputs = outputs["pred"]
        return outputs


__all__ = [
    "DATASET_DEFAULTS",
    "MEAN",
    "STD",
    "M2HHMXV3Loader",
    "_colorize_depth",
    "_colorize_semseg",
]
