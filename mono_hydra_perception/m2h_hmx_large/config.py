"""Minimal YAML config loader for the standalone ROS package (inference only)."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Tuple

import yaml


def _to_tuple(value: Any) -> Tuple[int, int]:
    if isinstance(value, (list, tuple)) and len(value) == 2:
        return int(value[0]), int(value[1])
    val = int(value)
    return (val, val)


@dataclass
class DatasetSettings:
    image_size: Tuple[int, int] = (480, 640)
    num_classes: int = 40
    min_depth: float = 0.1
    max_depth: float = 10.0


@dataclass
class ModelSettings:
    num_classes: int = 40
    min_depth: float = 0.1
    max_depth: float = 10.0


@dataclass
class HMXSettings:
    backbone_name: str = "facebook/dinov3-vitb16-pretrain-lvd1689m"
    decoder_dim: int = 256
    ltc_window_size: int = 4
    hm_d_state: int = 32
    hm_drop_path: float = 0.1
    gtf_extra_levels: int = 0
    train_last_n_blocks: int = 2
    intermediate_layer_indices: Optional[Tuple[int, ...]] = None
    num_register_tokens: int = 4
    use_lora: bool = True
    lora_rank: int = 16
    lora_alpha: float = 32
    lora_dropout: float = 0.05
    depth_bins: int = 64
    edge_refine_threshold: Optional[float] = None
    edge_refine_strength: float = 0.1


@dataclass
class ExperimentSettings:
    dataset: DatasetSettings
    model: ModelSettings
    hmx: HMXSettings


def load_config(path: str | Path) -> ExperimentSettings:
    cfg_path = Path(path)
    raw = yaml.safe_load(cfg_path.read_text()) or {}

    dataset_raw = raw.get("dataset", {})
    dataset = DatasetSettings(
        image_size=_to_tuple(dataset_raw.get("image_size", (480, 640))),
        num_classes=int(dataset_raw.get("num_classes", 40)),
        min_depth=float(dataset_raw.get("min_depth", 0.1)),
        max_depth=float(dataset_raw.get("max_depth", 10.0)),
    )

    model_raw = raw.get("model", {})
    model = ModelSettings(
        num_classes=int(model_raw.get("num_classes", dataset.num_classes)),
        min_depth=float(model_raw.get("min_depth", dataset.min_depth)),
        max_depth=float(model_raw.get("max_depth", dataset.max_depth)),
    )

    hmx_raw = raw.get("hmx", {})
    exp_hmx = HMXSettings(
        backbone_name=str(hmx_raw.get("backbone_name", "facebook/dinov3-vitb16-pretrain-lvd1689m")),
        decoder_dim=int(hmx_raw.get("decoder_dim", 256)),
        ltc_window_size=int(hmx_raw.get("ltc_window_size", 4)),
        hm_d_state=int(hmx_raw.get("hm_d_state", 32)),
        hm_drop_path=float(hmx_raw.get("hm_drop_path", 0.1)),
        gtf_extra_levels=int(hmx_raw.get("gtf_extra_levels", 0)),
        train_last_n_blocks=int(hmx_raw.get("train_last_n_blocks", 2)),
        intermediate_layer_indices=tuple(hmx_raw.get("intermediate_layer_indices", [])) or None,
        num_register_tokens=int(hmx_raw.get("num_register_tokens", 4)),
        use_lora=bool(hmx_raw.get("use_lora", True)),
        lora_rank=int(hmx_raw.get("lora_rank", 16)),
        lora_alpha=float(hmx_raw.get("lora_alpha", 32.0)),
        lora_dropout=float(hmx_raw.get("lora_dropout", 0.05)),
        depth_bins=int(hmx_raw.get("depth_bins", 64)),
        edge_refine_threshold=hmx_raw.get("edge_refine_threshold"),
        edge_refine_strength=float(hmx_raw.get("edge_refine_strength", 0.1)),
    )

    return ExperimentSettings(dataset=dataset, model=model, hmx=exp_hmx)
