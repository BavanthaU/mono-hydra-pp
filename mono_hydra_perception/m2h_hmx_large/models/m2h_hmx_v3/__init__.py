from __future__ import annotations

from .model import DINO_HMX_V3, HMXV3ModelConfig


def build_model(cfg: HMXV3ModelConfig) -> DINO_HMX_V3:
    """Instantiate the register-aware M2H-HMX v3 model."""
    return DINO_HMX_V3(cfg=cfg, num_seg_classes=cfg.num_seg_classes)
