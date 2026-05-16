from typing import Dict, List

import torch
import torch.nn as nn
import torch.nn.functional as F
from einops import rearrange

from mamba_ssm import Mamba
from timm.layers import DropPath


class RegisterGatedMambaBlock(nn.Module):
    """Injects global register context into spatial tokens before a Mamba scan."""

    def __init__(self, dim: int, d_state: int = 32, drop_path: float = 0.1) -> None:
        super().__init__()
        self.mamba = Mamba(d_model=dim, d_state=d_state)
        self.norm = nn.LayerNorm(dim)
        self.ffn = nn.Sequential(
            nn.LayerNorm(dim),
            nn.Linear(dim, dim * 2),
            nn.GELU(),
            nn.Linear(dim * 2, dim),
        )
        gate_hidden = max(dim // 4, 16)
        self.register_gate = nn.Sequential(
            nn.Linear(dim, gate_hidden),
            nn.SiLU(),
            nn.Linear(gate_hidden, dim),
            nn.Sigmoid(),
        )
        self.drop_path = DropPath(drop_path) if drop_path > 0 else nn.Identity()

    def forward(self, x: torch.Tensor, registers: torch.Tensor | None) -> torch.Tensor:
        B, C, H, W = x.shape
        if registers is None:
            registers = torch.zeros(B, C, device=x.device, dtype=x.dtype)
        elif registers.shape[-1] != C:
            registers = F.interpolate(registers.unsqueeze(-1).unsqueeze(-1), size=(1, 1)).view(B, C)
        gate = self.register_gate(registers).unsqueeze(1)
        seq = rearrange(x, "b c h w -> b (h w) c")
        gated = seq * gate
        mamba_out = self.mamba(self.norm(gated))
        seq = seq + self.drop_path(mamba_out)
        seq = seq + self.drop_path(self.ffn(self.norm(seq)))
        return rearrange(seq, "b (h w) c -> b c h w", h=H, w=W)


class HMX_Decoder(nn.Module):
    """Register-gated multi-task decoder built around Mamba blocks."""

    def __init__(
        self,
        num_scales: int,
        decoder_dim: int,
        num_tasks: int,
        ltc_window_size: int = 4,  # kept for config compatibility
        hm_d_state: int = 32,
        hm_drop_path: float = 0.1,
        gtf_extra_levels: int = 0,
    ) -> None:
        super().__init__()
        self.num_tasks = num_tasks
        self.num_scales = num_scales
        self.decoder_dim = decoder_dim

        self.register_blocks = nn.ModuleList(
            [RegisterGatedMambaBlock(decoder_dim, d_state=hm_d_state, drop_path=hm_drop_path) for _ in range(num_scales)]
        )

        block = lambda: nn.Sequential(
            nn.Conv2d(decoder_dim, decoder_dim, kernel_size=3, padding=1, bias=False),
            nn.GroupNorm(1, decoder_dim),
            nn.GELU(),
        )

        self.task_branches = nn.ModuleList(
            [
                nn.ModuleList([block() for _ in range(num_tasks)])
                for _ in range(num_scales)
            ]
        )

    def forward(self, feature_pyramid: Dict[str, torch.Tensor], registers: torch.Tensor | None) -> Dict[str, List[torch.Tensor]]:
        pyramid_levels = [feature_pyramid[k] for k in sorted(feature_pyramid.keys())]
        task_features_per_scale = {f"task_{t}": [] for t in range(self.num_tasks)}

        prev_shared = None
        prev_task_features = [None for _ in range(self.num_tasks)]

        for scale_idx, p_level_feat in enumerate(reversed(pyramid_levels)):
            shared = p_level_feat
            if prev_shared is not None:
                shared = shared + F.interpolate(prev_shared, size=p_level_feat.shape[-2:], mode="bilinear", align_corners=False)
            shared = self.register_blocks[scale_idx](shared, registers)

            for task_idx in range(self.num_tasks):
                branch = self.task_branches[scale_idx][task_idx](shared)
                prev_task = prev_task_features[task_idx]
                if prev_task is not None:
                    branch = branch + F.interpolate(prev_task, size=branch.shape[-2:], mode="bilinear", align_corners=False)
                task_features_per_scale[f"task_{task_idx}"].append(branch)
                prev_task_features[task_idx] = branch

            prev_shared = shared

        for t in range(self.num_tasks):
            task_features_per_scale[f"task_{t}"].reverse()

        return task_features_per_scale
