"""LoRA utilities shared across models."""
from __future__ import annotations

from typing import Optional, Sequence

import torch
import torch.nn as nn
import torch.nn.functional as F


class LoRALinear(nn.Module):
    """Wraps a Linear layer with a trainable low-rank adapter."""

    def __init__(self, linear: nn.Linear, rank: int, alpha: float, dropout: float) -> None:
        super().__init__()
        if rank < 0:
            raise ValueError("LoRA rank must be non-negative.")
        self.linear = linear
        self.rank = rank
        self.scaling = alpha / max(rank, 1)
        self.dropout = nn.Dropout(dropout) if dropout > 0 else nn.Identity()
        if rank > 0:
            self.lora_A = nn.Parameter(torch.zeros(rank, linear.in_features))
            self.lora_B = nn.Parameter(torch.zeros(linear.out_features, rank))
            nn.init.kaiming_uniform_(self.lora_A, a=5 ** 0.5)
            nn.init.zeros_(self.lora_B)
        else:
            self.register_parameter("lora_A", None)
            self.register_parameter("lora_B", None)
        # Freeze the base weights so gradients only flow into the adapter.
        self.linear.weight.requires_grad = False
        if self.linear.bias is not None:
            self.linear.bias.requires_grad = False

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        base = self.linear(x)
        if self.rank == 0:
            return base
        update = F.linear(self.dropout(x), self.lora_A)
        update = F.linear(update, self.lora_B)
        return base + self.scaling * update


def _resolve_parent(module: nn.Module, path: str) -> tuple[nn.Module, str]:
    parts = path.split(".")
    parent = module
    for part in parts[:-1]:
        parent = getattr(parent, part)
    return parent, parts[-1]


def _block_index_from_name(name: str) -> Optional[int]:
    parts = name.split(".")
    for idx, part in enumerate(parts[:-1]):
        if part == "blocks":
            try:
                return int(parts[idx + 1])
            except (IndexError, ValueError):
                return None
    return None


def inject_lora_adapters(
    model: nn.Module,
    *,
    target_modules: Sequence[str],
    rank: int,
    alpha: float,
    dropout: float,
    apply_to_layers: int,
) -> int:
    """Replace matching Linear layers with LoRALinear wrappers.

    Returns:
        Number of layers updated.
    """
    total_layers = getattr(getattr(model, "config", None), "num_hidden_layers", None)
    min_block = None
    if total_layers is not None and apply_to_layers > 0:
        min_block = max(total_layers - apply_to_layers, 0)

    replacements = 0
    for name, module in list(model.named_modules()):
        if not isinstance(module, nn.Linear):
            continue
        if not any(target in name for target in target_modules):
            continue
        block_idx = _block_index_from_name(name)
        if min_block is not None and block_idx is not None and block_idx < min_block:
            continue
        parent, attr = _resolve_parent(model, name)
        setattr(parent, attr, LoRALinear(module, rank=rank, alpha=alpha, dropout=dropout))
        replacements += 1
    return replacements


def _matches(name: str, patterns: Sequence[str]) -> bool:
    return any(pattern in name for pattern in patterns)


def configure_backbone_trainables(
    module: nn.Module,
    *,
    train_norms: bool,
    norm_patterns: Sequence[str],
    extra_patterns: Sequence[str],
) -> None:
    """Freeze backbone weights except LoRA (and optional norm/extra) parameters."""
    for name, param in module.named_parameters():
        if "lora_" in name:
            param.requires_grad = True
        elif train_norms and _matches(name, norm_patterns):
            param.requires_grad = True
        elif extra_patterns and _matches(name, extra_patterns):
            param.requires_grad = True
        else:
            param.requires_grad = False
