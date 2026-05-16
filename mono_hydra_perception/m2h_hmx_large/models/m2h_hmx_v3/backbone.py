import warnings
from typing import Dict, List, Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from transformers.modeling_outputs import BaseModelOutputWithPooling
from transformers import AutoConfig, AutoModel

try:
    from transformers.models.dinov3_vit import DINOv3ViTConfig, DINOv3ViTModel
except ImportError:  # pragma: no cover - only relevant with older Transformers builds
    DINOv3ViTConfig = None  # type: ignore[assignment]
    DINOv3ViTModel = None  # type: ignore[assignment]

from ..lora_utils import configure_backbone_trainables, inject_lora_adapters


_LOCAL_DINOV3_VITL16_NAMES = {
    "facebook/dinov3-vitl16-pretrain-lvd1689m",
}


class _LocalDINOv3ViTL16Model(nn.Module):
    """DINOv3 ViT-L/16 instantiated locally for bundled M2H checkpoints.

    The public DINOv3 ViT-L/16 Hugging Face repository is gated. The M2H HMX
    Large checkpoints in this package already contain the backbone weights, so
    runtime should not need network access or Hugging Face authentication just
    to construct the architecture. This wrapper keeps the checkpoint parameter
    names used by the bundled weights: ``embeddings.*``, ``layer.*``, ``norm.*``.
    """

    def __init__(self, config: DINOv3ViTConfig) -> None:  # type: ignore[valid-type]
        super().__init__()
        reference_model = DINOv3ViTModel(config)
        self.config = config
        self.embeddings = reference_model.embeddings
        self.rope_embeddings = reference_model.rope_embeddings
        self.layer = reference_model.model.layer
        self.norm = reference_model.norm

    def forward(
        self,
        pixel_values: torch.Tensor,
        bool_masked_pos: torch.Tensor | None = None,
        output_hidden_states: bool | None = None,
        return_dict: bool | None = None,
        **kwargs,
    ) -> BaseModelOutputWithPooling | tuple[torch.Tensor, ...]:
        output_hidden_states = bool(output_hidden_states)
        return_dict = True if return_dict is None else bool(return_dict)
        pixel_values = pixel_values.to(self.embeddings.patch_embeddings.weight.dtype)
        hidden_states = self.embeddings(pixel_values, bool_masked_pos=bool_masked_pos)
        position_embeddings = self.rope_embeddings(pixel_values)
        all_hidden_states = [hidden_states] if output_hidden_states else None

        kwargs.pop("output_attentions", None)
        for layer_module in self.layer:
            hidden_states = layer_module(hidden_states, position_embeddings=position_embeddings, **kwargs)
            if all_hidden_states is not None:
                all_hidden_states.append(hidden_states)

        sequence_output = self.norm(hidden_states)
        pooled_output = sequence_output[:, 0, :]
        if not return_dict:
            values: tuple[torch.Tensor, ...] = (sequence_output, pooled_output)
            if all_hidden_states is not None:
                values = values + (tuple(all_hidden_states),)
            return values
        return BaseModelOutputWithPooling(
            last_hidden_state=sequence_output,
            pooler_output=pooled_output,
            hidden_states=tuple(all_hidden_states) if all_hidden_states is not None else None,
            attentions=None,
        )


def _build_local_dinov3_vitl16_config() -> DINOv3ViTConfig:  # type: ignore[valid-type]
    if DINOv3ViTConfig is None:
        raise ImportError("Transformers DINOv3 ViT support is not available.")
    return DINOv3ViTConfig(
        patch_size=16,
        hidden_size=1024,
        intermediate_size=4096,
        num_hidden_layers=24,
        num_attention_heads=16,
        image_size=640,
        num_channels=3,
        query_bias=True,
        key_bias=False,
        value_bias=True,
        proj_bias=True,
        mlp_bias=True,
        num_register_tokens=4,
        layerscale_value=1.0,
        drop_path_rate=0.0,
        use_gated_mlp=False,
    )


def _is_local_dinov3_vitl16(model_name: str) -> bool:
    return model_name in _LOCAL_DINOV3_VITL16_NAMES or model_name.endswith("/dinov3-vitl16-pretrain-lvd1689m")


def _make_backbone(model_name: str) -> tuple[nn.Module, object]:
    if _is_local_dinov3_vitl16(model_name):
        config = _build_local_dinov3_vitl16_config()
        return _LocalDINOv3ViTL16Model(config), config

    config = AutoConfig.from_pretrained(model_name)
    return AutoModel.from_pretrained(model_name), config


class HFA_Adapter_V3(nn.Module):
    """DINOv3 adapter that exposes register tokens and supports LoRA fine-tuning."""

    def __init__(
        self,
        model_name: str,
        decoder_dim: int = 256,
        intermediate_layer_indices: List[int] | None = None,
        train_last_n_blocks: int = 1,
        *,
        use_lora: bool = True,
        lora_rank: int = 16,
        lora_alpha: float = 16.0,
        lora_dropout: float = 0.05,
        num_register_tokens: int | None = None,
    ) -> None:
        super().__init__()
        self.backbone, self.config = _make_backbone(model_name)
        total_layers = int(getattr(self.config, "num_hidden_layers", 12))
        if intermediate_layer_indices is None:
            intermediate_layer_indices = [max(0, total_layers // 4 - 1), total_layers // 2, total_layers - 3, total_layers - 1]
        indices = sorted(set(int(idx) for idx in intermediate_layer_indices if 0 <= int(idx) < total_layers))
        if not indices:
            warnings.warn(
                "No valid intermediate_layer_indices provided; falling back to evenly spaced layers.",
                stacklevel=2,
            )
            indices = [max(0, total_layers // 4 - 1), total_layers // 2, total_layers - 3, total_layers - 1]

        self.layer_indices = indices
        self.num_scales = len(self.layer_indices)
        self.decoder_dim = decoder_dim

        self.use_lora = bool(use_lora)
        self.train_last_n_blocks = max(int(train_last_n_blocks), 0)

        if self.use_lora:
            target_modules = (
                "query",
                "key",
                "value",
                "q_proj",
                "k_proj",
                "v_proj",
                "fc1",
                "fc2",
            )
            updated = inject_lora_adapters(
                self.backbone,
                target_modules=target_modules,
                rank=int(lora_rank),
                alpha=float(lora_alpha),
                dropout=float(lora_dropout),
                apply_to_layers=self.train_last_n_blocks,
            )
            if updated == 0:
                warnings.warn(
                    "LoRA adaptation requested for DINOv3 but no layers matched the target modules.", stacklevel=2
                )
            configure_backbone_trainables(
                self.backbone,
                train_norms=True,
                norm_patterns=("ln", "norm"),
                extra_patterns=("lora_",),
            )
        else:
            for param in self.backbone.parameters():
                param.requires_grad = False
            if self.train_last_n_blocks > 0:
                self._unfreeze_last_blocks(self.train_last_n_blocks)

        self.enable_backbone_grad = self.use_lora or self.train_last_n_blocks > 0
        self.patch_size = getattr(self.config, "patch_size", 14)
        cfg_register_tokens = int(getattr(self.config, "num_register_tokens", 0))
        if cfg_register_tokens == 0 and num_register_tokens is not None:
            cfg_register_tokens = int(num_register_tokens)
        self.num_register_tokens = cfg_register_tokens
        self.num_special_tokens = 1 + self.num_register_tokens

        norm = lambda c: nn.GroupNorm(1, c)

        self.projection_layers = nn.ModuleList(
            [
                nn.Sequential(nn.Conv2d(self.config.hidden_size, decoder_dim, kernel_size=1), norm(decoder_dim))
                for _ in self.layer_indices
            ]
        )
        self.fusion_layers = nn.ModuleList(
            [
                nn.Sequential(nn.Conv2d(decoder_dim, decoder_dim, kernel_size=3, padding=1), norm(decoder_dim))
                for _ in self.layer_indices
            ]
        )
        refinement_blocks = max(self.num_scales - 1, 1)
        self.refinement_layers = nn.ModuleList(
            [
                nn.Sequential(nn.Conv2d(decoder_dim, decoder_dim, kernel_size=3, padding=1), norm(decoder_dim))
                for _ in range(refinement_blocks)
            ]
        )
        self.register_proj = nn.Linear(self.config.hidden_size, decoder_dim)

    def _find_transformer_blocks(self) -> List[nn.Module]:
        candidates = []
        if hasattr(self.backbone, "encoder"):
            candidates.append(self.backbone.encoder)
        candidates.append(self.backbone)
        for module in candidates:
            for attr in ("layers", "layer", "blocks", "block"):
                blocks = getattr(module, attr, None)
                if isinstance(blocks, (nn.ModuleList, list, tuple)):
                    return list(blocks)
        return []

    def _unfreeze_last_blocks(self, num_blocks: int) -> None:
        blocks = self._find_transformer_blocks()
        if not blocks:
            return
        for block in blocks[-num_blocks:]:
            for param in block.parameters():
                param.requires_grad = True

    def _reassemble_tokens(self, tokens: torch.Tensor, patch_res: Tuple[int, int]) -> torch.Tensor:
        B, N, C = tokens.shape
        H, W = patch_res
        if N != H * W:
            # fall back to square reshaping if resolution metadata is unreliable
            side = int(N**0.5)
            H = W = side
        return tokens.permute(0, 2, 1).reshape(B, C, H, W)

    def _build_feature_pyramid(self, projected_features: List[torch.Tensor]) -> Dict[str, torch.Tensor]:
        fused_features = []
        fused_p = self.fusion_layers[-1](projected_features[-1])
        fused_features.append(fused_p)

        for i in range(self.num_scales - 2, -1, -1):
            upsampled_map = F.interpolate(
                fused_features[-1], size=projected_features[i].shape[-2:], mode="bilinear", align_corners=False
            )
            fused_map = self.fusion_layers[i](projected_features[i] + upsampled_map)
            fused_features.append(fused_map)

        fused_features.reverse()

        p4 = fused_features[-1]
        pooled = F.max_pool2d(p4, kernel_size=2, stride=2)
        p5 = self.refinement_layers[-1](pooled)

        p3 = self.refinement_layers[0](F.interpolate(p4, scale_factor=2, mode="bilinear", align_corners=False))
        p2 = self.refinement_layers[1](F.interpolate(p3, scale_factor=2, mode="bilinear", align_corners=False)) if len(self.refinement_layers) > 1 else p3

        return {"p2": p2, "p3": p3, "p4": p4, "p5": p5}

    def forward(self, image: torch.Tensor) -> Tuple[Dict[str, torch.Tensor], torch.Tensor]:
        grad_ctx = torch.enable_grad if self.enable_backbone_grad else torch.no_grad
        with grad_ctx():
            outputs = self.backbone(image, output_hidden_states=True, return_dict=True)

        hidden_states = outputs.hidden_states[1:]
        patch_res_h = max(1, image.shape[2] // self.patch_size)
        patch_res_w = max(1, image.shape[3] // self.patch_size)
        patch_res = (patch_res_h, patch_res_w)

        projected_features: List[torch.Tensor] = []
        register_vec = torch.zeros(
            image.shape[0], self.decoder_dim, device=image.device, dtype=image.dtype, requires_grad=self.enable_backbone_grad
        )

        for i, layer_idx in enumerate(self.layer_indices):
            if layer_idx >= len(hidden_states):
                raise IndexError(
                    f"Layer index {layer_idx} is out of range for backbone with {len(hidden_states)} hidden states."
                )
            layer_output = hidden_states[layer_idx]
            if self.num_register_tokens > 0 and register_vec is not None and layer_idx == self.layer_indices[-1]:
                start = 1
                end = min(start + self.num_register_tokens, layer_output.shape[1])
                register_tokens = layer_output[:, start:end, :]
                register_vec = self.register_proj(register_tokens.mean(dim=1))
            patch_tokens = layer_output[:, self.num_special_tokens :, :]
            feature_map = self._reassemble_tokens(patch_tokens, patch_res)
            projected_features.append(self.projection_layers[i](feature_map))

        pyramid = self._build_feature_pyramid(projected_features)
        return pyramid, register_vec


# Backwards compatibility alias
HFA_Adapter_V2 = HFA_Adapter_V3
