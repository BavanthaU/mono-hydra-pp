import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Dict, List, Tuple


def _group_norm(channels: int, max_groups: int = 32) -> nn.GroupNorm:
    groups = min(max_groups, channels)
    while groups > 1 and channels % groups != 0:
        groups -= 1
    return nn.GroupNorm(groups, channels)


class FusionBlock(nn.Module):
    """Deeper fusion block with GroupNorm residual connection."""

    def __init__(self, dim):
        super().__init__()
        self.conv1 = nn.Conv2d(dim, dim, kernel_size=3, padding=1, bias=False)
        self.norm1 = _group_norm(dim)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(dim, dim, kernel_size=3, padding=1, bias=False)
        self.norm2 = _group_norm(dim)

    def forward(self, x):
        identity = x
        out = self.relu(self.norm1(self.conv1(x)))
        out = self.norm2(self.conv2(out))
        out += identity
        return self.relu(out)


class PredictionMLP(nn.Module):
    """Small MLP for prediction refinement with optional dynamic weights."""

    def __init__(self, in_dim, out_dim):
        super().__init__()
        self.conv1 = nn.Conv2d(in_dim, in_dim, kernel_size=3, padding=1)
        self.gelu = nn.GELU()
        self.conv2 = nn.Conv2d(in_dim, out_dim, kernel_size=1)

    def forward(self, x: torch.Tensor, dynamic_weights: torch.Tensor | None = None) -> torch.Tensor:
        x = self.gelu(self.conv1(x))
        if dynamic_weights is not None:
            # Apply per-sample 1x1 conv weights produced by a hypernetwork.
            B, C_in, H, W = x.shape
            C_out = dynamic_weights.shape[1]
            x_reshaped = x.reshape(1, B * C_in, H, W)
            w_reshaped = dynamic_weights.reshape(B * C_out, C_in, 1, 1)
            out = F.conv2d(x_reshaped, w_reshaped, groups=B)
            return out.reshape(B, C_out, H, W)
        return self.conv2(x)


class MSCABlock(nn.Module):
    """
    Multi-Scale Convolutional Attention (MSCA) block (SegNeXt/VAN style).

    Preserves spatial shape (B, C, H, W). Builds an attention map from depthwise
    local + strip convolutions, mixes with 1x1, and gates the identity path.
    """

    def __init__(
        self,
        channels: int,
        strip_kernel_size: int = 7,
        use_proj: bool = False,
        se_inner: bool = True,
    ) -> None:
        """
        Args:
            channels: Number of input/output channels.
            strip_kernel_size: Odd kernel for strip convs (e.g., 7 or 11).
            use_proj: If True, apply a 1x1 projection before attention; else use X directly.
            se_inner: If True, apply a lightweight SE-style channel gate inside attention.
        """
        super().__init__()
        k = max(3, strip_kernel_size)
        if k % 2 == 0:
            raise ValueError("strip_kernel_size must be odd to preserve spatial size.")

        norm = lambda c: nn.GroupNorm(1, c)

        self.identity = (
            nn.Sequential(nn.Conv2d(channels, channels, 1, bias=False), norm(channels))
            if use_proj
            else nn.Identity()
        )

        self.dw_local = nn.Sequential(
            nn.Conv2d(channels, channels, kernel_size=5, padding=2, groups=channels, bias=False),
            norm(channels),
            nn.GELU(),
        )
        self.dw_horizontal = nn.Sequential(
            nn.Conv2d(channels, channels, kernel_size=(1, k), padding=(0, k // 2), groups=channels, bias=False),
            norm(channels),
            nn.GELU(),
        )
        self.dw_vertical = nn.Sequential(
            nn.Conv2d(channels, channels, kernel_size=(k, 1), padding=(k // 2, 0), groups=channels, bias=False),
            norm(channels),
            nn.GELU(),
        )
        self.fuse = nn.Conv2d(channels, channels, kernel_size=1, bias=True)
        self.act = nn.GELU()
        self.gate = nn.Sigmoid()

        self.se_inner = None
        if se_inner:
            self.se_inner = nn.Sequential(
                nn.AdaptiveAvgPool2d(1),
                nn.Conv2d(channels, channels, kernel_size=1, bias=True),
                nn.Sigmoid(),
            )

        self._init_weights()

    def _init_weights(self) -> None:
        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode="fan_out", nonlinearity="relu")
                if m.bias is not None:
                    nn.init.zeros_(m.bias)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: Input tensor of shape (B, C, H, W).
        Returns:
            Tensor of shape (B, C, H, W) after attention modulation.
        """
        identity = self.identity(x)

        attn = self.dw_local(x)
        attn = attn + self.dw_horizontal(attn)
        attn = attn + self.dw_vertical(attn)

        attn = self.fuse(attn)
        if self.se_inner is not None:
            attn = attn * self.se_inner(attn)
        attn = self.act(attn)
        attn = self.gate(attn)

        return identity * attn


class CrossTaskMixer(nn.Module):
    """General mixer that injects multimodal cues into a target head."""

    def __init__(self, channels: int, context_tasks: tuple[str, ...]) -> None:
        super().__init__()
        self.context_tasks = context_tasks
        self.projections = nn.ModuleDict(
            {
                task: nn.Sequential(
                    nn.Conv2d(channels, channels, kernel_size=1, bias=False),
                    nn.GroupNorm(1, channels),
                    nn.GELU(),
                )
                for task in context_tasks
            }
        )
        self.gates = nn.ModuleDict(
            {
                task: nn.Sequential(
                    nn.Conv2d(channels, channels, kernel_size=1, bias=False),
                    nn.GroupNorm(1, channels),
                    nn.GELU(),
                    nn.Conv2d(channels, 1, kernel_size=1),
                    nn.Sigmoid(),
                )
                for task in context_tasks
            }
        )
        self.fuse = nn.Sequential(
            nn.Conv2d(channels * (len(context_tasks) + 1), channels, kernel_size=1, bias=False),
            nn.GroupNorm(1, channels),
            nn.GELU(),
        )
        self.cross = MSCABlock(channels, strip_kernel_size=9, use_proj=True, se_inner=False)

    def forward(self, base: torch.Tensor, context_maps: Dict[str, torch.Tensor] | None) -> torch.Tensor:
        if not context_maps:
            return base
        target_hw = base.shape[-2:]
        fused = [base]
        for task in self.context_tasks:
            tensor = context_maps.get(task)
            if tensor is None:
                continue
            if tensor.shape[-2:] != target_hw:
                tensor = F.interpolate(tensor, size=target_hw, mode="bilinear", align_corners=False)
            proj = self.projections[task](tensor)
            gate = self.gates[task](tensor)
            fused.append(proj * (1.0 + gate))
        mixed = torch.cat(fused, dim=1)
        mixed = self.fuse(mixed)
        return mixed + self.cross(mixed)


class _BaseHead(nn.Module):
    def __init__(self, decoder_dim, out_dim, num_scales=4, extra_refine_blocks: int = 0):
        super().__init__()
        self.num_scales = num_scales
        self.fusion_blocks = nn.ModuleList([FusionBlock(decoder_dim) for _ in range(num_scales)])
        self.pred_conv = PredictionMLP(decoder_dim, out_dim)
        self.upsample = nn.Upsample(scale_factor=2, mode="bilinear", align_corners=True)
        self.refine_blocks = nn.ModuleList([FusionBlock(decoder_dim) for _ in range(extra_refine_blocks)])

    def _fuse(
        self,
        features_list: List[torch.Tensor],
        aux_indices: tuple[int, ...] = (),
        dynamic_pred_weights: torch.Tensor | None = None,
    ) -> Tuple[torch.Tensor, list, torch.Tensor]:
        # features_list ordered finest->coarsest; iterate coarse->fine
        fused_map = None
        aux_preds: list[torch.Tensor] = []
        target_size = (features_list[0].shape[-2] * 4, features_list[0].shape[-1] * 4)
        for i, feat in enumerate(reversed(features_list)):
            if fused_map is not None:
                upsampled = self.upsample(fused_map)
                upsampled = F.interpolate(upsampled, size=feat.shape[-2:], mode="bilinear", align_corners=False)
                feat = feat + upsampled
            fused_map = self.fusion_blocks[-(i + 1)](feat)
            if i in aux_indices:
                aux_pred = self.pred_conv(fused_map)
                aux_preds.append(F.interpolate(aux_pred, size=target_size, mode="bilinear", align_corners=False))
        for block in self.refine_blocks:
            fused_map = block(fused_map)
        pred = self.pred_conv(fused_map, dynamic_weights=dynamic_pred_weights)
        pred_out = F.interpolate(pred, scale_factor=4, mode="bilinear", align_corners=False)
        return pred_out, aux_preds, fused_map


class SimpleSegmentationHead(_BaseHead):
    def __init__(
        self,
        decoder_dim,
        num_classes,
        num_scales=4,
        aux_indices: tuple[int, ...] = (1, 2),
        extra_refine_blocks: int = 0,
        use_context: bool = True,
        boundary_threshold: float | None = None,
        boundary_strength: float = 0.1,
        context_tasks: tuple[str, ...] = ("depth", "normals", "edge"),
    ):
        super().__init__(decoder_dim, num_classes, num_scales, extra_refine_blocks=extra_refine_blocks)
        self.aux_indices = aux_indices
        self.context = MSCABlock(decoder_dim, strip_kernel_size=9, use_proj=False, se_inner=True) if use_context else None
        self.task_mixer = CrossTaskMixer(decoder_dim, context_tasks=context_tasks) if context_tasks else None
        self.boundary_refine = (
            EdgeAwareRefine(num_classes, threshold=boundary_threshold, strength=boundary_strength)
            if boundary_threshold is not None and boundary_strength > 0
            else None
        )

    def forward(
        self,
        features_list: List[torch.Tensor],
        return_aux: bool = False,
        context_maps: Dict[str, torch.Tensor] | None = None,
        return_context: bool = False,
        dynamic_weights: torch.Tensor | None = None,
    ):
        pred_out, aux_preds, fused_map = self._fuse(
            features_list,
            aux_indices=self.aux_indices if return_aux else (),
            dynamic_pred_weights=dynamic_weights,
        )
        if self.task_mixer is not None:
            fused_map = self.task_mixer(fused_map, context_maps)
        if self.context is not None:
            context_map = self.context(fused_map)
            context_logits = self.pred_conv(context_map)
            context_logits = F.interpolate(context_logits, scale_factor=4, mode="bilinear", align_corners=False)
            pred_out = pred_out + context_logits
        outputs = (pred_out, aux_preds) if return_aux else pred_out
        if return_context:
            return outputs, fused_map
        return outputs

    def refine_with_edges(self, seg_logits: torch.Tensor, edge_logits: torch.Tensor | None) -> torch.Tensor:
        if self.boundary_refine is None or edge_logits is None:
            return seg_logits
        return self.boundary_refine(seg_logits, edge_logits)


class RegressionHead(_BaseHead):
    def __init__(self, decoder_dim, output_dim, num_scales=4, aux_indices: tuple[int, ...] = (1,), extra_refine_blocks: int = 0):
        super().__init__(decoder_dim, output_dim, num_scales, extra_refine_blocks=extra_refine_blocks)
        self.aux_indices = aux_indices

    def forward(self, features_list: List[torch.Tensor], return_aux: bool = False):
        pred_out, aux_preds, _ = self._fuse(features_list, aux_indices=self.aux_indices if return_aux else ())
        return (pred_out, aux_preds) if return_aux else pred_out


class NormalsHead(_BaseHead):
    """Surface normals head with lightweight refinement."""

    def __init__(
        self,
        decoder_dim: int,
        num_scales: int = 4,
        aux_indices: tuple[int, ...] = (1, 2),
        extra_refine_blocks: int = 1,
        context_tasks: tuple[str, ...] = ("semseg", "depth", "edge"),
    ):
        super().__init__(decoder_dim, 3, num_scales, extra_refine_blocks=extra_refine_blocks)
        self.aux_indices = aux_indices
        self.context = None
        self.orientation_enhancer = nn.Identity()
        self.normal_proj = nn.Sequential(
            nn.Conv2d(decoder_dim, decoder_dim, kernel_size=1, bias=False),
            nn.GELU(),
            nn.Conv2d(decoder_dim, 3, kernel_size=1),
        )
        self.context_mixer = CrossTaskMixer(decoder_dim, context_tasks=context_tasks) if context_tasks else None

    def forward(
        self,
        features_list: List[torch.Tensor],
        return_aux: bool = False,
        context_maps: Dict[str, torch.Tensor] | None = None,
        return_context: bool = False,
        dynamic_weights: torch.Tensor | None = None,
    ):
        _, aux_preds, fused_map = self._fuse(
            features_list,
            aux_indices=self.aux_indices if return_aux else (),
        )
        if self.context_mixer is not None:
            fused_map = self.context_mixer(fused_map, context_maps)
        enhanced = self.orientation_enhancer(fused_map)
        if dynamic_weights is not None:
            proj_feat = self.normal_proj[0](enhanced)
            proj_feat = self.normal_proj[1](proj_feat)
            B, C_in, H, W = proj_feat.shape
            C_out = dynamic_weights.shape[1]
            x_reshaped = proj_feat.reshape(1, B * C_in, H, W)
            w_reshaped = dynamic_weights.reshape(B * C_out, C_in, 1, 1)
            normals = F.conv2d(x_reshaped, w_reshaped, groups=B).reshape(B, C_out, H, W)
        else:
            normals = self.normal_proj(enhanced)
        normals = F.interpolate(normals, scale_factor=4, mode="bilinear", align_corners=False)
        outputs = (normals, aux_preds) if return_aux else normals
        if return_context:
            return outputs, enhanced
        return outputs


class EdgeHead(_BaseHead):
    """Edge head streamlined for efficiency."""

    def __init__(
        self,
        decoder_dim: int,
        output_dim: int = 1,
        num_scales: int = 4,
        aux_indices: tuple[int, ...] = (1,),
        extra_refine_blocks: int = 1,
    ):
        super().__init__(decoder_dim, output_dim, num_scales, extra_refine_blocks=extra_refine_blocks)
        self.aux_indices = aux_indices
        self.context = None
        self.high_freq_branch = None
        mid_channels = max(decoder_dim // 2, 32)
        self.edge_proj = nn.Sequential(
            nn.Conv2d(decoder_dim, mid_channels, kernel_size=3, padding=1, bias=False),
            nn.GELU(),
            nn.Conv2d(mid_channels, output_dim, kernel_size=1),
        )
        self.confidence_gate = None
        self.context_mixer = CrossTaskMixer(decoder_dim, context_tasks=("semseg", "depth", "normals"))

    def forward(
        self,
        features_list: List[torch.Tensor],
        return_aux: bool = False,
        context_maps: Dict[str, torch.Tensor] | None = None,
        return_context: bool = False,
        dynamic_weights: torch.Tensor | None = None,
    ):
        _, aux_preds, fused_map = self._fuse(
            features_list,
            aux_indices=self.aux_indices if return_aux else (),
        )
        fused_map = self.context_mixer(fused_map, context_maps)
        context = self.context(fused_map) if self.context is not None else 0
        freq = self.high_freq_branch(fused_map) if self.high_freq_branch is not None else 0
        enriched = fused_map + context + freq
        if dynamic_weights is not None:
            proj_feat = self.edge_proj[0](enriched)
            proj_feat = self.edge_proj[1](proj_feat)
            B, C_in, H, W = proj_feat.shape
            C_out = dynamic_weights.shape[1]
            x_reshaped = proj_feat.reshape(1, B * C_in, H, W)
            w_reshaped = dynamic_weights.reshape(B * C_out, C_in, 1, 1)
            logits = F.conv2d(x_reshaped, w_reshaped, groups=B).reshape(B, C_out, H, W)
        else:
            logits = self.edge_proj(enriched)
        logits = F.interpolate(logits, scale_factor=4, mode="bilinear", align_corners=True)
        if self.confidence_gate is not None:
            conf = torch.sigmoid(self.confidence_gate(freq))
            if conf.shape[-2:] != logits.shape[-2:]:
                conf = F.interpolate(conf, size=logits.shape[-2:], mode="bilinear", align_corners=True)
            logits = logits * (1.0 + conf)
        outputs = (logits, aux_preds) if return_aux else logits
        if return_context:
            return outputs, enriched
        return outputs


class EdgeAwareRefine(nn.Module):
    """Use confident edge predictions to refine semantic logits."""

    def __init__(self, num_classes: int, threshold: float = 0.6, strength: float = 0.1):
        super().__init__()
        self.threshold = float(threshold)
        self.strength = float(strength)
        self.conv = nn.Conv2d(1, num_classes, kernel_size=1, bias=True)

    def forward(self, seg_logits: torch.Tensor, edge_logits: torch.Tensor) -> torch.Tensor:
        edge_prob = torch.sigmoid(edge_logits)
        gate = torch.sigmoid((edge_prob - self.threshold) * 10.0)
        correction = self.conv(gate) * self.strength
        return seg_logits + correction


class DepthResidualHead(_BaseHead):
    """Depth head with multi-scale sharpening augmented by semantic feedback."""

    def __init__(
        self,
        decoder_dim,
        num_scales=4,
        aux_indices: tuple[int, ...] = (1, 2),
        extra_refine_blocks: int = 1,
        msca_kernel: int = 7,
    ):
        super().__init__(decoder_dim, out_dim=1, num_scales=num_scales, extra_refine_blocks=extra_refine_blocks)
        self.aux_indices = aux_indices
        # SegNeXt-style attention to refine the fused depth features in a cheap way.
        self.attn = MSCABlock(decoder_dim, strip_kernel_size=msca_kernel, use_proj=False, se_inner=True)
        self.residual_dw = nn.Conv2d(decoder_dim, decoder_dim, kernel_size=3, padding=1, groups=decoder_dim, bias=False)
        self.residual_pw = nn.Conv2d(decoder_dim, 1, kernel_size=1, bias=True)
        mid_channels = max(16, decoder_dim // 4)
        self.confidence_head = nn.Sequential(
            nn.Conv2d(decoder_dim, mid_channels, kernel_size=3, padding=1, bias=False),
            _group_norm(mid_channels),
            nn.GELU(),
            nn.Conv2d(mid_channels, 1, kernel_size=1),
            nn.Sigmoid(),
        )
        self.laplacian = nn.Conv2d(decoder_dim, decoder_dim, kernel_size=3, padding=1, groups=decoder_dim, bias=False)
        self._init_laplacian(decoder_dim)
        self.multi_dilate = nn.ModuleList()  # pruned heavy dilation branches
        self.dilate_merge = nn.Identity()
        self.context_mixer = CrossTaskMixer(decoder_dim, context_tasks=("semseg", "normals", "edge"))

    def _init_laplacian(self, channels: int) -> None:
        kernel = torch.tensor([[0.0, -1.0, 0.0], [-1.0, 4.0, -1.0], [0.0, -1.0, 0.0]])
        weight = kernel.view(1, 1, 3, 3).repeat(channels, 1, 1, 1)
        with torch.no_grad():
            self.laplacian.weight.copy_(weight)
        for param in self.laplacian.parameters():
            param.requires_grad = False

    def forward(
        self,
        features_list: List[torch.Tensor],
        return_aux: bool = False,
        context_maps: Dict[str, torch.Tensor] | None = None,
        return_context: bool = False,
    ):
        coarse, aux_preds, fused_map = self._fuse(features_list, aux_indices=self.aux_indices if return_aux else ())
        fused_map = self.context_mixer(fused_map, context_maps)
        if len(self.multi_dilate) > 0:
            context_feats = [fused_map]
            for branch in self.multi_dilate:
                context_feats.append(branch(fused_map))
            fused_map = self.dilate_merge(torch.cat(context_feats, dim=1))
        fused_map = self.attn(fused_map)
        shared_feat = fused_map
        lap_feat = self.laplacian(fused_map)
        residual_feat = F.relu(self.residual_dw(fused_map + lap_feat))
        residual = self.residual_pw(residual_feat)
        confidence = self.confidence_head(residual_feat)
        residual = residual * confidence
        residual = F.interpolate(residual, scale_factor=4, mode="bilinear", align_corners=False)
        pred_out = coarse + residual
        outputs = (pred_out, aux_preds) if return_aux else pred_out
        if return_context:
            return outputs, shared_feat
        return outputs


class BinDepthHead(_BaseHead):
    """Adaptive bin-based depth head with dual refinement."""

    def __init__(
        self,
        decoder_dim: int,
        num_scales: int = 4,
        aux_indices: tuple[int, ...] = (1, 2),
        extra_refine_blocks: int = 1,
        min_depth: float = 0.0,
        max_depth: float = 10.0,
        n_bins: int = 64,
    ):
        super().__init__(decoder_dim, out_dim=1, num_scales=num_scales, extra_refine_blocks=extra_refine_blocks)
        self.aux_indices = aux_indices
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.n_bins = int(n_bins)
        groups = max(1, decoder_dim // 32)

        self.bin_classifier = nn.Sequential(
            nn.Conv2d(decoder_dim, decoder_dim, kernel_size=3, padding=1, bias=False),
            nn.GroupNorm(groups, decoder_dim),
            nn.GELU(),
            nn.Conv2d(decoder_dim, self.n_bins, kernel_size=1),
        )
        mid_channels = max(64, decoder_dim // 2)
        self.offset_head = nn.Sequential(
            nn.Conv2d(decoder_dim, mid_channels, kernel_size=3, padding=1, bias=False),
            nn.GroupNorm(max(1, mid_channels // 32), mid_channels),
            nn.GELU(),
            nn.Conv2d(mid_channels, 1, kernel_size=1),
        )
        reducer = max(decoder_dim // 4, 32)
        self.bin_width_mlp = nn.Sequential(
            nn.AdaptiveAvgPool2d(1),
            nn.Conv2d(decoder_dim, reducer, kernel_size=1, bias=False),
            nn.GELU(),
            nn.Conv2d(reducer, self.n_bins, kernel_size=1),
        )

    def _compute_bin_centers(self, fused_map: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        B = fused_map.shape[0]
        logits = self.bin_width_mlp(fused_map).reshape(B, self.n_bins)
        widths = torch.softmax(logits, dim=1)
        depth_range = max(self.max_depth - self.min_depth, 1e-6)
        edges = torch.cumsum(widths, dim=1) * depth_range + self.min_depth
        edges_prev = torch.cat(
            [
                torch.full((B, 1), self.min_depth, device=fused_map.device, dtype=fused_map.dtype),
                edges[:, :-1],
            ],
            dim=1,
        )
        centers = (edges + edges_prev) * 0.5
        return centers, edges

    def forward(
        self,
        features_list: List[torch.Tensor],
        return_aux: bool = False,
        context_maps: Dict[str, torch.Tensor] | None = None,
        return_context: bool = False,
        dynamic_weights: Dict[str, torch.Tensor] | None = None,
    ):
        pred_weights = None
        bin_classifier_weights = None
        if dynamic_weights:
            pred_weights = dynamic_weights.get("depth")
            bin_classifier_weights = dynamic_weights.get("depth_bin_classifier")
        _, aux_preds, fused_map = self._fuse(
            features_list,
            aux_indices=self.aux_indices if return_aux else (),
            dynamic_pred_weights=pred_weights,
        )
        if bin_classifier_weights is not None:
            feat = self.bin_classifier[0](fused_map)
            feat = self.bin_classifier[1](feat)
            feat = self.bin_classifier[2](feat)
            B, C_in, H, W = feat.shape
            C_out = bin_classifier_weights.shape[1]
            x_reshaped = feat.reshape(1, B * C_in, H, W)
            w_reshaped = bin_classifier_weights.reshape(B * C_out, C_in, 1, 1)
            bin_logits = F.conv2d(x_reshaped, w_reshaped, groups=B).reshape(B, C_out, H, W)
        else:
            bin_logits = self.bin_classifier(fused_map)
        bin_probs = F.softmax(bin_logits, dim=1)
        centers, edges = self._compute_bin_centers(fused_map)
        centers = centers.reshape(centers.shape[0], self.n_bins, 1, 1)
        coarse_depth = torch.sum(bin_probs * centers, dim=1, keepdim=True)
        offset = self.offset_head(fused_map)
        pred = coarse_depth + offset
        pred = F.interpolate(pred, scale_factor=4, mode="bilinear", align_corners=False)
        coarse_up = F.interpolate(coarse_depth, scale_factor=4, mode="bilinear", align_corners=False)
        offset_up = F.interpolate(offset, scale_factor=4, mode="bilinear", align_corners=False)
        output_dict = {
            "depth": pred,
            "coarse": coarse_up,
            "offset": offset_up,
            "bin_logits": bin_logits,
            "bin_edges": edges,
        }
        outputs = (output_dict, aux_preds) if return_aux else output_dict
        if return_context:
            return outputs, fused_map
        return outputs
