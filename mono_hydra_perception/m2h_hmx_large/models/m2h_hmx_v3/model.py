import torch
import torch.nn as nn
import torch.nn.functional as F
from dataclasses import dataclass, field
from typing import Dict, List

from .backbone import HFA_Adapter_V3
from .decoder import HMX_Decoder
from .heads import SimpleSegmentationHead, RegressionHead, EdgeHead, BinDepthHead, NormalsHead

# Training losses are not used by ROS inference, but model construction expects these symbols.
try:
    from models.m2h_hmx.losses import (  # type: ignore
        CombinedSegLoss,
        SILogRMSELoss,
        CosineSimilarityLoss,
        BCEWithLogitsLoss,
        FocalLoss,
        UncertaintyLoss,
        geometric_consistency_loss,
    )
except Exception:  # pragma: no cover
    class _InferenceOnlyLoss(nn.Module):  # type: ignore[misc]
        def __init__(self, *args, **kwargs) -> None:
            super().__init__()
        def forward(self, *args, **kwargs):
            return torch.tensor(0.0)

    CombinedSegLoss = SILogRMSELoss = CosineSimilarityLoss = BCEWithLogitsLoss = FocalLoss = UncertaintyLoss = _InferenceOnlyLoss

    def geometric_consistency_loss(*args, **kwargs):
        return torch.tensor(0.0)


def _edge_dice_loss(logits: torch.Tensor, target: torch.Tensor, ignore_index: int = 255, eps: float = 1e-6) -> torch.Tensor:
    probs = torch.sigmoid(logits)
    if target.ndim == probs.ndim - 1:
        target = target.unsqueeze(1)
    mask = (target != ignore_index).float()
    preds = probs * mask
    tgts = (target.float() * mask)
    intersection = (preds * tgts).sum(dim=(2, 3))
    denom = preds.sum(dim=(2, 3)) + tgts.sum(dim=(2, 3)) + eps
    dice = 1.0 - (2.0 * intersection + eps) / denom
    return dice.mean()


def _semantic_edge_map_from_logits(logits: torch.Tensor) -> torch.Tensor:
    with torch.no_grad():
        probs = torch.softmax(logits, dim=1)
        hard = torch.argmax(probs, dim=1, keepdim=True).float()
        gx = torch.abs(hard[:, :, :, 1:] - hard[:, :, :, :-1])
        gy = torch.abs(hard[:, :, 1:, :] - hard[:, :, :-1, :])
        gx = F.pad(gx, (0, 1, 0, 0))
        gy = F.pad(gy, (0, 0, 0, 1))
        edge = torch.clamp(gx + gy, 0, 1)
    return edge


@dataclass
class HMXV3ModelConfig:
    backbone_name: str = "facebook/dinov3-vitb16-pretrain-lvd1689m"
    decoder_dim: int = 256
    num_seg_classes: int = 40
    ltc_window_size: int = 4
    min_depth: float = 0.1
    max_depth: float = 8.0
    depth_bins: int = 64
    focal_for_edges: bool = False
    hm_d_state: int = 32
    hm_drop_path: float = 0.1
    gtf_extra_levels: int = 1
    train_last_n_blocks: int = 1
    intermediate_layer_indices: List[int] | None = None
    num_register_tokens: int = 4
    use_lora: bool = True
    lora_rank: int = 16
    lora_alpha: float = 16.0
    lora_dropout: float = 0.05
    aux_weights: Dict[str, float] = field(
        default_factory=lambda: {"semseg": 0.3, "depth": 0.3, "edge": 0.1, "normals": 0.0}
    )
    consistency_dn: float = 0.5
    consistency_se: float = 0.2
    geom_consistency_weight: float = 0.5
    use_uncertainty: bool = True
    task_weights: Dict[str, float] | None = None
    depth_scale_weight: float = 0.0
    edge_refine_threshold: float | None = None
    edge_refine_strength: float = 0.1
    depth_coarse_weight: float = 0.0
    depth_offset_weight: float = 0.0
    depth_bin_weight: float = 0.0
    edge_dice_weight: float = 0.0
    tasks: tuple[str, ...] = field(default_factory=lambda: ("semseg", "depth", "normals", "edge"))


class DINO_HMX_V3(nn.Module):
    """Enhanced M2H-HMX with partial backbone tuning, richer HM blocks, and deep supervision."""

    outputs_are_structured = True

    def __init__(self, cfg: HMXV3ModelConfig, num_seg_classes: int):
        super().__init__()
        self.cfg = cfg
        self.tasks = tuple(cfg.tasks) if getattr(cfg, "tasks", None) else ("semseg", "depth", "normals", "edge")
        self.task_to_idx = {name: idx for idx, name in enumerate(self.tasks)}
        self.num_tasks = len(self.tasks)
        self.expects_batch_dict = True
        self.min_depth = cfg.min_depth
        self.max_depth = cfg.max_depth

        self.hfa_adapter = HFA_Adapter_V3(
            model_name=cfg.backbone_name,
            decoder_dim=cfg.decoder_dim,
            intermediate_layer_indices=cfg.intermediate_layer_indices,
            train_last_n_blocks=cfg.train_last_n_blocks,
            use_lora=cfg.use_lora,
            lora_rank=cfg.lora_rank,
            lora_alpha=cfg.lora_alpha,
            lora_dropout=cfg.lora_dropout,
            num_register_tokens=cfg.num_register_tokens,
        )

        self.decoder = HMX_Decoder(
            num_scales=len(self.hfa_adapter.layer_indices),
            decoder_dim=cfg.decoder_dim,
            num_tasks=self.num_tasks,
            ltc_window_size=cfg.ltc_window_size,
            hm_d_state=cfg.hm_d_state,
            hm_drop_path=cfg.hm_drop_path,
            gtf_extra_levels=cfg.gtf_extra_levels,
        )

        seg_context_tasks = tuple(t for t in ("depth", "normals", "edge") if t in self.tasks)
        self.segmentation_head = (
            SimpleSegmentationHead(
                decoder_dim=cfg.decoder_dim,
                num_classes=num_seg_classes,
                num_scales=self.decoder.num_scales,
                extra_refine_blocks=1,
                boundary_threshold=cfg.edge_refine_threshold if "edge" in self.tasks else None,
                boundary_strength=cfg.edge_refine_strength,
                context_tasks=seg_context_tasks,
            )
            if "semseg" in self.tasks
            else None
        )
        self.depth_head = (
            BinDepthHead(
                decoder_dim=cfg.decoder_dim,
                num_scales=self.decoder.num_scales,
                extra_refine_blocks=1,
                min_depth=cfg.min_depth,
                max_depth=cfg.max_depth,
                n_bins=cfg.depth_bins,
            )
            if "depth" in self.tasks
            else None
        )
        normals_context_tasks = tuple(t for t in ("semseg", "depth", "edge") if t in self.tasks)
        self.normals_head = (
            NormalsHead(
                decoder_dim=cfg.decoder_dim, num_scales=self.decoder.num_scales, extra_refine_blocks=1, context_tasks=normals_context_tasks
            )
            if "normals" in self.tasks
            else None
        )
        self.edge_head = EdgeHead(decoder_dim=cfg.decoder_dim, output_dim=1, num_scales=self.decoder.num_scales) if "edge" in self.tasks else None

        self.seg_loss = CombinedSegLoss()
        if cfg.focal_for_edges:
            self.edge_loss = FocalLoss()
        else:
            self.edge_loss = BCEWithLogitsLoss()
        self.depth_loss = SILogRMSELoss(min_depth=self.min_depth, max_depth=self.max_depth)
        self.normal_loss = CosineSimilarityLoss()
        self.loss_balancer = UncertaintyLoss(num_tasks=self.num_tasks)
        self.aux_weights = cfg.aux_weights
        self.consistency_dn = cfg.consistency_dn
        self.consistency_se = cfg.consistency_se
        self.geom_consistency_weight = cfg.geom_consistency_weight
        self.depth_scale_weight = cfg.depth_scale_weight
        self.depth_coarse_weight = cfg.depth_coarse_weight
        self.depth_offset_weight = cfg.depth_offset_weight
        self.depth_bin_weight = cfg.depth_bin_weight
        self.edge_dice_weight = cfg.edge_dice_weight
        self.depth_bins = cfg.depth_bins
        self.use_uncertainty = bool(cfg.use_uncertainty)
        self.task_weights = cfg.task_weights or {}

    def _collect_task_context(self, decoder_outputs: Dict[str, List[torch.Tensor]]) -> Dict[str, torch.Tensor]:
        """Gather highest-resolution decoder features for each task."""
        context: Dict[str, torch.Tensor] = {}
        for name, idx in self.task_to_idx.items():
            feats = decoder_outputs.get(f"task_{idx}")
            if isinstance(feats, (list, tuple)) and feats:
                context[name] = feats[0]
        return context

    def format_outputs_for_tasks(self, outputs: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        formatted: Dict[str, torch.Tensor] = {}
        for task in self.tasks:
            if task in outputs:
                formatted[task] = outputs[task]
        if "aux" in outputs:
            formatted["aux"] = outputs["aux"]
        return formatted

    def forward(self, batch: dict) -> dict:
        feature_pyramid, register_tokens = self.hfa_adapter(batch["images"])
        decoder_outputs = self.decoder(feature_pyramid, register_tokens)
        # Task-aware mixers operate on shared decoder context.
        task_context = self._collect_task_context(decoder_outputs)

        return_aux = self.training
        aux_dict: Dict[str, List[torch.Tensor]] = {}
        pred: Dict[str, torch.Tensor] = {}
        depth_outputs: Dict[str, torch.Tensor] = {}
        pred_edge = None

        def feats(task: str) -> list[torch.Tensor]:
            return decoder_outputs.get(f"task_{self.task_to_idx[task]}", [])

        if "depth" in self.tasks and self.depth_head is not None:
            depth_res = self.depth_head(
                feats("depth"),
                return_aux=return_aux,
                context_maps=task_context,
                return_context=True,
            )
            if return_aux:
                (depth_outputs, aux_depth), depth_features = depth_res
                aux_dict["depth"] = aux_depth
            else:
                depth_outputs, depth_features = depth_res
            pred_depth = depth_outputs["depth"]
            task_context["depth"] = depth_features
            pred["depth"] = pred_depth
            if depth_outputs.get("coarse") is not None:
                pred["depth_coarse"] = depth_outputs["coarse"]
            if depth_outputs.get("offset") is not None:
                pred["depth_offset"] = depth_outputs["offset"]
            if depth_outputs.get("bin_logits") is not None:
                pred["depth_bin_logits"] = depth_outputs["bin_logits"]
            if depth_outputs.get("bin_edges") is not None:
                pred["depth_bin_edges"] = depth_outputs["bin_edges"]

        if "edge" in self.tasks and self.edge_head is not None:
            edge_res = self.edge_head(
                feats("edge"),
                return_aux=return_aux,
                context_maps=task_context,
                return_context=True,
            )
            if return_aux:
                (pred_edge, aux_edge), edge_features = edge_res
                aux_dict["edge"] = aux_edge
            else:
                pred_edge, edge_features = edge_res
            task_context["edge"] = edge_features
            pred["edge"] = pred_edge

        if "normals" in self.tasks and self.normals_head is not None:
            normals_res = self.normals_head(
                feats("normals"),
                return_aux=return_aux,
                context_maps=task_context,
                return_context=True,
            )
            if return_aux:
                (pred_normals, aux_normals), normals_features = normals_res
                aux_dict["normals"] = aux_normals
            else:
                pred_normals, normals_features = normals_res
            pred_normals = F.normalize(pred_normals, p=2, dim=1)
            task_context["normals"] = normals_features
            pred["normals"] = pred_normals

        if "semseg" in self.tasks and self.segmentation_head is not None:
            seg_res = self.segmentation_head(
                feats("semseg"),
                return_aux=return_aux,
                context_maps=task_context,
                return_context=True,
            )
            if return_aux:
                (pred_seg, aux_seg), seg_features = seg_res
                aux_dict["semseg"] = aux_seg
            else:
                pred_seg, seg_features = seg_res
            if pred_edge is not None:
                pred_seg = self.segmentation_head.refine_with_edges(pred_seg, pred_edge)
            task_context["semseg"] = seg_features
            pred["semseg"] = pred_seg

        if aux_dict:
            pred["aux"] = aux_dict
        return {"pred": pred}

    def _aux_losses(self, aux_preds: List[torch.Tensor], target: torch.Tensor, criterion, weight: float) -> torch.Tensor:
        if not aux_preds or weight <= 0 or target is None:
            return torch.tensor(0.0, device=self.loss_balancer.log_vars.device)
        return sum(criterion(p, target) * weight for p in aux_preds)

    def compute_losses(self, outputs: dict, targets: dict) -> dict:
        task_losses: Dict[str, torch.Tensor] = {}
        all_losses: Dict[str, float | torch.Tensor] = {}
        aux = outputs.get("aux", {})

        # Segmentation
        main_seg = outputs.get("semseg")
        seg_loss = (
            self.seg_loss(main_seg, targets["semseg"]) if "semseg" in outputs and "semseg" in targets else torch.tensor(0.0, device=self.loss_balancer.log_vars.device)
        )
        seg_loss = seg_loss + self._aux_losses(aux.get("semseg", []), targets.get("semseg"), self.seg_loss, self.aux_weights.get("semseg", 0.0))
        task_losses["semseg"] = seg_loss

        # Depth
        depth_loss = (
            self.depth_loss(outputs["depth"], targets["depth"]) if "depth" in outputs and "depth" in targets else torch.tensor(0.0, device=self.loss_balancer.log_vars.device)
        )
        depth_loss = depth_loss + self._aux_losses(aux.get("depth", []), targets.get("depth"), self.depth_loss, self.aux_weights.get("depth", 0.0))
        if self.depth_scale_weight > 0 and "depth" in outputs and "depth" in targets:
            l1_loss = F.l1_loss(outputs["depth"], targets["depth"])
            depth_loss = depth_loss + l1_loss * self.depth_scale_weight
        coarse_pred = outputs.get("depth_coarse")
        if coarse_pred is not None and "depth" in targets and self.depth_coarse_weight > 0:
            coarse_loss = F.l1_loss(coarse_pred, targets["depth"])
            depth_loss = depth_loss + coarse_loss * self.depth_coarse_weight
        offset_pred = outputs.get("depth_offset")
        if (
            offset_pred is not None
            and coarse_pred is not None
            and "depth" in targets
            and self.depth_offset_weight > 0
        ):
            residual_target = targets["depth"] - coarse_pred.detach()
            offset_loss = F.l1_loss(offset_pred, residual_target)
            depth_loss = depth_loss + offset_loss * self.depth_offset_weight
        bin_logits = outputs.get("depth_bin_logits")
        bin_edges = outputs.get("depth_bin_edges")
        if (
            bin_logits is not None
            and bin_edges is not None
            and "depth" in targets
            and self.depth_bin_weight > 0
        ):
            target_depth = targets["depth"]
            logits_h, logits_w = bin_logits.shape[-2:]
            resized = F.interpolate(target_depth, size=(logits_h, logits_w), mode="bilinear", align_corners=False)
            resized = resized.squeeze(1)
            mask = (resized > 0).detach()
            if self.min_depth is not None:
                min_d = self.min_depth
            else:
                finite = resized[mask]
                min_d = float(finite.min().item()) if finite.numel() > 0 else 0.0
            if self.max_depth is not None:
                max_d = self.max_depth
            else:
                finite = resized[mask]
                max_d = float(finite.max().item()) if finite.numel() > 0 else min_d + 1.0
            clamped = resized.clamp(min=min_d, max=max_d)
            bin_targets = torch.zeros_like(clamped, dtype=torch.long)
            for b in range(clamped.shape[0]):
                edges_b = bin_edges[b]
                flat = clamped[b].flatten()
                idx = torch.bucketize(flat, edges_b)
                idx = torch.clamp(idx, max=self.depth_bins - 1)
                bin_targets[b] = idx.view_as(clamped[b])
            log_probs = F.log_softmax(bin_logits, dim=1)
            nll = F.nll_loss(log_probs, bin_targets, reduction="none")
            masked = nll * mask.float()
            denom = mask.float().sum().clamp_min(1.0)
            depth_loss = depth_loss + masked.sum() / denom * self.depth_bin_weight
        task_losses["depth"] = depth_loss

        # Normals + geometric consistency
        geom_weight = self.geom_consistency_weight
        if "normals" in outputs and "normals" in targets and "depth" in outputs and "depth" in targets:
            normal_loss = self.normal_loss(
                outputs["normals"],
                targets["normals"],
                mask=(targets["normals"].abs().sum(dim=1, keepdim=True) != 0),
            )
            if geom_weight > 0:
                geom_loss = geometric_consistency_loss(outputs["depth"], outputs["normals"], targets["depth"]) * geom_weight
                normals_total = normal_loss + geom_loss
            else:
                normals_total = normal_loss
        else:
            normals_total = torch.tensor(0.0, device=self.loss_balancer.log_vars.device)
        normals_total = normals_total + self._aux_losses(
            aux.get("normals", []), targets.get("normals"), self.normal_loss, self.aux_weights.get("normals", 0.0)
        )
        task_losses["normals"] = normals_total

        # Edge
        if "edge" in outputs and "edge" in targets:
            edge_loss = self.edge_loss(outputs["edge"], targets["edge"])
            if self.edge_dice_weight > 0:
                edge_loss = edge_loss + _edge_dice_loss(outputs["edge"], targets["edge"]) * self.edge_dice_weight
        else:
            edge_loss = torch.tensor(0.0, device=self.loss_balancer.log_vars.device)
        edge_loss = edge_loss + self._aux_losses(aux.get("edge", []), targets.get("edge"), self.edge_loss, self.aux_weights.get("edge", 0.0))
        task_losses["edge"] = edge_loss

        # Consistency regularizers (lightweight)
        if self.consistency_dn > 0 and "depth" in outputs and "normals" in outputs:
            task_losses["normals"] = task_losses["normals"] + geometric_consistency_loss(
                outputs["depth"].detach(), outputs["normals"]
            ) * self.consistency_dn
        if self.consistency_se > 0 and "edge" in outputs and "semseg" in outputs:
            pseudo_edges = _semantic_edge_map_from_logits(outputs["semseg"]).detach()
            task_losses["edge"] = task_losses["edge"] + F.l1_loss(torch.sigmoid(outputs["edge"]), pseudo_edges) * self.consistency_se

        for task_name, raw_loss in task_losses.items():
            all_losses[f"raw_loss/{task_name}"] = raw_loss.item() if torch.is_tensor(raw_loss) else raw_loss

        raw_total = sum(task_losses.values())
        use_balancer = self.use_uncertainty and self.training
        if use_balancer:
            total_loss, balanced_losses = self.loss_balancer(task_losses)
        else:
            balanced_losses = {}
            total_loss = None
            for task_name, loss in task_losses.items():
                weight = float(self.task_weights.get(task_name, 1.0))
                weighted_loss = loss * weight
                balanced_losses[task_name] = weighted_loss
                total_loss = weighted_loss if total_loss is None else (total_loss + weighted_loss)
            if total_loss is None:
                total_loss = raw_total

        # DDP safeguard
        dummy_loss = sum(p.sum() for p in outputs.values() if torch.is_tensor(p)) * 0.0
        total_loss = total_loss + dummy_loss

        all_losses.update({f"loss_{k}": v for k, v in balanced_losses.items()})
        if use_balancer:
            all_losses["total"] = raw_total.sum() if torch.is_tensor(raw_total) else raw_total
            all_losses["total_balanced"] = total_loss.sum() if torch.is_tensor(total_loss) else total_loss
        else:
            all_losses["total"] = total_loss.sum() if torch.is_tensor(total_loss) else total_loss

        if self.training and use_balancer:
            sorted_tasks = sorted(task_losses.keys())
            for i, task_name in enumerate(sorted_tasks):
                all_losses[f"balance/log_var_{task_name}"] = self.loss_balancer.log_vars[i].item()

        return all_losses
