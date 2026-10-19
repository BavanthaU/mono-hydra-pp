#!/usr/bin/env python3
"""Run a deterministic health check on the bundled fixed-shape ONNX graph."""
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import onnxruntime as ort

parser = argparse.ArgumentParser()
parser.add_argument("--model", type=Path, default=Path(__file__).parents[1] / "models/mavista_geost_step025250_temporal_480x640.onnx")
args = parser.parse_args()
session = ort.InferenceSession(str(args.model), providers=["CPUExecutionProvider"])
image = np.zeros((1, 3, 480, 640), dtype=np.float32)
intrinsics = np.array([[518.8579, 519.4696, 320.0, 240.0]], dtype=np.float32)
state = np.zeros((1, 96, 30, 40), dtype=np.float32)
reset = np.array([True], dtype=np.bool_)
inputs = {
    "image": image,
    "previous_hidden_state": state,
    "reset_mask": reset,
    "intrinsics": intrinsics,
}
depth, logits, confidence, next_state = session.run(None, inputs)
inputs["previous_hidden_state"] = next_state
inputs["reset_mask"] = np.array([False], dtype=np.bool_)
depth_next, logits_next, confidence_next, state_next = session.run(None, inputs)
labels = logits[:, 1:41].argmax(axis=1).astype(np.int32) + 1
assert depth.shape == (1, 1, 480, 640) and np.isfinite(depth).all()
assert confidence.shape == (1, 480, 640) and np.isfinite(confidence).all()
assert labels.min() >= 1 and labels.max() <= 40
assert next_state.shape == state.shape and np.isfinite(next_state).all()
assert np.isfinite(depth_next).all() and np.isfinite(logits_next).all() and np.isfinite(confidence_next).all()
assert np.isfinite(state_next).all()
print("providers:", session.get_providers())
print("metric_depth:", depth.shape, depth.dtype)
print("semantic_labels:", labels.shape, labels.dtype, int(labels.min()), int(labels.max()))
print("semantic_confidence:", confidence.shape, confidence.dtype)
print("next_hidden_state:", next_state.shape, next_state.dtype)
