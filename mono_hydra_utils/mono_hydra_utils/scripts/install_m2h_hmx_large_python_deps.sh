#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export MAX_JOBS="${MAX_JOBS:-2}"
export PIP_BREAK_SYSTEM_PACKAGES=1
TORCH_VERSION="${TORCH_VERSION:-2.10.0}"
TORCHVISION_VERSION="${TORCHVISION_VERSION:-0.25.0}"
MAMBA_VERSION="${MAMBA_VERSION:-2.3.2.post1}"

python3 -m pip install --user --upgrade pip
python3 -m pip install --user \
  "torch==${TORCH_VERSION}" \
  "torchvision==${TORCHVISION_VERSION}" \
  transformers \
  timm \
  einops \
  onnxruntime-gpu

PY_TAG=$(python3 - <<'PY'
import sys
print(f"cp{sys.version_info.major}{sys.version_info.minor}")
PY
)
ARCH=$(uname -m)
TORCH_SHORT=$(python3 - <<'PY'
import torch
parts = torch.__version__.split("+", 1)[0].split(".")
print(".".join(parts[:2]))
PY
)

if [ "$PY_TAG" = "cp312" ] && [ "$ARCH" = "x86_64" ] && [ "$TORCH_SHORT" = "2.10" ]; then
  python3 -m pip install --user --force-reinstall --no-deps \
    "https://github.com/state-spaces/mamba/releases/download/v${MAMBA_VERSION}/mamba_ssm-${MAMBA_VERSION}%2Bcu12torch2.10cxx11abiTRUE-cp312-cp312-linux_x86_64.whl"
else
  python3 -m pip install --user --force-reinstall --no-build-isolation \
    "mamba-ssm==${MAMBA_VERSION}"
fi

PYTHONPATH="$SCRIPT_DIR/.." python3 -m mono_hydra_utils.runtime_checks
