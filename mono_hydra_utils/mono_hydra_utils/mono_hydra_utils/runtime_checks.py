from __future__ import annotations

import importlib
import sys


REQUIRED_MODULES = (
    "torch",
    "torchvision",
    "transformers",
    "timm",
    "einops",
    "onnxruntime",
    "mamba_ssm",
)


def _version(module_name: str) -> str:
    module = importlib.import_module(module_name)
    return str(getattr(module, "__version__", "unknown"))


def main() -> int:
    missing: list[str] = []
    for module_name in REQUIRED_MODULES:
        try:
            print(f"{module_name}: ok {_version(module_name)}")
        except Exception as exc:
            missing.append(f"{module_name}: {exc}")

    if missing:
        print("Missing or unusable M2H-HMX-Large runtime dependencies:", file=sys.stderr)
        for item in missing:
            print(f"  {item}", file=sys.stderr)
        return 1

    try:
        torch = importlib.import_module("torch")
        print(f"torch.cuda_available: {torch.cuda.is_available()}")
    except Exception:
        pass

    try:
        onnxruntime = importlib.import_module("onnxruntime")
        providers = ", ".join(onnxruntime.get_available_providers())
        print(f"onnxruntime.providers: {providers}")
    except Exception:
        pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
