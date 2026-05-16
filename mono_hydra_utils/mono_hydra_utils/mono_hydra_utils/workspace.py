from __future__ import annotations

import os
from pathlib import Path


def package_source_root() -> Path:
    """Return the source tree root for the mono_hydra_utils package."""
    return Path(__file__).resolve().parents[1]


def workspace_root(start: Path | None = None) -> Path:
    """Locate the ROS 2 workspace root used by Mono Hydra tools."""
    env_root = os.environ.get("MONO_HYDRA_ROS2_WS")
    if env_root:
        return Path(env_root).expanduser().resolve()

    current = Path(start or __file__).resolve()
    for candidate in (current, *current.parents):
        if (candidate / "src" / "mono_hydra_utils").exists():
            return candidate

    return Path.cwd().resolve()
