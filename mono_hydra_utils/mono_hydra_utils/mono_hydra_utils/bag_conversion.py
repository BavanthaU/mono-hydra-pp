from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

from .workspace import workspace_root


DEFAULT_ITC_BAG = Path(
    "/home/bavantha/ros1_workspaces/hydra2_ws/data/itc/ITC_2nd_floor_full_loop.bag"
)

DEFAULT_TOPICS = (
    "/camera/color/image_raw",
    "/camera/color/camera_info",
    "/camera/imu",
    "/tf_static",
)


def _run(command: list[str]) -> None:
    print(" ".join(command), flush=True)
    subprocess.run(command, check=True)


def ensure_rosbags_venv(workspace: Path) -> Path:
    venv = workspace / ".venv_rosbags"
    converter = venv / "bin" / "rosbags-convert"
    if converter.exists():
        return converter

    _run([sys.executable, "-m", "venv", str(venv)])
    pip = venv / "bin" / "python"
    _run([str(pip), "-m", "pip", "install", "--upgrade", "pip"])
    _run([str(pip), "-m", "pip", "install", "rosbags"])
    return converter


def default_output_bag(workspace: Path, source_bag: Path) -> Path:
    bag_name = source_bag.stem
    return workspace / "test_data" / "itc_ros2_bags" / f"{bag_name}_ros2"


def convert_ros1_bag(
    source_bag: Path,
    destination_bag: Path | None = None,
    topics: tuple[str, ...] = DEFAULT_TOPICS,
    storage: str = "mcap",
) -> Path:
    workspace = workspace_root()
    source_bag = source_bag.expanduser().resolve()
    destination = (destination_bag or default_output_bag(workspace, source_bag)).expanduser()

    if not source_bag.is_file():
        raise FileNotFoundError(f"ROS 1 bag not found: {source_bag}")

    converter = ensure_rosbags_venv(workspace)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        shutil.rmtree(destination)

    command = [
        str(converter),
        "--src",
        str(source_bag),
        "--dst",
        str(destination),
        "--dst-storage",
        storage,
        "--src-typestore",
        "ros1_noetic",
        "--dst-typestore",
        "ros2_jazzy",
        "--include-topic",
        *topics,
    ]
    _run(command)

    metadata = destination / "metadata.yaml"
    if not metadata.is_file():
        raise RuntimeError(f"Conversion did not produce metadata.yaml at {metadata}")

    return destination


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert selected Mono Hydra ROS 1 bag topics to a ROS 2 Jazzy bag."
    )
    parser.add_argument(
        "source_bag",
        nargs="?",
        default=str(DEFAULT_ITC_BAG),
        help="Input ROS 1 bag path.",
    )
    parser.add_argument(
        "destination_bag",
        nargs="?",
        default=None,
        help="Output ROS 2 bag directory. Defaults to test_data/itc_ros2_bags/<name>_ros2.",
    )
    parser.add_argument(
        "--topic",
        action="append",
        dest="topics",
        help="Topic to include. May be repeated. Defaults to the ITC runtime topics.",
    )
    parser.add_argument(
        "--storage",
        default="mcap",
        help="rosbag2 storage plugin to write. Default: mcap.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    topics = tuple(args.topics) if args.topics else DEFAULT_TOPICS
    try:
        output = convert_ros1_bag(
            Path(args.source_bag),
            Path(args.destination_bag) if args.destination_bag else None,
            topics=topics,
            storage=args.storage,
        )
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
