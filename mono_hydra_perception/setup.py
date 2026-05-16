from pathlib import Path

from setuptools import find_packages, setup

package_name = "mono_hydra_perception"


def package_data_files() -> list[tuple[str, list[str]]]:
    entries: list[tuple[str, list[str]]] = [
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ]
    for directory in ("config", "weights", "onnx_models"):
        root = Path(directory)
        if not root.exists():
            continue
        for path in sorted(p for p in root.rglob("*") if p.is_file()):
            entries.append((f"share/{package_name}/{path.parent}", [str(path)]))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(
        include=[
            "mono_hydra_perception",
            "mono_hydra_perception.*",
            "m2h_hmx_large",
            "m2h_hmx_large.*",
        ]
    ),
    data_files=package_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bavantha Udugama",
    maintainer_email="bavanthau@eng.pdn.ac.lk",
    description="ROS 2 multi-task perception package for Mono Hydra.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "m2h_hmx_large_node = mono_hydra_perception.m2h_hmx_large_node:main",
            "m2h_onnx_node = mono_hydra_perception.m2h_onnx_node:main",
            "temporal_pose_warp_filter_node = mono_hydra_perception.temporal_pose_warp_filter_node:main",
        ],
    },
)
