from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "mavista"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "LICENSE"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/models", [str(path) for path in Path("models").glob("*")]),
    ],
    install_requires=["setuptools", "numpy>=1.24,<2.0"],
    zip_safe=False,
    maintainer="MAViSTA Maintainer",
    maintainer_email="maintainer@example.com",
    description="MAViSTA ROS 2 inference topics for Hydra",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mavista_node = mavista.node:main",
        ],
    },
)
