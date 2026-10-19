import os
import site
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _python_cuda_library_path() -> str:
    roots = []
    for getter in (site.getusersitepackages, site.getsitepackages):
        try:
            value = getter()
        except Exception:
            continue
        roots.extend(value if isinstance(value, list) else [value])
    paths = []
    for root in roots:
        paths.extend(str(path) for path in sorted((Path(root) / "nvidia").glob("*/lib")) if path.is_dir())
    existing = os.environ.get("LD_LIBRARY_PATH", "")
    if existing:
        paths.append(existing)
    return ":".join(dict.fromkeys(paths))


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("mavista"))
    arguments = [
        DeclareLaunchArgument("input_image_topic", default_value="/camera/color/image_rect"),
        DeclareLaunchArgument(
            "input_camera_info_topic", default_value="/camera/color/camera_info"
        ),
        DeclareLaunchArgument("provider", default_value="auto"),
        DeclareLaunchArgument("input_qos_reliability", default_value="best_effort"),
        DeclareLaunchArgument("output_depth_topic", default_value="/hydra/camera/depth/image_rect"),
        DeclareLaunchArgument(
            "output_semantic_topic", default_value="/hydra/camera/semantic/image_raw"
        ),
        DeclareLaunchArgument("semantic_confidence_threshold", default_value="0.0"),
        DeclareLaunchArgument(
            "output_semantic_visualization_topic", default_value="/mavista/semantic/visualization"
        ),
        DeclareLaunchArgument("publish_diagnostics", default_value="false"),
        DeclareLaunchArgument("publish_semantic_visualization", default_value="false"),
    ]
    node = Node(
        package="mavista",
        executable="mavista_node",
        name="mavista",
        output="screen",
        additional_env={"LD_LIBRARY_PATH": _python_cuda_library_path()},
        parameters=[
            str(share / "config" / "mavista.yaml"),
            {
                "input_image_topic": LaunchConfiguration("input_image_topic"),
                "input_camera_info_topic": LaunchConfiguration("input_camera_info_topic"),
                "provider": LaunchConfiguration("provider"),
                "input_qos_reliability": LaunchConfiguration("input_qos_reliability"),
                "output_depth_topic": LaunchConfiguration("output_depth_topic"),
                "output_semantic_topic": LaunchConfiguration("output_semantic_topic"),
                "semantic_confidence_threshold": ParameterValue(
                    LaunchConfiguration("semantic_confidence_threshold"), value_type=float
                ),
                "output_semantic_visualization_topic": LaunchConfiguration(
                    "output_semantic_visualization_topic"
                ),
                "publish_diagnostics": ParameterValue(
                    LaunchConfiguration("publish_diagnostics"), value_type=bool
                ),
                "publish_semantic_visualization": ParameterValue(
                    LaunchConfiguration("publish_semantic_visualization"), value_type=bool
                ),
            },
        ],
    )
    return LaunchDescription([*arguments, node])
