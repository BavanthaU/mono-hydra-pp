from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "launch", "mono_hydra_scannet.launch.py"]
                    )
                ),
                launch_arguments={
                    "perception_dataset": "scannet",
                    "labelspace_name": "scannet20_config",
                    "hydra_labelspace_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_perception"), "config", "label_spaces", "scannet20_config_label_space.yaml"]
                    ),
                    "use_rvio2_bridge": "true",
                    "rvio2_trajectory_topic": "/rvio2/trajectory",
                    "rvio2_input_pose_frame": "body",
                    "external_odom_topic": "/external_odometry",
                    "odom_adapter_publish_tf": "true",
                    "odom_adapter_force_frame_ids": "true",
                    "enable_lcd": "true",
                }.items(),
            )
        ]
    )
