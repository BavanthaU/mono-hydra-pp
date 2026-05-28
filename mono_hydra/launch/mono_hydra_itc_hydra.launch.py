from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sequence_name", default_value="itc_ros1_style_manual"),
            DeclareLaunchArgument("output_root", default_value="output"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("visualize", default_value="false"),
            DeclareLaunchArgument("start_mesh_marker", default_value="false"),
            DeclareLaunchArgument("hydra_exit_after_clock", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "launch", "mono_hydra_itc_rosbag.launch.py"]
                    )
                ),
                launch_arguments={
                    "sequence_name": LaunchConfiguration("sequence_name"),
                    "output_root": LaunchConfiguration("output_root"),
                    "use_hydra_backend": "true",
                    "use_perception": "false",
                    "use_rvio2_backend": "false",
                    "use_rvio2_bridge": "false",
                    "use_kimera_vio_ros_node": "false",
                    "use_kimera_pose_graph_bridge": "false",
                    "start_mesh_marker": LaunchConfiguration("start_mesh_marker"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "visualize": LaunchConfiguration("visualize"),
                    "hydra_exit_after_clock": LaunchConfiguration("hydra_exit_after_clock"),
                }.items(),
            ),
        ]
    )
