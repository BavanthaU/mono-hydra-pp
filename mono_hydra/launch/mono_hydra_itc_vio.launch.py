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
            DeclareLaunchArgument("enable_lcd", default_value="true"),
            DeclareLaunchArgument("kimera_lcd_no_optimize", default_value="true"),
            DeclareLaunchArgument("rvio2_input_pose_frame", default_value="body"),
            DeclareLaunchArgument("odom_adapter_publish_tf", default_value="false"),
            DeclareLaunchArgument("kimera_publish_tf", default_value="true"),
            DeclareLaunchArgument("kimera_publish_lcd_tf", default_value="true"),
            DeclareLaunchArgument("kimera_publish_camera_tf", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "launch", "mono_hydra_itc_rosbag.launch.py"]
                    )
                ),
                launch_arguments={
                    "sequence_name": LaunchConfiguration("sequence_name"),
                    "output_root": LaunchConfiguration("output_root"),
                    "use_hydra_backend": "false",
                    "use_perception": "false",
                    "use_rvio2_backend": "true",
                    "use_rvio2_bridge": "true",
                    "use_kimera_vio_ros_node": "true",
                    "use_kimera_pose_graph_bridge": "true",
                    "start_mesh_marker": "false",
                    "use_rviz": "false",
                    "visualize": "false",
                    "enable_lcd": LaunchConfiguration("enable_lcd"),
                    "kimera_lcd_no_optimize": LaunchConfiguration("kimera_lcd_no_optimize"),
                    "rvio2_input_pose_frame": LaunchConfiguration("rvio2_input_pose_frame"),
                    "odom_adapter_publish_tf": LaunchConfiguration("odom_adapter_publish_tf"),
                    "kimera_publish_tf": LaunchConfiguration("kimera_publish_tf"),
                    "kimera_publish_lcd_tf": LaunchConfiguration("kimera_publish_lcd_tf"),
                    "kimera_publish_camera_tf": LaunchConfiguration("kimera_publish_camera_tf"),
                }.items(),
            ),
        ]
    )
