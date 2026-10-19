from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("sequence_name", default_value="itc_mavista_manual"),
            DeclareLaunchArgument("output_root", default_value="output"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("visualize", default_value="false"),
            DeclareLaunchArgument("start_mesh_marker", default_value="false"),
            DeclareLaunchArgument("hydra_exit_after_clock", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "launch", "mono_hydra_scannet.launch.py"]
                    )
                ),
                launch_arguments={
                    "dataset": "itc",
                    "sequence_name": LaunchConfiguration("sequence_name"),
                    "output_root": LaunchConfiguration("output_root"),
                    "use_sim_time": "true",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "visualize": LaunchConfiguration("visualize"),
                    "start_mesh_marker": LaunchConfiguration("start_mesh_marker"),
                    "hydra_exit_after_clock": LaunchConfiguration("hydra_exit_after_clock"),
                    "use_hydra_backend": "true",
                    "use_perception": "false",
                    "use_rvio2_backend": "false",
                    "use_rvio2_bridge": "false",
                    "use_vio_feature_interface": "false",
                    "use_odom_adapter": "false",
                    "use_kimera_vio_ros_node": "false",
                    "use_kimera_pose_graph_bridge": "false",
                    "labelspace_name": "nyu40",
                    "hydra_input_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_topics.yaml"]
                    ),
                    "hydra_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_topics.yaml"]
                    ),
                    "hydra_labelspace_path": PathJoinSubstitution(
                        [
                            FindPackageShare("mavista"),
                            "config",
                            "nyu40_hydra_label_space.yaml",
                        ]
                    ),
                    "lcd_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_lcd.yaml"]
                    ),
                    "hydra_extra_yaml": "{input: {inputs: {camera: {sensor: {min_range: 0.1, max_range: 10.0}}}}}",
                    "rgb_topic": "/camera/color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "hydra_rgb_topic": "/camera/color/image_raw",
                    "hydra_camera_info_topic": "/camera/color/camera_info",
                    "perception_depth_topic": "/hydra/camera/depth/image_rect",
                    "perception_label_topic": "/hydra/camera/semantic/image_raw",
                    "depth_topic": "/hydra/camera/depth/image_rect",
                    "label_topic": "/hydra/camera/semantic/image_raw",
                    "use_temporal_alignment": "false",
                    "odom_frame": "odom",
                    "robot_frame": "base_link_kimera",
                    "camera_frame": "left_cam_kimera",
                    "map_frame": "map",
                    "publish_world_tf": "false",
                    "publish_sensor_static_tf": "false",
                    "imu_topic": "/camera/imu",
                }.items(),
            ),
        ]
    )
