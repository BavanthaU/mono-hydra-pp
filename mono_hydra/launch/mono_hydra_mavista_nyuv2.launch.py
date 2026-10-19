from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    input_image_topic = LaunchConfiguration("input_image_topic")
    input_camera_info_topic = LaunchConfiguration("input_camera_info_topic")
    provider = LaunchConfiguration("provider")
    output_depth_topic = LaunchConfiguration("output_depth_topic")
    output_semantic_topic = LaunchConfiguration("output_semantic_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("input_image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument(
                "input_camera_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument("provider", default_value="cuda"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("visualize", default_value="false"),
            DeclareLaunchArgument("sequence_name", default_value="mavista_nyuv2"),
            DeclareLaunchArgument("output_root", default_value="output"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("robot_frame", default_value="base_link_kimera"),
            DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("output_depth_topic", default_value="/hydra/camera/depth/image_rect"),
            DeclareLaunchArgument(
                "output_semantic_topic", default_value="/hydra/camera/semantic/image_raw"
            ),
            DeclareLaunchArgument(
                "output_semantic_visualization_topic",
                default_value="/mavista/semantic/visualization",
            ),
            DeclareLaunchArgument("semantic_confidence_threshold", default_value="0.0"),
            DeclareLaunchArgument("min_depth_m", default_value="0.1"),
            DeclareLaunchArgument("max_depth_m", default_value="10.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "launch", "mono_hydra_scannet.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "visualize": LaunchConfiguration("visualize"),
                    "sequence_name": LaunchConfiguration("sequence_name"),
                    "output_root": LaunchConfiguration("output_root"),
                    "dataset": "nyuv2",
                    "perception_dataset": "nyuv2",
                    "labelspace_name": "nyu40",
                    "hydra_labelspace_path": PathJoinSubstitution(
                        [
                            FindPackageShare("mavista"),
                            "config",
                            "nyu40_hydra_label_space.yaml",
                        ]
                    ),
                    "hydra_input_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_scannet_topics.yaml"]
                    ),
                    "hydra_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_scannet_topics.yaml"]
                    ),
                    "hydra_extra_yaml": "{input: {inputs: {camera: {sensor: {min_range: 0.1, max_range: 10.0}}}}}",
                    "enable_lcd": "false",
                    "use_hydra_backend": "true",
                    "use_perception": "true",
                    "perception_backend": "mavista",
                    "mavista_provider": provider,
                    "mavista_semantic_confidence_threshold": LaunchConfiguration(
                        "semantic_confidence_threshold"
                    ),
                    "mavista_min_depth_m": LaunchConfiguration("min_depth_m"),
                    "mavista_max_depth_m": LaunchConfiguration("max_depth_m"),
                    "rgb_topic": input_image_topic,
                    "camera_info_topic": input_camera_info_topic,
                    "hydra_rgb_topic": input_image_topic,
                    "hydra_camera_info_topic": input_camera_info_topic,
                    "depth_topic": output_depth_topic,
                    "label_topic": output_semantic_topic,
                    "semantic_color_topic": output_semantic_topic,
                    "perception_depth_topic": output_depth_topic,
                    "perception_label_topic": output_semantic_topic,
                    "perception_semantic_color_topic": output_semantic_topic,
                    "mavista_output_depth_topic": output_depth_topic,
                    "mavista_output_semantic_topic": output_semantic_topic,
                    "mavista_output_semantic_visualization_topic": LaunchConfiguration(
                        "output_semantic_visualization_topic"
                    ),
                    "perception_publish_synced_inputs": "false",
                    "use_rvio2_backend": "false",
                    "use_rvio2_bridge": "false",
                    "use_kimera_vio_ros_node": "false",
                    "use_kimera_pose_graph_bridge": "false",
                    "odom_adapter_publish_tf": "false",
                    "publish_world_tf": "false",
                    "publish_sensor_static_tf": "false",
                    "odom_frame": LaunchConfiguration("odom_frame"),
                    "robot_frame": LaunchConfiguration("robot_frame"),
                    "camera_frame": LaunchConfiguration("camera_frame"),
                    "map_frame": LaunchConfiguration("map_frame"),
                }.items(),
            ),
        ]
    )
