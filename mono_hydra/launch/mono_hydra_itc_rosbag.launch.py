from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("visualize", default_value="false"),
            DeclareLaunchArgument("sequence_name", default_value="itc_rosbag"),
            DeclareLaunchArgument("output_root", default_value="output"),
            DeclareLaunchArgument("use_hydra_backend", default_value="true"),
            DeclareLaunchArgument("use_perception", default_value="true"),
            DeclareLaunchArgument("use_rvio2_bridge", default_value="true"),
            DeclareLaunchArgument("use_rvio2_backend", default_value="true"),
            DeclareLaunchArgument("use_kimera_vio_ros_node", default_value="true"),
            DeclareLaunchArgument("use_kimera_pose_graph_bridge", default_value="true"),
            DeclareLaunchArgument("start_mesh_marker", default_value="false"),
            DeclareLaunchArgument("perception_backend", default_value="m2h"),
            DeclareLaunchArgument("perception_skip_frequency", default_value="5"),
            DeclareLaunchArgument("perception_input_queue_size", default_value="256"),
            DeclareLaunchArgument("perception_output_queue_size", default_value="10"),
            DeclareLaunchArgument("perception_warn_output_lag_s", default_value="5.0"),
            DeclareLaunchArgument("perception_max_output_lag_s", default_value="0.0"),
            DeclareLaunchArgument("perception_depth_scale", default_value="1.0"),
            DeclareLaunchArgument("perception_image_height", default_value="480"),
            DeclareLaunchArgument("perception_image_width", default_value="640"),
            DeclareLaunchArgument("perception_config_path", default_value=""),
            DeclareLaunchArgument("perception_checkpoint_path", default_value=""),
            DeclareLaunchArgument(
                "onnx_model_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("mono_hydra_perception"),
                        "onnx_models",
                        "scannet_depth_sem_192x256_trt_clean.onnx",
                    ]
                ),
            ),
            DeclareLaunchArgument("onnx_input_width", default_value="256"),
            DeclareLaunchArgument("onnx_input_height", default_value="192"),
            DeclareLaunchArgument("onnx_intra_op_num_threads", default_value="8"),
            DeclareLaunchArgument("onnx_inter_op_num_threads", default_value="1"),
            DeclareLaunchArgument("use_kimera_external_lc_bridge", default_value="false"),
            DeclareLaunchArgument("enable_lcd", default_value="true"),
            DeclareLaunchArgument("kimera_lcd_no_optimize", default_value="true"),
            DeclareLaunchArgument("kimera_lcd_no_detection", default_value="false"),
            DeclareLaunchArgument("kimera_lcd_disable_stereo_match_depth_check", default_value="false"),
            DeclareLaunchArgument("kimera_no_incremental_pose", default_value="false"),
            DeclareLaunchArgument("kimera_publish_tf", default_value="true"),
            DeclareLaunchArgument("kimera_publish_lcd_tf", default_value="true"),
            DeclareLaunchArgument("kimera_publish_camera_tf", default_value="true"),
            DeclareLaunchArgument("rvio2_input_pose_frame", default_value="body"),
            DeclareLaunchArgument("odom_adapter_publish_tf", default_value="false"),
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
                    "use_hydra_backend": LaunchConfiguration("use_hydra_backend"),
                    "use_perception": LaunchConfiguration("use_perception"),
                    "use_kimera_vio_ros_node": LaunchConfiguration("use_kimera_vio_ros_node"),
                    "use_kimera_pose_graph_bridge": LaunchConfiguration("use_kimera_pose_graph_bridge"),
                    "start_mesh_marker": LaunchConfiguration("start_mesh_marker"),
                    "labelspace_name": "nyud20_config",
                    "hydra_input_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_topics.yaml"]
                    ),
                    "hydra_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_topics.yaml"]
                    ),
                    "hydra_labelspace_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_perception"), "config", "label_spaces", "nyud20_config_label_space.yaml"]
                    ),
                    "lcd_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_lcd.yaml"]
                    ),
                    "hydra_exit_after_clock": LaunchConfiguration("hydra_exit_after_clock"),
                    "perception_dataset": "itc",
                    "perception_backend": LaunchConfiguration("perception_backend"),
                    "perception_skip_frequency": LaunchConfiguration("perception_skip_frequency"),
                    "perception_input_queue_size": LaunchConfiguration("perception_input_queue_size"),
                    "perception_output_queue_size": LaunchConfiguration("perception_output_queue_size"),
                    "perception_warn_output_lag_s": LaunchConfiguration("perception_warn_output_lag_s"),
                    "perception_max_output_lag_s": LaunchConfiguration("perception_max_output_lag_s"),
                    "perception_publish_synced_inputs": "true",
                    "perception_synced_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "perception_synced_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "perception_depth_scale": LaunchConfiguration("perception_depth_scale"),
                    "perception_image_height": LaunchConfiguration("perception_image_height"),
                    "perception_image_width": LaunchConfiguration("perception_image_width"),
                    "perception_config_path": LaunchConfiguration("perception_config_path"),
                    "perception_checkpoint_path": LaunchConfiguration("perception_checkpoint_path"),
                    "onnx_model_path": LaunchConfiguration("onnx_model_path"),
                    "onnx_input_width": LaunchConfiguration("onnx_input_width"),
                    "onnx_input_height": LaunchConfiguration("onnx_input_height"),
                    "onnx_intra_op_num_threads": LaunchConfiguration("onnx_intra_op_num_threads"),
                    "onnx_inter_op_num_threads": LaunchConfiguration("onnx_inter_op_num_threads"),
                    "odom_frame": "odom",
                    "publish_world_tf": "false",
                    "camera_frame": "left_cam_kimera",
                    "imu_topic": "/camera/imu",
                    "rgb_topic": "/camera/color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "hydra_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "hydra_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "perception_depth_topic": "/camera/depth_cam/image_raw",
                    "perception_label_topic": "/camera/seg_cam/labels_argmax",
                    "perception_semantic_color_topic": "/camera/seg_cam/image_raw",
                    "depth_topic": "/camera/depth_cam/image_raw",
                    "label_topic": "/camera/seg_cam/labels_argmax",
                    "temporal_fixed_frame": "odom",
                    "temporal_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "use_rvio2_bridge": LaunchConfiguration("use_rvio2_bridge"),
                    "use_rvio2_backend": LaunchConfiguration("use_rvio2_backend"),
                    "rvio2_mono_image_topic": "/rvio2_bridge/cam0/image_raw",
                    "rvio2_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "config", "itc_realsense_848x480.yaml"]
                    ),
                    "rvio2_left_camera_params_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "RealSense_RGBD_RVIO2", "LeftCameraParams.yaml"]
                    ),
                    "kimera_params_folder": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "RealSense_RGBD_RVIO2"]
                    ),
                    "kimera_sensor_params_folder": "",
                    "kimera_flags_folder": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "RealSense_RGBD_RVIO2", "flags"]
                    ),
                    "kimera_log_output_path": PathJoinSubstitution(
                        [LaunchConfiguration("output_root"), LaunchConfiguration("sequence_name"), "vio_logs"]
                    ),
                    "kimera_use_external_odom": "true",
                    "kimera_lcd_no_optimize": LaunchConfiguration("kimera_lcd_no_optimize"),
                    "kimera_lcd_no_detection": LaunchConfiguration("kimera_lcd_no_detection"),
                    "kimera_lcd_disable_stereo_match_depth_check": LaunchConfiguration(
                        "kimera_lcd_disable_stereo_match_depth_check"
                    ),
                    "kimera_no_incremental_pose": LaunchConfiguration("kimera_no_incremental_pose"),
                    "kimera_do_coarse_imu_camera_temporal_sync": "false",
                    "kimera_do_fine_imu_camera_temporal_sync": "false",
                    "kimera_publish_tf": LaunchConfiguration("kimera_publish_tf"),
                    "kimera_publish_lcd_tf": LaunchConfiguration("kimera_publish_lcd_tf"),
                    "kimera_publish_camera_tf": LaunchConfiguration("kimera_publish_camera_tf"),
                    "kimera_left_cam_topic": "/mono_hydra_perception/synced/image_raw",
                    "kimera_rgbd_sync_queue_size": "3000",
                    "kimera_stereo_sync_queue_size": "3000",
                    "rvio2_input_pose_frame": LaunchConfiguration("rvio2_input_pose_frame"),
                    "enable_lcd": LaunchConfiguration("enable_lcd"),
                    "use_kimera_external_lc_bridge": LaunchConfiguration("use_kimera_external_lc_bridge"),
                    "rvio2_trajectory_topic": "/rvio2/trajectory",
                    "odom_adapter_publish_tf": LaunchConfiguration("odom_adapter_publish_tf"),
                    "publish_sensor_static_tf": "false",
                }.items(),
            )
        ]
    )
