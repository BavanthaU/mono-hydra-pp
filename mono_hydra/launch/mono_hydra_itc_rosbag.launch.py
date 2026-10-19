import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _validate_weight_preset(context):
    preset = LaunchConfiguration("perception_dataset").perform(context)
    backend = LaunchConfiguration("perception_backend").perform(context).strip().lower()
    if preset != "itc" and backend not in ("torch", "pytorch", "stock"):
        raise RuntimeError("ITC weight experiments require perception_backend:=torch")
    return []


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
            DeclareLaunchArgument(
                "perception_dataset", default_value="itc",
                choices=["itc", "scannet", "nyud", "nyudv2"],
                description="Torch model preset and semantic label space; sensor setup remains ITC.",
            ),
            DeclareLaunchArgument("perception_skip_frequency", default_value="3"),
            DeclareLaunchArgument("perception_target_fps", default_value="0.0"),
            DeclareLaunchArgument("use_ptc_refinement", default_value="false"),
            DeclareLaunchArgument("ptc_python_path", default_value=os.environ.get("PTC_DEPTH_PYTHONPATH", "")),
            DeclareLaunchArgument("ptc_width", default_value="424"),
            DeclareLaunchArgument("ptc_references", default_value="1"),
            DeclareLaunchArgument("revision_fusion_weighting", default_value="false"),
            DeclareLaunchArgument("revision_object_association", default_value="false"),
            DeclareLaunchArgument("hydra_extra_yaml", default_value="{}"),
            DeclareLaunchArgument("ptc_local", default_value="true"),
            DeclareLaunchArgument("ptc_max_scale_change", default_value="0.10"),
            DeclareLaunchArgument("perception_input_queue_size", default_value="2"),
            DeclareLaunchArgument("perception_output_queue_size", default_value="2"),
            DeclareLaunchArgument("perception_warn_output_lag_s", default_value="0.5"),
            DeclareLaunchArgument("perception_max_output_lag_s", default_value="1.0"),
            DeclareLaunchArgument("perception_depth_scale", default_value="1.0"),
            DeclareLaunchArgument("perception_preserve_aspect_ratio", default_value="false"),
            DeclareLaunchArgument("perception_image_height", default_value="0"),
            DeclareLaunchArgument("perception_image_width", default_value="0"),
            DeclareLaunchArgument("perception_config_path", default_value=""),
            DeclareLaunchArgument("perception_checkpoint_path", default_value=""),
            DeclareLaunchArgument("use_temporal_alignment", default_value="false"),
            DeclareLaunchArgument("temporal_history_size", default_value="3"),
            DeclareLaunchArgument("temporal_max_history_age_s", default_value="0.5"),
            DeclareLaunchArgument("temporal_depth_gate_abs", default_value="0.30"),
            DeclareLaunchArgument("temporal_depth_gate_rel", default_value="0.05"),
            DeclareLaunchArgument("temporal_depth_fusion_mode", default_value="median"),
            DeclareLaunchArgument("temporal_label_min_votes", default_value="2"),
            DeclareLaunchArgument("temporal_fuse_labels", default_value="false"),
            DeclareLaunchArgument(
                "kimera_left_cam_topic", default_value="/mono_hydra_perception/synced/image_raw"
            ),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth_cam/image_raw"),
            DeclareLaunchArgument("label_topic", default_value="/camera/seg_cam/labels_argmax"),
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
            # Match the ROS 1 split: Kimera detects visual loop closures while
            # Hydra consumes those pose-graph edges and performs PGMO. Hydra's
            # separate semantic DSG detector remains opt-in for ITC.
            DeclareLaunchArgument("enable_hydra_lcd", default_value="false"),
            DeclareLaunchArgument("kimera_lcd_no_optimize", default_value="true"),
            DeclareLaunchArgument("kimera_lcd_no_detection", default_value="false"),
            DeclareLaunchArgument("kimera_lcd_disable_stereo_match_depth_check", default_value="false"),
            DeclareLaunchArgument("kimera_no_incremental_pose", default_value="false"),
            DeclareLaunchArgument("kimera_publish_tf", default_value="true"),
            DeclareLaunchArgument("kimera_publish_lcd_tf", default_value="true"),
            DeclareLaunchArgument("kimera_publish_camera_tf", default_value="true"),
            DeclareLaunchArgument("rvio2_input_pose_frame", default_value="body"),
            # classical = rvio2_mono_node (KLT), superpoint = rvio2_sp_mono_node
            DeclareLaunchArgument("rvio2_variant", default_value="superpoint", choices=["classical", "superpoint"]),
            DeclareLaunchArgument(
                "rvio2_sp_config_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mono_hydra_vio"), "config", "itc_realsense_848x480_superpoint.yaml"]
                ),
            ),
            DeclareLaunchArgument("rvio2_sp_tracker_method", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_detector", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_gyro_prediction", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_klt_fallback", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_refine", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_update_descriptor", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_klt_gyro_prediction", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_use_gpu", default_value="-1"),
            DeclareLaunchArgument("rvio2_sp_model_path", default_value=""),
            DeclareLaunchArgument("rvio2_sp_stats_csv", default_value=""),
            # Extra config keys, e.g. "SuperPoint.DetThreshold=0.005;SPTrack.SearchRadius=32"
            DeclareLaunchArgument("rvio2_sp_config_overrides", default_value=""),
            DeclareLaunchArgument("odom_adapter_publish_tf", default_value="false"),
            DeclareLaunchArgument("hydra_exit_after_clock", default_value="false"),
            OpaqueFunction(function=_validate_weight_preset),
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
                    "labelspace_name": PythonExpression([
                        "'scannet20_config' if '", LaunchConfiguration("perception_dataset"),
                        "' == 'scannet' else 'nyud20_config'",
                    ]),
                    "hydra_input_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_topics.yaml"]
                    ),
                    "hydra_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_topics.yaml"]
                    ),
                    "hydra_labelspace_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_perception"), "config", "label_spaces",
                         PythonExpression([
                             "'scannet20_config_label_space.yaml' if '",
                             LaunchConfiguration("perception_dataset"),
                             "' == 'scannet' else 'nyud20_config_label_space.yaml'",
                         ])]
                    ),
                    "lcd_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra"), "config", "hydra_itc_lcd.yaml"]
                    ),
                    "hydra_exit_after_clock": LaunchConfiguration("hydra_exit_after_clock"),
                    "perception_dataset": LaunchConfiguration("perception_dataset"),
                    "perception_backend": LaunchConfiguration("perception_backend"),
                    "perception_skip_frequency": LaunchConfiguration("perception_skip_frequency"),
                    "perception_target_fps": LaunchConfiguration("perception_target_fps"),
                    "use_ptc_refinement": LaunchConfiguration("use_ptc_refinement"),
                    "ptc_python_path": LaunchConfiguration("ptc_python_path"),
                    "ptc_width": LaunchConfiguration("ptc_width"),
                    "ptc_references": LaunchConfiguration("ptc_references"),
                    "revision_fusion_weighting": LaunchConfiguration("revision_fusion_weighting"),
                    "revision_object_association": LaunchConfiguration("revision_object_association"),
                    "hydra_extra_yaml": LaunchConfiguration("hydra_extra_yaml"),
                    "ptc_local": LaunchConfiguration("ptc_local"),
                    "ptc_max_scale_change": LaunchConfiguration("ptc_max_scale_change"),
                    "perception_input_queue_size": LaunchConfiguration("perception_input_queue_size"),
                    "perception_output_queue_size": LaunchConfiguration("perception_output_queue_size"),
                    "perception_warn_output_lag_s": LaunchConfiguration("perception_warn_output_lag_s"),
                    "perception_max_output_lag_s": LaunchConfiguration("perception_max_output_lag_s"),
                    "perception_publish_synced_inputs": "true",
                    "perception_synced_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "perception_synced_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "perception_depth_scale": LaunchConfiguration("perception_depth_scale"),
                    "perception_preserve_aspect_ratio": LaunchConfiguration(
                        "perception_preserve_aspect_ratio"
                    ),
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
                    "perception_depth_topic": LaunchConfiguration("depth_topic"),
                    "perception_label_topic": LaunchConfiguration("label_topic"),
                    "perception_semantic_color_topic": "/camera/seg_cam/image_raw",
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "label_topic": LaunchConfiguration("label_topic"),
                    "use_temporal_alignment": LaunchConfiguration("use_temporal_alignment"),
                    "temporal_history_size": LaunchConfiguration("temporal_history_size"),
                    "temporal_max_history_age_s": LaunchConfiguration("temporal_max_history_age_s"),
                    "temporal_depth_gate_abs": LaunchConfiguration("temporal_depth_gate_abs"),
                    "temporal_depth_gate_rel": LaunchConfiguration("temporal_depth_gate_rel"),
                    "temporal_depth_fusion_mode": LaunchConfiguration("temporal_depth_fusion_mode"),
                    "temporal_label_min_votes": LaunchConfiguration("temporal_label_min_votes"),
                    "temporal_fuse_labels": LaunchConfiguration("temporal_fuse_labels"),
                    "temporal_fixed_frame": "odom",
                    "temporal_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "use_rvio2_bridge": LaunchConfiguration("use_rvio2_bridge"),
                    "use_rvio2_backend": LaunchConfiguration("use_rvio2_backend"),
                    "use_sparse_depth_factors": "false",
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
                    "kimera_left_cam_topic": LaunchConfiguration("kimera_left_cam_topic"),
                    "kimera_rgbd_sync_queue_size": "10",
                    "kimera_stereo_sync_queue_size": "10",
                    "rvio2_input_pose_frame": LaunchConfiguration("rvio2_input_pose_frame"),
                    "rvio2_variant": LaunchConfiguration("rvio2_variant"),
                    "rvio2_sp_config_path": LaunchConfiguration("rvio2_sp_config_path"),
                    "rvio2_sp_tracker_method": LaunchConfiguration("rvio2_sp_tracker_method"),
                    "rvio2_sp_detector": LaunchConfiguration("rvio2_sp_detector"),
                    "rvio2_sp_gyro_prediction": LaunchConfiguration("rvio2_sp_gyro_prediction"),
                    "rvio2_sp_klt_fallback": LaunchConfiguration("rvio2_sp_klt_fallback"),
                    "rvio2_sp_refine": LaunchConfiguration("rvio2_sp_refine"),
                    "rvio2_sp_update_descriptor": LaunchConfiguration("rvio2_sp_update_descriptor"),
                    "rvio2_sp_klt_gyro_prediction": LaunchConfiguration("rvio2_sp_klt_gyro_prediction"),
                    "rvio2_sp_use_gpu": LaunchConfiguration("rvio2_sp_use_gpu"),
                    "rvio2_sp_model_path": LaunchConfiguration("rvio2_sp_model_path"),
                    "rvio2_sp_stats_csv": LaunchConfiguration("rvio2_sp_stats_csv"),
                    "rvio2_sp_config_overrides": LaunchConfiguration("rvio2_sp_config_overrides"),
                    # Include arguments are evaluated in order. Capture the
                    # parent's Kimera switch before reusing enable_lcd for Hydra.
                    "enable_kimera_lcd": LaunchConfiguration("enable_lcd"),
                    "enable_lcd": LaunchConfiguration("enable_hydra_lcd"),
                    "use_kimera_external_lc_bridge": LaunchConfiguration("use_kimera_external_lc_bridge"),
                    "rvio2_trajectory_topic": "/rvio2/trajectory",
                    "odom_adapter_publish_tf": LaunchConfiguration("odom_adapter_publish_tf"),
                    "publish_sensor_static_tf": "false",
                }.items(),
            )
        ]
    )
