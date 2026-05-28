import os
from pathlib import Path
import site

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return str(value).lower() in ("1", "true", "yes", "on")


def _python_cuda_library_path() -> str:
    roots = []
    for getter in (site.getusersitepackages, site.getsitepackages):
        try:
            value = getter()
        except Exception:
            continue
        roots.extend(value if isinstance(value, list) else [value])

    dirs = []
    for root in roots:
        nvidia_root = Path(root) / "nvidia"
        if nvidia_root.is_dir():
            dirs.extend(str(path) for path in sorted(nvidia_root.glob("*/lib")) if path.is_dir())

    existing = os.environ.get("LD_LIBRARY_PATH", "")
    if existing:
        dirs.append(existing)
    return ":".join(dict.fromkeys(dirs))


def _official_hydra_group(context):
    if not _as_bool(LaunchConfiguration("use_hydra_backend").perform(context)):
        return []
    try:
        from ament_index_python.packages import get_package_share_directory

        hydra_ros_share = Path(get_package_share_directory("hydra_ros"))
        hydra_share = Path(get_package_share_directory("hydra"))
    except Exception as exc:
        raise RuntimeError(f"Official Hydra ROS 2 package is required in this overlay: {exc}") from exc

    launch_file = hydra_ros_share / "launch" / "hydra.launch.yaml"
    if not launch_file.exists():
        raise RuntimeError(f"Official Hydra launch file not found: {launch_file}")

    mono_hydra_share = Path(FindPackageShare("mono_hydra").perform(context))
    labelspace_path = LaunchConfiguration("hydra_labelspace_path").perform(context)
    if not labelspace_path:
        labelspace_path = str(mono_hydra_share / "config" / "scannet20_label_space.yaml")
    lcd_config_path = LaunchConfiguration("lcd_config_path").perform(context)
    if not lcd_config_path:
        lcd_config_path = str(hydra_share / "config" / "lcd" / "default.yaml")
    input_config_path = LaunchConfiguration("hydra_input_config_path").perform(context)
    if not input_config_path:
        input_config_path = str(mono_hydra_share / "config" / "hydra_scannet_topics.yaml")
    hydra_config_path = LaunchConfiguration("hydra_config_path").perform(context)
    if not hydra_config_path:
        hydra_config_path = str(mono_hydra_share / "config" / "hydra_scannet_topics.yaml")
    start_visualizer = (
        "true"
        if _as_bool(LaunchConfiguration("use_rviz").perform(context))
        else LaunchConfiguration("visualize")
    )
    depth_topic = (
        LaunchConfiguration("temporal_depth_topic").perform(context)
        if _as_bool(LaunchConfiguration("use_temporal_alignment").perform(context))
        else LaunchConfiguration("depth_topic").perform(context)
    )
    label_topic = (
        LaunchConfiguration("temporal_label_topic").perform(context)
        if _as_bool(LaunchConfiguration("use_temporal_alignment").perform(context))
        else LaunchConfiguration("label_topic").perform(context)
    )

    return [
        GroupAction(
            [
                SetRemap(src="hydra/input/camera/rgb/image_raw", dst=LaunchConfiguration("hydra_rgb_topic")),
                SetRemap(
                    src="hydra/input/camera/rgb/camera_info",
                    dst=LaunchConfiguration("hydra_camera_info_topic"),
                ),
                SetRemap(src="hydra/input/camera/depth_registered/image_rect", dst=depth_topic),
                SetRemap(src="hydra/input/camera/semantic/image_raw", dst=label_topic),
                SetRemap(src="/hydra_visualizer/graph", dst="/hydra_dsg_visualizer/dsg_markers"),
                SetRemap(src="/hydra_visualizer/agent_poses", dst="/hydra_dsg_visualizer/agent_poses"),
                SetRemap(src="/hydra_visualizer/dynamic_objects", dst="/hydra_dsg_visualizer/dynamic_layers_viz"),
                SetRemap(src="/hydra_visualizer/mesh", dst="/hydra_dsg_visualizer/dsg_mesh"),
                SetRemap(src="/hydra_visualizer/robot0/dsg_mesh", dst="/hydra_dsg_visualizer/dsg_mesh"),
                SetRemap(
                    src="/mono_hydra_vio_ros/pose_graph",
                    dst=LaunchConfiguration("kimera_pose_graph_incremental_topic"),
                ),
                SetRemap(
                    src="/hydra/backend/deformation_graph_mesh_mesh",
                    dst="/incremental_dsg_builder_node/pgmo/deformation_graph_mesh_mesh",
                ),
                SetRemap(
                    src="/hydra/backend/deformation_graph_pose_mesh",
                    dst="/incremental_dsg_builder_node/pgmo/deformation_graph_pose_mesh",
                ),
                IncludeLaunchDescription(
                    AnyLaunchDescriptionSource(str(launch_file)),
                    launch_arguments={
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                        "dataset": LaunchConfiguration("dataset"),
                        "labelspace": LaunchConfiguration("labelspace_name"),
                        "sensor_frame": LaunchConfiguration("camera_frame"),
                        "robot_frame": LaunchConfiguration("robot_frame"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "input_config_path": input_config_path,
                        "hydra_config_path": hydra_config_path,
                        "labelspace_path": labelspace_path,
                        "enable_lcd": LaunchConfiguration("enable_lcd"),
                        "lcd_config_path": lcd_config_path,
                        "exit_after_clock": LaunchConfiguration("hydra_exit_after_clock"),
                        "log_name": LaunchConfiguration("sequence_name"),
                        "log_path": PathJoinSubstitution(
                            [LaunchConfiguration("output_root"), LaunchConfiguration("sequence_name")]
                        ),
                        "start_visualizer": start_visualizer,
                        "start_visualizer_rviz": "false",
                        "visualizer_config_path": str(mono_hydra_share / "config" / "hydra_visualizer_complete.yaml"),
                    }.items(),
                ),
            ]
        )
    ]


def _perception_nodes(context):
    if not _as_bool(LaunchConfiguration("use_perception").perform(context)):
        return []

    backend = LaunchConfiguration("perception_backend").perform(context).strip().lower()
    perception_depth_topic = LaunchConfiguration("perception_depth_topic").perform(context).strip()
    if not perception_depth_topic:
        perception_depth_topic = LaunchConfiguration("depth_topic").perform(context)
    perception_label_topic = LaunchConfiguration("perception_label_topic").perform(context).strip()
    if not perception_label_topic:
        perception_label_topic = LaunchConfiguration("label_topic").perform(context)
    perception_semantic_color_topic = LaunchConfiguration("perception_semantic_color_topic").perform(context).strip()
    if not perception_semantic_color_topic:
        perception_semantic_color_topic = LaunchConfiguration("semantic_color_topic").perform(context)
    publish_color_semantic = (
        _as_bool(LaunchConfiguration("perception_publish_color_semantic").perform(context))
        or _as_bool(LaunchConfiguration("use_rviz").perform(context))
    )
    if backend in ("m2h", "legacy", "original", "original_m2h"):
        return [
            Node(
                package="mono_hydra_perception",
                executable="m2h_legacy_node",
                name="mono_hydra_perception",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "image_topic": LaunchConfiguration("rgb_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "image_depth_topic": perception_depth_topic,
                        "image_semantic_topic": perception_semantic_color_topic,
                        "label_ids_topic": perception_label_topic,
                        "input_queue_size": ParameterValue(
                            LaunchConfiguration("perception_input_queue_size"), value_type=int
                        ),
                        "output_queue_size": ParameterValue(
                            LaunchConfiguration("perception_output_queue_size"), value_type=int
                        ),
                        "warn_output_lag_s": ParameterValue(
                            LaunchConfiguration("perception_warn_output_lag_s"), value_type=float
                        ),
                        "max_output_lag_s": ParameterValue(
                            LaunchConfiguration("perception_max_output_lag_s"), value_type=float
                        ),
                        "publish_synced_inputs": ParameterValue(
                            LaunchConfiguration("perception_publish_synced_inputs"), value_type=bool
                        ),
                        "synced_rgb_topic": LaunchConfiguration("perception_synced_rgb_topic"),
                        "synced_camera_info_topic": LaunchConfiguration("perception_synced_camera_info_topic"),
                        "publish_label_ids": True,
                        "publish_color_semantic": ParameterValue(publish_color_semantic, value_type=bool),
                        "model_path": LaunchConfiguration("m2h_model_path"),
                        "feed_width": ParameterValue(LaunchConfiguration("m2h_feed_width"), value_type=int),
                        "feed_height": ParameterValue(LaunchConfiguration("m2h_feed_height"), value_type=int),
                        "skip_frequency": ParameterValue(LaunchConfiguration("perception_skip_frequency"), value_type=int),
                        "arch_name": LaunchConfiguration("m2h_arch_name"),
                        "model_variant": LaunchConfiguration("m2h_model_variant"),
                        "num_classes": ParameterValue(LaunchConfiguration("m2h_num_classes"), value_type=int),
                        "depth_output_scale": ParameterValue(
                            LaunchConfiguration("m2h_depth_output_scale"), value_type=float
                        ),
                        "color20_mat_filepath": LaunchConfiguration("m2h_color20_mat_filepath"),
                        "objects40_csv_mapping": LaunchConfiguration("m2h_objects40_csv_mapping"),
                    }
                ],
            )
        ]

    if backend in ("torch", "pytorch", "stock"):
        return [
            Node(
                package="mono_hydra_perception",
                executable="m2h_hmx_large_node",
                name="mono_hydra_perception",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "dataset": LaunchConfiguration("perception_dataset"),
                        "device": LaunchConfiguration("perception_device"),
                        "half": ParameterValue(LaunchConfiguration("perception_half"), value_type=bool),
                        "skip_frequency": ParameterValue(LaunchConfiguration("perception_skip_frequency"), value_type=int),
                        "depth_scale": ParameterValue(LaunchConfiguration("perception_depth_scale"), value_type=float),
                        "image_height": ParameterValue(LaunchConfiguration("perception_image_height"), value_type=int),
                        "image_width": ParameterValue(LaunchConfiguration("perception_image_width"), value_type=int),
                        "image_topic": LaunchConfiguration("rgb_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "image_depth_topic": perception_depth_topic,
                        "image_semantic_topic": perception_semantic_color_topic,
                        "label_ids_topic": perception_label_topic,
                        "input_queue_size": ParameterValue(
                            LaunchConfiguration("perception_input_queue_size"), value_type=int
                        ),
                        "output_queue_size": ParameterValue(
                            LaunchConfiguration("perception_output_queue_size"), value_type=int
                        ),
                        "warn_output_lag_s": ParameterValue(
                            LaunchConfiguration("perception_warn_output_lag_s"), value_type=float
                        ),
                        "max_output_lag_s": ParameterValue(
                            LaunchConfiguration("perception_max_output_lag_s"), value_type=float
                        ),
                        "publish_synced_inputs": ParameterValue(
                            LaunchConfiguration("perception_publish_synced_inputs"), value_type=bool
                        ),
                        "synced_rgb_topic": LaunchConfiguration("perception_synced_rgb_topic"),
                        "synced_camera_info_topic": LaunchConfiguration("perception_synced_camera_info_topic"),
                        "publish_label_ids": True,
                        "publish_color_semantic": ParameterValue(publish_color_semantic, value_type=bool),
                        "config_path": LaunchConfiguration("perception_config_path"),
                        "checkpoint_path": LaunchConfiguration("perception_checkpoint_path"),
                        "label_mapping_yaml": LaunchConfiguration("perception_label_mapping_yaml"),
                        "color_map_path": LaunchConfiguration("perception_color_map_path"),
                    }
                ],
            )
        ]

    if backend == "onnx":
        perception_share = Path(FindPackageShare("mono_hydra_perception").perform(context))
        model_path = LaunchConfiguration("onnx_model_path").perform(context)
        if not model_path:
            model_path = str(perception_share / "onnx_models" / "scannet_depth_sem_320x416.onnx")
        return [
            Node(
                package="mono_hydra_perception",
                executable="m2h_onnx_node",
                name="mono_hydra_perception",
                output="screen",
                additional_env={"LD_LIBRARY_PATH": _python_cuda_library_path()},
                parameters=[
                    {
                        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                        "model_path": model_path,
                        "image_topic": LaunchConfiguration("rgb_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "image_depth_topic": perception_depth_topic,
                        "image_semantic_topic": perception_semantic_color_topic,
                        "label_ids_topic": perception_label_topic,
                        "input_queue_size": ParameterValue(
                            LaunchConfiguration("perception_input_queue_size"), value_type=int
                        ),
                        "output_queue_size": ParameterValue(
                            LaunchConfiguration("perception_output_queue_size"), value_type=int
                        ),
                        "warn_output_lag_s": ParameterValue(
                            LaunchConfiguration("perception_warn_output_lag_s"), value_type=float
                        ),
                        "max_output_lag_s": ParameterValue(
                            LaunchConfiguration("perception_max_output_lag_s"), value_type=float
                        ),
                        "publish_synced_inputs": ParameterValue(
                            LaunchConfiguration("perception_publish_synced_inputs"), value_type=bool
                        ),
                        "synced_rgb_topic": LaunchConfiguration("perception_synced_rgb_topic"),
                        "synced_camera_info_topic": LaunchConfiguration("perception_synced_camera_info_topic"),
                        "publish_color_semantic": ParameterValue(publish_color_semantic, value_type=bool),
                        "input_width": ParameterValue(LaunchConfiguration("onnx_input_width"), value_type=int),
                        "input_height": ParameterValue(LaunchConfiguration("onnx_input_height"), value_type=int),
                        "num_classes": ParameterValue(LaunchConfiguration("onnx_num_classes"), value_type=int),
                        "depth_scale": ParameterValue(LaunchConfiguration("perception_depth_scale"), value_type=float),
                        "inference_provider": LaunchConfiguration("onnx_provider"),
                        "skip_frequency": ParameterValue(LaunchConfiguration("perception_skip_frequency"), value_type=int),
                        "intra_op_num_threads": ParameterValue(
                            LaunchConfiguration("onnx_intra_op_num_threads"), value_type=int
                        ),
                        "inter_op_num_threads": ParameterValue(
                            LaunchConfiguration("onnx_inter_op_num_threads"), value_type=int
                        ),
                    }
                ],
            )
        ]

    raise RuntimeError(f"Unknown perception_backend '{backend}'. Use 'm2h', 'torch', or 'onnx'.")


def _temporal_alignment_node(context):
    if not _as_bool(LaunchConfiguration("use_temporal_alignment").perform(context)):
        return []
    input_depth_topic = LaunchConfiguration("temporal_input_depth_topic").perform(context).strip()
    if not input_depth_topic:
        input_depth_topic = LaunchConfiguration("perception_depth_topic").perform(context).strip()
    if not input_depth_topic:
        input_depth_topic = LaunchConfiguration("depth_topic").perform(context)
    input_label_topic = LaunchConfiguration("temporal_input_label_topic").perform(context).strip()
    if not input_label_topic:
        input_label_topic = LaunchConfiguration("perception_label_topic").perform(context).strip()
    if not input_label_topic:
        input_label_topic = LaunchConfiguration("label_topic").perform(context)
    return [
        Node(
            package="mono_hydra_perception",
            executable="temporal_pose_warp_filter_node",
            name="temporal_pose_warp_filter",
            output="screen",
            parameters=[
                {
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "fixed_frame": LaunchConfiguration("temporal_fixed_frame"),
                    "camera_frame": LaunchConfiguration("camera_frame"),
                    "camera_info_topic": LaunchConfiguration("temporal_camera_info_topic"),
                    "depth_topic": input_depth_topic,
                    "label_topic": input_label_topic,
                    "output_depth_topic": LaunchConfiguration("temporal_depth_topic"),
                    "output_label_topic": LaunchConfiguration("temporal_label_topic"),
                    "output_color_topic": LaunchConfiguration("temporal_color_topic"),
                    "output_mask_topic": LaunchConfiguration("temporal_mask_topic"),
                    "publish_color_semantic": ParameterValue(
                        LaunchConfiguration("temporal_publish_color_semantic"), value_type=bool
                    ),
                    "color_map_path": LaunchConfiguration("perception_color_map_path"),
                    "history_size": ParameterValue(LaunchConfiguration("temporal_history_size"), value_type=int),
                    "max_history_age_s": ParameterValue(
                        LaunchConfiguration("temporal_max_history_age_s"), value_type=float
                    ),
                    "history_use_fused": ParameterValue(LaunchConfiguration("temporal_history_use_fused"), value_type=bool),
                    "depth_gate_abs": ParameterValue(LaunchConfiguration("temporal_depth_gate_abs"), value_type=float),
                    "depth_gate_rel": ParameterValue(LaunchConfiguration("temporal_depth_gate_rel"), value_type=float),
                    "depth_fusion_mode": LaunchConfiguration("temporal_depth_fusion_mode"),
                    "depth_alpha": ParameterValue(LaunchConfiguration("temporal_depth_alpha"), value_type=float),
                    "label_min_votes": ParameterValue(LaunchConfiguration("temporal_label_min_votes"), value_type=int),
                    "dynamic_labels": LaunchConfiguration("temporal_dynamic_labels"),
                    "ignore_labels": LaunchConfiguration("temporal_ignore_labels"),
                    "sync_queue": ParameterValue(LaunchConfiguration("temporal_sync_queue"), value_type=int),
                    "sync_slop": ParameterValue(LaunchConfiguration("temporal_sync_slop"), value_type=float),
                    "tf_timeout": ParameterValue(LaunchConfiguration("temporal_tf_timeout"), value_type=float),
                    "tf_buffer_size": ParameterValue(LaunchConfiguration("temporal_tf_buffer_size"), value_type=float),
                    "pose_quality_mode": LaunchConfiguration("temporal_pose_quality_mode"),
                    "pose_quality_odom_topic": "/mono_hydra_vio/odometry",
                }
            ],
        )
    ]


def generate_launch_description():
    effective_depth_topic = PythonExpression(
        [
            "'",
            LaunchConfiguration("temporal_depth_topic"),
            "' if '",
            LaunchConfiguration("use_temporal_alignment"),
            "'.lower() in ('1', 'true', 'yes', 'on') else '",
            LaunchConfiguration("depth_topic"),
            "'",
        ]
    )
    effective_label_topic = PythonExpression(
        [
            "'",
            LaunchConfiguration("temporal_label_topic"),
            "' if '",
            LaunchConfiguration("use_temporal_alignment"),
            "'.lower() in ('1', 'true', 'yes', 'on') else '",
            LaunchConfiguration("label_topic"),
            "'",
        ]
    )
    effective_semantic_color_topic = PythonExpression(
        [
            "'",
            LaunchConfiguration("temporal_color_topic"),
            "' if '",
            LaunchConfiguration("use_temporal_alignment"),
            "'.lower() in ('1', 'true', 'yes', 'on') else '",
            LaunchConfiguration("semantic_color_topic"),
            "'",
        ]
    )
    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("visualize", default_value="false"),
        DeclareLaunchArgument("start_mesh_marker", default_value="false"),
        DeclareLaunchArgument("mesh_marker_input_topic", default_value="/hydra/backend/dsg_mesh"),
        DeclareLaunchArgument("mesh_alpha", default_value="0.92"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([FindPackageShare("mono_hydra"), "rviz", "mono_hydra_ros2.rviz"]),
        ),
        DeclareLaunchArgument("dataset", default_value="scannet"),
        DeclareLaunchArgument("sequence_name", default_value="scannet"),
        DeclareLaunchArgument("labelspace_name", default_value="scannet20_config"),
        DeclareLaunchArgument("hydra_labelspace_path", default_value=""),
        DeclareLaunchArgument("hydra_input_config_path", default_value=""),
        DeclareLaunchArgument("hydra_config_path", default_value=""),
        DeclareLaunchArgument("enable_lcd", default_value="true"),
        DeclareLaunchArgument("lcd_config_path", default_value=""),
        DeclareLaunchArgument("hydra_exit_after_clock", default_value="false"),
        DeclareLaunchArgument("use_hydra_backend", default_value="true"),
        DeclareLaunchArgument("use_perception", default_value="true"),
        DeclareLaunchArgument("perception_backend", default_value="torch"),
        DeclareLaunchArgument("perception_dataset", default_value="scannet"),
        DeclareLaunchArgument("perception_device", default_value="auto"),
        DeclareLaunchArgument("perception_half", default_value="true"),
        DeclareLaunchArgument("perception_depth_scale", default_value="1.427"),
        DeclareLaunchArgument("perception_image_height", default_value="0"),
        DeclareLaunchArgument("perception_image_width", default_value="0"),
        DeclareLaunchArgument("perception_publish_color_semantic", default_value="false"),
        DeclareLaunchArgument("perception_skip_frequency", default_value="3"),
        DeclareLaunchArgument("perception_input_queue_size", default_value="256"),
        DeclareLaunchArgument("perception_output_queue_size", default_value="10"),
        DeclareLaunchArgument("perception_warn_output_lag_s", default_value="5.0"),
        DeclareLaunchArgument("perception_max_output_lag_s", default_value="0.0"),
        DeclareLaunchArgument("perception_depth_topic", default_value="/camera/depth_cam/image_raw"),
        DeclareLaunchArgument("perception_label_topic", default_value="/camera/seg_cam/labels_argmax"),
        DeclareLaunchArgument("perception_semantic_color_topic", default_value="/camera/seg_cam/image_raw"),
        DeclareLaunchArgument("perception_publish_synced_inputs", default_value="false"),
        DeclareLaunchArgument("perception_synced_rgb_topic", default_value="/mono_hydra_perception/synced/image_raw"),
        DeclareLaunchArgument(
            "perception_synced_camera_info_topic",
            default_value="/mono_hydra_perception/synced/camera_info",
        ),
        DeclareLaunchArgument("perception_config_path", default_value=""),
        DeclareLaunchArgument("perception_checkpoint_path", default_value=""),
        DeclareLaunchArgument("perception_label_mapping_yaml", default_value=""),
        DeclareLaunchArgument("perception_color_map_path", default_value=""),
        DeclareLaunchArgument(
            "m2h_model_path",
            default_value=PathJoinSubstitution([FindPackageShare("mono_hydra_perception"), "weights", "m2h_indoor.pt"]),
        ),
        DeclareLaunchArgument("m2h_feed_width", default_value="256"),
        DeclareLaunchArgument("m2h_feed_height", default_value="256"),
        DeclareLaunchArgument("m2h_arch_name", default_value="vit_small"),
        DeclareLaunchArgument("m2h_model_variant", default_value="default"),
        DeclareLaunchArgument("m2h_num_classes", default_value="41"),
        DeclareLaunchArgument("m2h_depth_output_scale", default_value="0.967"),
        DeclareLaunchArgument(
            "m2h_color20_mat_filepath",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mono_hydra_perception"), "config", "colors", "color_config20.mat"]
            ),
        ),
        DeclareLaunchArgument(
            "m2h_objects40_csv_mapping",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mono_hydra_perception"), "config", "colors", "nyud40_config.yaml"]
            ),
        ),
        DeclareLaunchArgument("onnx_model_path", default_value=""),
        DeclareLaunchArgument("onnx_input_width", default_value="416"),
        DeclareLaunchArgument("onnx_input_height", default_value="320"),
        DeclareLaunchArgument("onnx_num_classes", default_value="20"),
        DeclareLaunchArgument("onnx_provider", default_value="auto"),
        DeclareLaunchArgument("onnx_intra_op_num_threads", default_value="8"),
        DeclareLaunchArgument("onnx_inter_op_num_threads", default_value="1"),
        DeclareLaunchArgument("use_rvio2_backend", default_value="false"),
        DeclareLaunchArgument("use_rvio2_bridge", default_value="false"),
        DeclareLaunchArgument("odom_adapter_publish_tf", default_value="true"),
        DeclareLaunchArgument("odom_adapter_force_frame_ids", default_value="true"),
        DeclareLaunchArgument("use_kimera_vio_ros_node", default_value="true"),
        DeclareLaunchArgument(
            "kimera_params_folder",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mono_hydra_vio"), "params", "ScanNet_depth_factors_off"]
            ),
        ),
        DeclareLaunchArgument(
            "kimera_sensor_params_folder",
            default_value=PathJoinSubstitution([FindPackageShare("mono_hydra_vio"), "params", "ScanNet"]),
        ),
        DeclareLaunchArgument(
            "kimera_flags_folder",
            default_value=PathJoinSubstitution([FindPackageShare("mono_hydra_vio"), "params", "ScanNet", "flags"]),
        ),
        DeclareLaunchArgument(
            "kimera_vocabulary_path",
            default_value=PathJoinSubstitution([FindPackageShare("mono_hydra_vio"), "vocabulary", "ORBvoc.yml"]),
        ),
        DeclareLaunchArgument(
            "kimera_log_output_path",
            default_value="output/kimera_vio",
        ),
        DeclareLaunchArgument("kimera_log_output", default_value="true"),
        DeclareLaunchArgument("kimera_use_external_odom", default_value="true"),
        DeclareLaunchArgument("kimera_lcd_no_optimize", default_value="false"),
        DeclareLaunchArgument("kimera_lcd_no_detection", default_value="false"),
        DeclareLaunchArgument("kimera_lcd_disable_stereo_match_depth_check", default_value="false"),
        DeclareLaunchArgument("kimera_no_incremental_pose", default_value="false"),
        DeclareLaunchArgument("kimera_do_coarse_imu_camera_temporal_sync", default_value="false"),
        DeclareLaunchArgument("kimera_do_fine_imu_camera_temporal_sync", default_value="false"),
        DeclareLaunchArgument("kimera_publish_tf", default_value="false"),
        DeclareLaunchArgument("kimera_publish_lcd_tf", default_value="false"),
        DeclareLaunchArgument("kimera_force_same_image_timestamp", default_value="true"),
        DeclareLaunchArgument("kimera_publish_camera_tf", default_value="false"),
        DeclareLaunchArgument("kimera_left_cam_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("kimera_right_cam_topic", default_value=""),
        DeclareLaunchArgument("kimera_rgbd_sync_queue_size", default_value="10"),
        DeclareLaunchArgument("kimera_stereo_sync_queue_size", default_value="10"),
        DeclareLaunchArgument("publish_world_tf", default_value="true"),
        DeclareLaunchArgument("publish_sensor_static_tf", default_value="true"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="scannet_world"),
        DeclareLaunchArgument("robot_frame", default_value="base_link_kimera"),
        DeclareLaunchArgument("camera_frame", default_value="scannet_camera"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/aligned"),
        DeclareLaunchArgument("external_odom_topic", default_value="/external_odometry"),
        DeclareLaunchArgument("rvio2_trajectory_topic", default_value="/rvio2/trajectory"),
        DeclareLaunchArgument("rvio2_config_path", default_value=""),
        DeclareLaunchArgument("rvio2_mono_image_topic", default_value="/rvio2/cam0/image_raw"),
        DeclareLaunchArgument(
            "rvio2_left_camera_params_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mono_hydra_vio"), "params", "ScanNet", "LeftCameraParams.yaml"]
            ),
        ),
        DeclareLaunchArgument("rvio2_input_pose_frame", default_value="sensor"),
        DeclareLaunchArgument("use_kimera_pose_graph_bridge", default_value="true"),
        DeclareLaunchArgument("kimera_pose_graph_topic", default_value="/mono_hydra_vio_ros/pose_graph"),
        DeclareLaunchArgument(
            "kimera_pose_graph_incremental_topic",
            default_value="/mono_hydra_vio_ros/pose_graph_incremental",
        ),
        DeclareLaunchArgument("use_kimera_external_lc_bridge", default_value="false"),
        DeclareLaunchArgument("external_loop_closures_topic", default_value="/hydra/external_loop_closures"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth_cam/image_raw"),
        DeclareLaunchArgument("label_topic", default_value="/camera/seg_cam/labels_argmax"),
        DeclareLaunchArgument("semantic_color_topic", default_value="/camera/seg_cam/image_raw"),
        DeclareLaunchArgument("use_temporal_alignment", default_value="false"),
        DeclareLaunchArgument("temporal_depth_topic", default_value="/temporally_aligned/depth"),
        DeclareLaunchArgument("temporal_label_topic", default_value="/temporally_aligned/labels_argmax"),
        DeclareLaunchArgument("temporal_color_topic", default_value="/temporally_aligned/semantic"),
        DeclareLaunchArgument("temporal_mask_topic", default_value=""),
        DeclareLaunchArgument("temporal_input_depth_topic", default_value=""),
        DeclareLaunchArgument("temporal_input_label_topic", default_value=""),
        DeclareLaunchArgument("temporal_fixed_frame", default_value="odom"),
        DeclareLaunchArgument("temporal_camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("temporal_publish_color_semantic", default_value="true"),
        DeclareLaunchArgument("temporal_history_size", default_value="3"),
        DeclareLaunchArgument("temporal_max_history_age_s", default_value="0.5"),
        DeclareLaunchArgument("temporal_history_use_fused", default_value="true"),
        DeclareLaunchArgument("temporal_depth_gate_abs", default_value="0.2"),
        DeclareLaunchArgument("temporal_depth_gate_rel", default_value="0.05"),
        DeclareLaunchArgument("temporal_depth_fusion_mode", default_value="median"),
        DeclareLaunchArgument("temporal_depth_alpha", default_value="0.5"),
        DeclareLaunchArgument("temporal_label_min_votes", default_value="2"),
        DeclareLaunchArgument("temporal_dynamic_labels", default_value=""),
        DeclareLaunchArgument("temporal_ignore_labels", default_value=""),
        DeclareLaunchArgument("temporal_sync_queue", default_value="5"),
        DeclareLaunchArgument("temporal_sync_slop", default_value="0.02"),
        DeclareLaunchArgument("temporal_tf_timeout", default_value="0.05"),
        DeclareLaunchArgument("temporal_tf_buffer_size", default_value="10.0"),
        DeclareLaunchArgument("temporal_pose_quality_mode", default_value="none"),
        DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("hydra_rgb_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("hydra_camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("use_sparse_depth_factors", default_value="false"),
        DeclareLaunchArgument("use_semantic_masking", default_value="true"),
        DeclareLaunchArgument("semantic_mask_topic", default_value="/mono_hydra_vio/semantic_feature_mask"),
        DeclareLaunchArgument("semantic_mask_inflate_px", default_value="8"),
        DeclareLaunchArgument("sparse_depth_topic", default_value="/mono_hydra_vio/sparse_depth"),
        DeclareLaunchArgument("sparse_depth_stride", default_value="16"),
        DeclareLaunchArgument("sparse_depth_min_m", default_value="0.05"),
        DeclareLaunchArgument("sparse_depth_max_m", default_value="8.0"),
        DeclareLaunchArgument("superpoint_keypoint_mask_topic", default_value=""),
        DeclareLaunchArgument("output_root", default_value="output"),
    ]

    map_to_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mono_hydra_map_to_odom_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            LaunchConfiguration("map_frame"),
            "--child-frame-id",
            LaunchConfiguration("odom_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_world_tf")),
    )

    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mono_hydra_base_to_camera_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            LaunchConfiguration("robot_frame"),
            "--child-frame-id",
            LaunchConfiguration("camera_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_sensor_static_tf")),
    )

    rgb_to_mono_node = Node(
        package="mono_hydra_vio",
        executable="rgb_to_mono_node",
        name="mono_hydra_rvio2_rgb_to_mono",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rvio2_backend")),
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "input_topic": LaunchConfiguration("rgb_topic"),
                "output_topic": LaunchConfiguration("rvio2_mono_image_topic"),
            }
        ],
    )

    rvio2_node = Node(
        package="mono_hydra_vio",
        executable="rvio2_mono_node",
        name="rvio2_mono_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rvio2_backend")),
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "image_topic": LaunchConfiguration("rvio2_mono_image_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "config_path": LaunchConfiguration("rvio2_config_path"),
                "publish_tf": False,
            }
        ],
    )

    vio_feature_node = Node(
        package="mono_hydra_vio",
        executable="vio_feature_interface_node",
        name="mono_hydra_vio_feature_interface",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "enable_sparse_depth": ParameterValue(LaunchConfiguration("use_sparse_depth_factors"), value_type=bool),
                "dense_depth_topic": effective_depth_topic,
                "semantic_label_topic": effective_label_topic,
                "semantic_mask_topic": LaunchConfiguration("semantic_mask_topic"),
                "semantic_mask_inflate_px": ParameterValue(LaunchConfiguration("semantic_mask_inflate_px"), value_type=int),
                "sparse_depth_topic": LaunchConfiguration("sparse_depth_topic"),
                "sparse_depth_stride": ParameterValue(LaunchConfiguration("sparse_depth_stride"), value_type=int),
                "sparse_depth_min_m": ParameterValue(LaunchConfiguration("sparse_depth_min_m"), value_type=float),
                "sparse_depth_max_m": ParameterValue(LaunchConfiguration("sparse_depth_max_m"), value_type=float),
                "superpoint_keypoint_mask_topic": LaunchConfiguration("superpoint_keypoint_mask_topic"),
            }
        ],
    )

    odom_adapter_node = Node(
        package="mono_hydra_vio",
        executable="odom_adapter_node",
        name="mono_hydra_vio_odom_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "input_odom_topic": LaunchConfiguration("external_odom_topic"),
                "rvio2_trajectory_topic": LaunchConfiguration("rvio2_trajectory_topic"),
                "output_odom_topic": "/mono_hydra_vio/odometry",
                "output_path_topic": "/mono_hydra_vio/path",
                "use_rvio2_bridge": ParameterValue(LaunchConfiguration("use_rvio2_bridge"), value_type=bool),
                "publish_tf": ParameterValue(LaunchConfiguration("odom_adapter_publish_tf"), value_type=bool),
                "publish_bridge_external_odom": True,
                "publish_camera_tf": False,
                "map_frame_id": LaunchConfiguration("map_frame"),
                "odom_frame_id": LaunchConfiguration("odom_frame"),
                "base_link_frame_id": LaunchConfiguration("robot_frame"),
                "camera_frame_id": LaunchConfiguration("camera_frame"),
                "path_max_length": 5000,
                "force_frame_ids": ParameterValue(
                    LaunchConfiguration("odom_adapter_force_frame_ids"), value_type=bool
                ),
                "left_camera_params_path": LaunchConfiguration("rvio2_left_camera_params_path"),
                "input_pose_frame": LaunchConfiguration("rvio2_input_pose_frame"),
            }
        ],
    )

    kimera_vio_ros_node = Node(
        package="mono_hydra_vio",
        executable="mono_hydra_vio_ros_node",
        name="mono_hydra_vio_ros_node",
        namespace="mono_hydra_vio_ros",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_kimera_vio_ros_node")),
        arguments=[
            "--vocabulary_path",
            LaunchConfiguration("kimera_vocabulary_path"),
            "--flagfile",
            PathJoinSubstitution([LaunchConfiguration("kimera_flags_folder"), "Pipeline.flags"]),
            "--flagfile",
            PathJoinSubstitution([LaunchConfiguration("kimera_flags_folder"), "Mesher.flags"]),
            "--flagfile",
            PathJoinSubstitution([LaunchConfiguration("kimera_flags_folder"), "VioBackend.flags"]),
            "--flagfile",
            PathJoinSubstitution([LaunchConfiguration("kimera_flags_folder"), "RegularVioBackend.flags"]),
            "--flagfile",
            PathJoinSubstitution([LaunchConfiguration("kimera_flags_folder"), "Visualizer3D.flags"]),
            "--output_path",
            LaunchConfiguration("kimera_log_output_path"),
            "--logtostderr=1",
            "--colorlogtostderr=1",
        ],
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "params_folder_path": LaunchConfiguration("kimera_params_folder"),
                "sensor_params_folder_path": LaunchConfiguration("kimera_sensor_params_folder"),
                "use_lcd": ParameterValue(LaunchConfiguration("enable_lcd"), value_type=bool),
                "use_external_odom": ParameterValue(LaunchConfiguration("kimera_use_external_odom"), value_type=bool),
                "lcd_no_optimize": ParameterValue(LaunchConfiguration("kimera_lcd_no_optimize"), value_type=bool),
                "lcd_no_detection": ParameterValue(LaunchConfiguration("kimera_lcd_no_detection"), value_type=bool),
                "lcd_disable_stereo_match_depth_check": ParameterValue(
                    LaunchConfiguration("kimera_lcd_disable_stereo_match_depth_check"), value_type=bool
                ),
                "no_incremental_pose": ParameterValue(
                    LaunchConfiguration("kimera_no_incremental_pose"), value_type=bool
                ),
                "do_coarse_imu_camera_temporal_sync": ParameterValue(
                    LaunchConfiguration("kimera_do_coarse_imu_camera_temporal_sync"), value_type=bool
                ),
                "do_fine_imu_camera_temporal_sync": ParameterValue(
                    LaunchConfiguration("kimera_do_fine_imu_camera_temporal_sync"), value_type=bool
                ),
                "visualize": ParameterValue(LaunchConfiguration("visualize"), value_type=bool),
                "use_rviz": ParameterValue(LaunchConfiguration("use_rviz"), value_type=bool),
                "viz_type": 1,
                "log_output": ParameterValue(LaunchConfiguration("kimera_log_output"), value_type=bool),
                "publish_tf": ParameterValue(LaunchConfiguration("kimera_publish_tf"), value_type=bool),
                "publish_lcd_tf": ParameterValue(LaunchConfiguration("kimera_publish_lcd_tf"), value_type=bool),
                "publish_camera_tf": ParameterValue(LaunchConfiguration("kimera_publish_camera_tf"), value_type=bool),
                "force_same_image_timestamp": ParameterValue(
                    LaunchConfiguration("kimera_force_same_image_timestamp"), value_type=bool
                ),
                "robot_id": 0,
                "odom_frame_id": LaunchConfiguration("odom_frame"),
                "base_link_frame_id": LaunchConfiguration("robot_frame"),
                "map_frame_id": LaunchConfiguration("map_frame"),
                "left_cam_frame_id": LaunchConfiguration("camera_frame"),
                "right_cam_frame_id": "right_cam",
                "left_cam_topic": LaunchConfiguration("kimera_left_cam_topic"),
                "right_cam_topic": LaunchConfiguration("kimera_right_cam_topic"),
                "depth_cam_topic": effective_depth_topic,
                "imu_topic": LaunchConfiguration("imu_topic"),
                "external_odom_topic": "/mono_hydra_vio/odometry",
                "semantic_mask_topic": LaunchConfiguration("semantic_mask_topic"),
                "use_semantic_masking": ParameterValue(LaunchConfiguration("use_semantic_masking"), value_type=bool),
                "semantic_mask_is_label_image": True,
                "semantic_mask_inflate_px": ParameterValue(LaunchConfiguration("semantic_mask_inflate_px"), value_type=int),
                "rgbd_sync_queue_size": ParameterValue(
                    LaunchConfiguration("kimera_rgbd_sync_queue_size"), value_type=int
                ),
                "stereo_sync_queue_size": ParameterValue(
                    LaunchConfiguration("kimera_stereo_sync_queue_size"), value_type=int
                ),
            }
        ],
    )

    kimera_pose_graph_bridge_node = Node(
        package="mono_hydra_vio",
        executable="kimera_pose_graph_bridge_node",
        name="mono_hydra_kimera_pose_graph_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_kimera_pose_graph_bridge")),
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "pose_graph_topic": LaunchConfiguration("kimera_pose_graph_topic"),
                "pose_graph_incremental_topic": LaunchConfiguration("kimera_pose_graph_incremental_topic"),
                "external_loop_closures_topic": LaunchConfiguration("external_loop_closures_topic"),
                "publish_external_loop_closures": ParameterValue(
                    LaunchConfiguration("use_kimera_external_lc_bridge"), value_type=bool
                ),
            }
        ],
    )

    mesh_marker_node = Node(
        package="mono_hydra_utils",
        executable="mesh_marker_node",
        name="mono_hydra_dsg_mesh_marker",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_mesh_marker")),
        parameters=[
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "input_mesh_topic": LaunchConfiguration("mesh_marker_input_topic"),
                "output_marker_topic": "/hydra_dsg_visualizer/dsg_mesh_marker",
                "fallback_frame_id": LaunchConfiguration("map_frame"),
                "mesh_alpha": ParameterValue(LaunchConfiguration("mesh_alpha"), value_type=float),
                "default_color": [0.72, 0.74, 0.78, 0.92],
                "max_triangles": 0,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mono_hydra_rviz",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        remappings=[
            ("/camera/color/image_raw", LaunchConfiguration("rgb_topic")),
            ("/camera/color/camera_info", LaunchConfiguration("camera_info_topic")),
            ("/camera/depth_cam/image_raw", effective_depth_topic),
            ("/camera/seg_cam/labels_argmax", effective_label_topic),
            ("/camera/seg_cam/image_raw", effective_semantic_color_topic),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        declared
        + [
            map_to_world_tf,
            base_to_camera_tf,
            rgb_to_mono_node,
            rvio2_node,
            vio_feature_node,
            odom_adapter_node,
            kimera_vio_ros_node,
            kimera_pose_graph_bridge_node,
            mesh_marker_node,
            OpaqueFunction(function=_perception_nodes),
            OpaqueFunction(function=_temporal_alignment_node),
            OpaqueFunction(function=_official_hydra_group),
            rviz_node,
        ]
    )
