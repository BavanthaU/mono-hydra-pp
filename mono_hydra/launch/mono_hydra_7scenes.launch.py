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
                    "dataset": "7scenes",
                    "sequence_name": "7scenes",
                    "labelspace_name": "scannet20_config",
                    "hydra_labelspace_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_perception"), "config", "label_spaces", "scannet20_config_label_space.yaml"]
                    ),
                    "perception_dataset": "scannet",
                    "perception_input_queue_size": "3000",
                    "perception_output_queue_size": "50",
                    "perception_warn_output_lag_s": "5.0",
                    "perception_max_output_lag_s": "0.0",
                    "perception_publish_synced_inputs": "true",
                    "perception_synced_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "perception_synced_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "odom_frame": "scannet_world",
                    "camera_frame": "scannet_camera",
                    "rgb_topic": "/camera/color/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "hydra_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "hydra_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "perception_depth_topic": "/camera/depth_cam/image_raw",
                    "perception_label_topic": "/camera/seg_cam/labels_argmax",
                    "perception_semantic_color_topic": "/camera/seg_cam/image_raw",
                    "depth_topic": "/camera/depth_cam/image_raw",
                    "label_topic": "/camera/seg_cam/labels_argmax",
                    "temporal_fixed_frame": "scannet_world",
                    "temporal_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "use_rvio2_bridge": "false",
                    "rvio2_left_camera_params_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "7Scenes", "LeftCameraParams.yaml"]
                    ),
                    "rvio2_input_pose_frame": "sensor",
                    "odom_adapter_force_frame_ids": "true",
                    "kimera_params_folder": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "7Scenes"]
                    ),
                    "kimera_sensor_params_folder": "",
                    "kimera_flags_folder": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "7Scenes", "flags"]
                    ),
                    "kimera_use_external_odom": "true",
                    "kimera_left_cam_topic": "/mono_hydra_perception/synced/image_raw",
                    "kimera_rgbd_sync_queue_size": "3000",
                    "kimera_stereo_sync_queue_size": "3000",
                    "enable_lcd": "true",
                }.items(),
            )
        ]
    )
