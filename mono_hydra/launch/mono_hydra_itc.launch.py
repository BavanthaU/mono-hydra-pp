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
                    "dataset": "itc",
                    "sequence_name": "itc",
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
                    "perception_dataset": "itc",
                    "perception_skip_frequency": "5",
                    "perception_input_queue_size": "3000",
                    "perception_publish_synced_inputs": "true",
                    "perception_synced_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "perception_synced_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "perception_depth_scale": "1.0",
                    "perception_image_height": "480",
                    "perception_image_width": "640",
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
                    "use_rvio2_bridge": "true",
                    "use_rvio2_backend": "true",
                    "rvio2_mono_image_topic": "/rvio2_bridge/cam0/image_raw",
                    "rvio2_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "config", "itc_realsense_848x480.yaml"]
                    ),
                    "kimera_params_folder": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "RealSense_RGBD_RVIO2"]
                    ),
                    "kimera_sensor_params_folder": "",
                    "kimera_flags_folder": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "RealSense_RGBD_RVIO2", "flags"]
                    ),
                    "kimera_use_external_odom": "true",
                    "kimera_lcd_no_optimize": "true",
                    "kimera_publish_tf": "true",
                    "kimera_publish_lcd_tf": "true",
                    "kimera_publish_camera_tf": "true",
                    "kimera_left_cam_topic": "/mono_hydra_perception/synced/image_raw",
                    "kimera_rgbd_sync_queue_size": "3000",
                    "kimera_stereo_sync_queue_size": "3000",
                    "enable_lcd": "true",
                    "odom_adapter_publish_tf": "false",
                    "publish_sensor_static_tf": "false",
                }.items(),
            )
        ]
    )
