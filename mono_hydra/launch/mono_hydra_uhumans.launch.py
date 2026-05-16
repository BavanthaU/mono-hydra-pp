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
                    "dataset": "uhumans2",
                    "sequence_name": "uhumans2",
                    "labelspace_name": "nyud20_config",
                    "hydra_labelspace_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_perception"), "config", "label_spaces", "nyud20_config_label_space.yaml"]
                    ),
                    "lcd_config_path": PathJoinSubstitution(
                        [FindPackageShare("hydra"), "config", "lcd", "uhumans2.yaml"]
                    ),
                    "perception_dataset": "nyud",
                    "perception_depth_scale": "1.0",
                    "perception_image_height": "480",
                    "perception_image_width": "640",
                    "perception_input_queue_size": "3000",
                    "perception_publish_synced_inputs": "true",
                    "perception_synced_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "perception_synced_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "perception_publish_color_semantic": "true",
                    "odom_frame": "world",
                    "publish_world_tf": "false",
                    "camera_frame": "left_cam_kimera",
                    "robot_frame": "base_link_kimera",
                    "imu_topic": "/tesse/imu/clean/imu",
                    "rgb_topic": "/tesse/left_cam/rgb/image_raw",
                    "camera_info_topic": "/tesse/left_cam/camera_info",
                    "hydra_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                    "hydra_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "perception_depth_topic": "/tesse/m2h/depth/image_raw",
                    "perception_label_topic": "/tesse/m2h/labels_argmax",
                    "perception_semantic_color_topic": "/m2h_hmx_large/semantic/image_raw",
                    "depth_topic": "/tesse/m2h/depth/image_raw",
                    "label_topic": "/tesse/m2h/labels_argmax",
                    "semantic_color_topic": "/m2h_hmx_large/semantic/image_raw",
                    "temporal_fixed_frame": "world",
                    "temporal_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    "use_rvio2_bridge": "true",
                    "use_rvio2_backend": "true",
                    "rvio2_config_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "config", "uhumans2_office_720x480.yaml"]
                    ),
                    "rvio2_left_camera_params_path": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "uHumans2_RGBD", "LeftCameraParams.yaml"]
                    ),
                    "kimera_params_folder": PathJoinSubstitution(
                        [FindPackageShare("mono_hydra_vio"), "params", "uHumans2_RGBD_RVIO2_depth_factors_off"]
                    ),
                    "kimera_sensor_params_folder": "",
                    "kimera_flags_folder": PathJoinSubstitution(
                        [
                            FindPackageShare("mono_hydra_vio"),
                            "params",
                            "uHumans2_RGBD_RVIO2_depth_factors_off",
                            "flags",
                        ]
                    ),
                    "kimera_use_external_odom": "true",
                    "kimera_publish_tf": "true",
                    "kimera_publish_lcd_tf": "true",
                    "kimera_left_cam_topic": "/mono_hydra_perception/synced/image_raw",
                    "kimera_rgbd_sync_queue_size": "3000",
                    "kimera_stereo_sync_queue_size": "3000",
                    "rvio2_input_pose_frame": "body",
                    "enable_lcd": "true",
                    "odom_adapter_publish_tf": "false",
                }.items(),
            )
        ]
    )
