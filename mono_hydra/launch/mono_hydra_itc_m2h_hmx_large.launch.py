from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset", default_value="itc"),
            DeclareLaunchArgument("device", default_value="auto"),
            DeclareLaunchArgument("half", default_value="true"),
            DeclareLaunchArgument("skip_frequency", default_value="3"),
            DeclareLaunchArgument("depth_scale", default_value="1.0"),
            DeclareLaunchArgument("image_width", default_value="640"),
            DeclareLaunchArgument("image_height", default_value="480"),
            Node(
                package="mono_hydra_perception",
                executable="m2h_hmx_large_node",
                name="mono_hydra_perception",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "dataset": LaunchConfiguration("dataset"),
                        "device": LaunchConfiguration("device"),
                        "half": ParameterValue(LaunchConfiguration("half"), value_type=bool),
                        "skip_frequency": ParameterValue(LaunchConfiguration("skip_frequency"), value_type=int),
                        "depth_scale": ParameterValue(LaunchConfiguration("depth_scale"), value_type=float),
                        "image_width": ParameterValue(LaunchConfiguration("image_width"), value_type=int),
                        "image_height": ParameterValue(LaunchConfiguration("image_height"), value_type=int),
                        "image_topic": "/camera/color/image_raw",
                        "camera_info_topic": "/camera/color/camera_info",
                        "image_depth_topic": "/camera/depth_cam/image_raw",
                        "image_semantic_topic": "/camera/seg_cam/image_raw",
                        "label_ids_topic": "/camera/seg_cam/labels_argmax",
                        "input_queue_size": 256,
                        "output_queue_size": 10,
                        "warn_output_lag_s": 5.0,
                        "max_output_lag_s": 0.0,
                        "publish_synced_inputs": True,
                        "synced_rgb_topic": "/mono_hydra_perception/synced/image_raw",
                        "synced_camera_info_topic": "/mono_hydra_perception/synced/camera_info",
                    }
                ],
            ),
        ]
    )
