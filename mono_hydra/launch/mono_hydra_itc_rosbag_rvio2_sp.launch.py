"""ITC rosbag pipeline with RVIO2-SP (SuperPoint guided tracking) as external odometry.

Equivalent to ``mono_hydra_itc_rosbag.launch.py rvio2_variant:=superpoint``. Every
other argument of that launch file (and the rvio2_sp_* switches) passes through.
"""

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
                        [FindPackageShare("mono_hydra"), "launch", "mono_hydra_itc_rosbag.launch.py"]
                    )
                ),
                launch_arguments={"rvio2_variant": "superpoint"}.items(),
            )
        ]
    )
