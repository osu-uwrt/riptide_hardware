import launch
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC

import os

imu_launch_file = os.path.join(
    get_package_share_directory('microstrain_inertial_driver'),
    "launch", "microstrain_launch.py"
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="puddles", description="Name of the vehicle"),
        DeclareLaunchArgument('serial_port', default_value="/dev/imu_riptide"),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(imu_launch_file),
            launch_arguments=[
                ('params_file', os.path.join(get_package_share_directory('riptide_hardware2'), "cfg", "imu_config.yaml")),
                ('namespace', ['/', LC('robot')]),
                ('configure', 'true'),
                ('activate', 'true'),
                ('port', LC('serial_port')),
                ('imu_frame_id', [LC('robot'), '/imu_link']),
            ]
        )
    ])