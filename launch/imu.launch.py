from matplotlib.pyplot import get
import launch
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC

import os

imu_launch_file = os.path.join(
    get_package_share_directory('imu_3dm_gx4'),
    "launch", "imu.launch.py"
)

merge_launch_file = os.path.join(
    get_package_share_directory('imu_3dm_gx4'),
    "launch", "merge.launch.py"
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="puddles", description="Name of the vehicle"),
        DeclareLaunchArgument('device', default_value="/dev/imu_riptide"),
        DeclareLaunchArgument('enable_mag_update', default_value=True),
        DeclareLaunchArgument('use_enu_frame', default_value=True),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(imu_launch_file),
            launch_arguments=[
                ('device', LC('device')),
                ('frame_id', [LC('robot'), '/imu_link']),
                ('enable_mag_update', LC('enable_mag_update')),
                ('use_enu_frame', LC('use_enu_frame')),
            ]
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(merge_launch_file)
        )
    ])