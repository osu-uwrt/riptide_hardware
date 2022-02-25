import launch
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC

import os

dvl_launch_file = os.path.join(
    get_package_share_directory('nortek_dvl'),
    "launch", "dvl.launch.py"
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="tempest", description="Name of the vehicle"),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(dvl_launch_file),
            launch_arguments=[
                ('frame_id', [LC('robot'), '/dvl_link']),
                ('sonar_frame_id', [LC('robot'), '/dvl_sonar%d_link']),
            ]
        )
    ])