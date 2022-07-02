import launch
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC

import os

copro_agent_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "copro_agent.launch.py",
)

diagnostics_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "diagnostics.launch.py"
)

dvl_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "dvl.launch.py"
)

imu_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "imu.launch.py"
)

mynt_camera_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "mynt_camera.launch.py"
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="tempest", description="Name of the vehicle"),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(copro_agent_launch_file),
            launch_arguments=[
                ('robot', LC('robot')),
            ]
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(diagnostics_launch_file),
            launch_arguments=[
                ('robot', LC('robot')),
            ]
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(dvl_launch_file),
            launch_arguments=[
                ('robot', LC('robot')),
            ]
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(imu_launch_file),
            launch_arguments=[
                ('robot', LC('robot')),
            ]
        )
        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(mynt_camera_launch_file),
        #     launch_arguments=[
        #         ('robot', LC('robot')),
        #     ]
        # )
    ])