from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC
import os

electrical_monitor_node = Node(
    name='electrical_monitor',
    package='riptide_hardware2',
    executable='electrical_monitor',
    output='screen',
    parameters=[
        {"diag_thresholds_file": os.path.join(get_package_share_directory('riptide_hardware2'), 'cfg', 'diagnostic_thresholds.yaml')},
        {"robot": LC('robot')},
    ]
)

voltage_monitor_node = Node(
    name='voltage_monitor',
    package='riptide_hardware2',
    executable='voltage_monitor',
    output='screen',
    parameters=[
        {"diag_thresholds_file": os.path.join(get_package_share_directory('riptide_hardware2'), 'cfg', 'diagnostic_thresholds.yaml')},
        {"robot": LC('robot')},
    ]
)

sensor_monitor_node = Node(
    name='sensor_monitor',
    package='riptide_hardware2',
    executable='sensor_monitor',
    output='screen'
)

computer_monitor_node = Node(
    name='computer_monitor',
    package='riptide_hardware2',
    executable='computer_monitor',
    output='screen'
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="puddles", description="Name of the vehicle"),

        electrical_monitor_node,
        voltage_monitor_node,
        sensor_monitor_node,
        computer_monitor_node
    ])