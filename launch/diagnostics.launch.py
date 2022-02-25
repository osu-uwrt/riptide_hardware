from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC
import os

analyzer_params_filepath = os.path.join(get_package_share_directory('riptide_hardware2'), 'cfg', 'diagnostic_analyzers.yaml')
thresholds_file = os.path.join(get_package_share_directory('riptide_hardware2'), 'cfg', 'diagnostic_thresholds.yaml')

aggregator = Node(
    package='diagnostic_aggregator',
    executable='aggregator_node',
    output='screen',
    parameters=[analyzer_params_filepath],
    arguments=['--ros-args', '--log-level', 'WARN']
)

electrical_monitor_node = Node(
    name='electrical_monitor',
    package='riptide_hardware2',
    executable='electrical_monitor',
    output='screen',
    parameters=[
        {"diag_thresholds_file": thresholds_file},
        {"robot": LC('robot')},
    ]
)

voltage_monitor_node = Node(
    name='voltage_monitor',
    package='riptide_hardware2',
    executable='voltage_monitor',
    output='screen',
    parameters=[
        {"diag_thresholds_file": thresholds_file},
        {"robot": LC('robot')},
    ]
)

sensor_monitor_node = Node(
    name='sensor_monitor',
    package='riptide_hardware2',
    executable='sensor_monitor',
    output='screen',
    parameters=[
        {"diag_thresholds_file": thresholds_file},
    ]
)

computer_monitor_node = Node(
    name='computer_monitor',
    package='riptide_hardware2',
    executable='computer_monitor',
    output='screen',
    parameters=[
        {"diag_thresholds_file": thresholds_file},
    ]
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value="tempest", description="Name of the vehicle"),

        electrical_monitor_node,
        voltage_monitor_node,
        sensor_monitor_node,
        computer_monitor_node,
        aggregator
    ])