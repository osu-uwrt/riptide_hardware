from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

copro_agent = Node(
    package='micro_ros_agent',
    executable='micro_ros_agent',
    output='screen',
    arguments=['serial', '--dev', '/dev/uwrt_copro']
)

def generate_launch_description():
    return LaunchDescription([
        copro_agent
    ])