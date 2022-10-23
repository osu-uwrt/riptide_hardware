from launch.launch_description import LaunchDescription
from launch_ros.actions import Node

copro_agent = Node(
    package='micro_ros_agent',
    executable='micro_ros_agent',
    output='screen',
    arguments=['udp4', '-p', '8888', '-v4'] ## change to -v4 for actual logs
)
depth_agent = Node(
    package='micro_ros_agent',
    executable='micro_ros_agent',
    output='screen',
    arguments=['serial', '--dev', '/dev/ttyACM1', '-v4'] ## change to -v4 for actual logs
)

def generate_launch_description():
    return LaunchDescription([
        copro_agent,
        depth_agent
    ])