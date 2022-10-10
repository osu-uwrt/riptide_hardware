from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration as LC
from datetime import datetime
from pathlib import Path
import os

"""
To change the settings of this camera, go into IE, 
hit download, enable activeX, set english, and then log in 

The camera has two streams. Settings can be changed in config
Stream 0 is high resolution: Can be 1080p at 25 FPS or 3 MegaPixel at 15 FPS.
Stream 1 is low resolution: Is at 25 FPS.
"""

# Should be whole number floating point
fps = 25.0

camera_node = Node(
    name='image_publisher_node',
    package='image_publisher',
    executable='image_publisher_node',
    output='screen',
    parameters=[
        {"camera_info_url": os.path.join(get_package_share_directory('riptide_hardware2'), 'cfg', 'underwater_cam.yaml')},
        {"flip_horizontal": False},
        {"flip_vertical": False},
        {"retry": True},
        {"publish_rate": fps}
    ],
    arguments=[
        "rtsp://192.168.1.10:554/user=admin_password=6QNMIQGe_channel=0_stream=0.sdp?real_stream"
    ]
)

video_recorder_node = Node(
    name='video_recorder',
    package='image_view',
    executable='video_recorder',
    output='screen',
    parameters=[
        {"fps": int(fps)},
        {"filename": PathJoinSubstitution([LC('record_path'), 
                        "underwater_cam_{0}T.avi".format(datetime.now().strftime('%Y-%m-%d-%I-%M-%S'))
                        ])}
    ],
    remappings=[
        ("/image", "/underwater_cam/image_raw")
    ]
)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('record_path', default_value=str(Path.home()), description="The path to record to"),
        GroupAction([
            PushRosNamespace("underwater_cam"),
            camera_node,

            # delay the recorder start
            TimerAction(period=3.0, actions=[video_recorder_node,])
        ])
    ])