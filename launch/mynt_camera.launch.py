from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration as LC

import os

image_proc_launch_file = os.path.join(
    get_package_share_directory('riptide_hardware2'),
    "launch", "mynt_image_proc.launch.py"
)

mynt_camera_node = Node(
    name='mynteye_wrapper_d_node',
    package='mynteye_wrapper_d',
    executable='mynteye_wrapper_d_node',
    output='screen',
    arguments=['--ros-args', '--log-level', 'WARN'],
    parameters=[
        # Camera params
        {"dev_index": LC('dev_index')},
        {"framerate": LC('framerate')},
        {"dev_mode": LC('dev_mode')},
        {"color_mode": LC('color_mode')},
        {"depth_mode": LC('depth_mode')},
        {"stream_mode": LC('stream_mode')},
        {"color_stream_format": LC('color_stream_format')},
        {"depth_stream_format": LC('depth_stream_format')},
        {"state_ae": LC('state_ae')},
        {"state_awb": LC('state_awb')},
        {"ir_intensity": LC('ir_intensity')},
        {"ir_depth_only": LC('ir_depth_only')},
        {"imu_timestamp_align": LC('imu_timestamp_align')},

        {"points_factor": LC('points_factor')},
        {"points_frequency": LC('points_frequency')},

        {"gravity": LC('gravity')},

        {"depth_type": LC('depth_type')},

        # Frame ids
        {"base_frame": LC('base_frame')},
        {"left_mono_frame": LC('left_frame')},
        {"left_color_frame": LC('left_frame')},
        {"right_mono_frame": LC('right_frame')},
        {"right_color_frame": LC('right_frame')},
        {"depth_frame": LC('depth_frame')},
        {"points_frame": LC('points_frame')},
        {"imu_frame": LC('imu_frame')},
        {"temp_frame": LC('temp_frame')},
        {"imu_frame_processed": LC('imu_frame_processed')},

        # Topic names

        {"left_mono_topic": LC('left_mono_topic')},
        {"left_color_topic": LC('left_color_topic')},
        {"right_mono_topic": LC('right_mono_topic')},
        {"right_color_topic": LC('right_color_topic')},
        {"depth_topic": LC('depth_topic')},
        {"points_topic": LC('points_topic')},
        {"imu_topic": LC('imu_topic')},
        {"temp_topic": LC('temp_topic')},
        {"imu_processed_topic": LC('imu_processed_topic')},

        {"mesh_file": LC('mesh_file')},
    ]
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tempest', description="Name of the vehicle"),
        DeclareLaunchArgument('dev_index', default_value='0'),
        DeclareLaunchArgument('name', default_value='stereo'),

        # Device mode
        DeclareLaunchArgument('device_color', default_value='0'),
        DeclareLaunchArgument('device_depth', default_value='1'),
        DeclareLaunchArgument('device_all', default_value='2'),
        # Color mode
        DeclareLaunchArgument('color_raw', default_value='0'),
        DeclareLaunchArgument('color_rectified', default_value='1'),
        # Depth mode
        DeclareLaunchArgument('depth_raw', default_value='0'),
        DeclareLaunchArgument('depth_gray', default_value='1'),
        DeclareLaunchArgument('depth_colorful', default_value='2'),
        # Stream mode
        DeclareLaunchArgument('stream_640x480', default_value='0'),
        DeclareLaunchArgument('stream_1280x480', default_value='1'),
        DeclareLaunchArgument('stream_1280x720', default_value='2'),
        DeclareLaunchArgument('stream_2560x720', default_value='3'),
        # Stream format
        DeclareLaunchArgument('stream_mjpg', default_value='0'),
        DeclareLaunchArgument('stream_yuyv', default_value='1'),
        # Depth type
        DeclareLaunchArgument('type_mono16', default_value='0'),
        DeclareLaunchArgument('type_16uc1', default_value='1'),

        # Device Configuration
        DeclareLaunchArgument('imu_timestamp_align', default_value='true'),
        DeclareLaunchArgument('framerate', default_value='30'),

        # Device mode
        #   device_color: left_color ✓ right_color ? depth x
        #   device_depth: left_color x right_color x depth ✓
        #   device_all:   left_color ✓ right_color ? depth ✓
        # Note: ✓: available, x: unavailable, ?: depends on #stream_mode
        DeclareLaunchArgument('dev_mode', default_value=LC('device_all')),
        DeclareLaunchArgument('color_mode', default_value=LC('color_raw')),
        # Note: must set DEPTH_RAW to get raw depth values for points
        DeclareLaunchArgument('depth_mode', default_value=LC('depth_raw')),
        DeclareLaunchArgument('stream_mode', default_value=LC('stream_2560x720')),
        DeclareLaunchArgument('color_stream_format', default_value=LC('stream_yuyv')),
        DeclareLaunchArgument('depth_stream_format', default_value=LC('stream_yuyv')),
        DeclareLaunchArgument('depth_type', default_value=LC('type_mono16')),

        # Auto-exposure
        DeclareLaunchArgument('state_ae', default_value='true'),
        # Auto-white balance
        DeclareLaunchArgument('state_awb', default_value='true'),
        # IR intensity
        DeclareLaunchArgument('ir_intensity', default_value='0'),
        # IR Depth Only
        DeclareLaunchArgument('ir_depth_only', default_value='false'),

        # Generating points frequency, make slower than framerate
        DeclareLaunchArgument('points_frequency', default_value='10.0'),
        # Points display z distance scale factor
        DeclareLaunchArgument('points_factor', default_value='1000.0'),

        # Setup your local gravity here
        DeclareLaunchArgument('gravity', default_value='9.8'),

        # Node params
        DeclareLaunchArgument('base_frame', default_value=[LC('robot'), '/', LC('name'), '/base_link']),
        DeclareLaunchArgument('left_frame', default_value=[LC('robot'), '/', LC('name'), '/left_optical']),
        DeclareLaunchArgument('right_frame', default_value=[LC('robot'), '/', LC('name'), '/right_optical']),
        DeclareLaunchArgument('depth_frame', default_value=[LC('robot'), '/', LC('name'), '/left_optical']),
        DeclareLaunchArgument('points_frame', default_value=[LC('robot'), '/', LC('name'), '/left_optical']),
        DeclareLaunchArgument('imu_frame', default_value=[LC('robot'), '/', LC('name'), '/imu_link']),
        DeclareLaunchArgument('temp_frame', default_value=[LC('robot'), '/', LC('name'), '/imu_link']),
        DeclareLaunchArgument('imu_frame_processed', default_value=[LC('robot'), '/', LC('name'), '/imu_link']),

        # left topics
        DeclareLaunchArgument('left_mono_topic', default_value=[LC('name'), '/left/image_mono']),
        DeclareLaunchArgument('left_color_topic', default_value=[LC('name'), '/left/image_raw']),
        # right topics
        DeclareLaunchArgument('right_mono_topic', default_value=[LC('name'), '/right/image_mono']),
        DeclareLaunchArgument('right_color_topic', default_value=[LC('name'), '/right/image_raw']),
        # depth topic
        DeclareLaunchArgument('depth_topic', default_value=[LC('name'), '/depth/image_raw']),
        # points topic
        DeclareLaunchArgument('points_topic', default_value=[LC('name'), '/points2']),
        # imu topic origin
        DeclareLaunchArgument('imu_topic', default_value=[LC('name'), '/imu/data_unprocessed']),
        # temp topic
        DeclareLaunchArgument('temp_topic', default_value=[LC('name'), '/temp/data_raw']),
        # imu topic processed
        DeclareLaunchArgument('imu_processed_topic', default_value=[LC('name'), '/imu/data_raw']),

        DeclareLaunchArgument('mesh_file', default_value='D-0315.mtl'),

        mynt_camera_node,

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(image_proc_launch_file),
            launch_arguments=[
                ('image_namespace', [LC('name'), '/left'])
            ]
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(image_proc_launch_file),
            launch_arguments=[
                ('image_namespace', [LC('name'), '/right'])
            ]
        ),

        # IncludeLaunchDescription(
        #     AnyLaunchDescriptionSource(image_proc_launch_file),
        #     launch_arguments=[
        #         ('image_namespace', [LC('name'), '/depth'])
        #     ]
        # ),
    ])