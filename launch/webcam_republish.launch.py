import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(package='image_transport', node_executable='republish', arguments=[ 
                'raw',
                '--ros-args',
                '-r in:=/image_raw',
                'raw', 
                '-r out:=/image'
            ],
            output='screen', node_name='republish')
    ])