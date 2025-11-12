import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='treadmill_controller',
            executable='treadmill_ctrl',
            name='Treadmill',
        ),
        launch_ros.actions.Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
        ),
])
