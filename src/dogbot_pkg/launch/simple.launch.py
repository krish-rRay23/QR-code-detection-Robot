# dogbot_pkg/launch/dogbot_launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to limo_base.launch.py
    agilex_ws_path = os.path.expanduser('~/agilex_ws')  # Change if it's in another path
    limo_launch_file = os.path.join(agilex_ws_path, 'install', 'limo_bringup', 'share', 'limo_bringup', 'launch', 'humble','limo_start.launch.py')

    # Include Limo base launch
    limo_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(limo_launch_file)
    )

    # Wait a few seconds for limo to initialize before launching your nodes
    delayed_nodes = TimerAction(
        period=3.0,  # seconds
        actions=[
            Node(
                package='dogbot_pkg',
                executable='velocity_publisher',
                name='velocity_publisher',
                output='screen'
            ),
            Node(
                package='dogbot_pkg',
                executable='velocity_subscriber',
                name='velocity_subscriber',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        limo_base_launch,
        delayed_nodes
    ])
