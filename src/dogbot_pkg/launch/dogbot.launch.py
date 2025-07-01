from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    agilex_ws_path = os.path.expanduser('~/agilex_ws')

    limo_bringup = os.path.join(agilex_ws_path, 'install', 'limo_bringup', 'share', 'limo_bringup', 'launch', 'humble', 'limo_start.launch.py')
    dabai_camera = os.path.join(agilex_ws_path, 'install', 'astra_camera', 'share', 'astra_camera', 'launch', 'dabai.launch.py')

    aruco_node_0 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_marker_0',
        namespace='marker_0',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.048,
            'marker_id': 0,
            'camera_frame': 'camera_color_optical_frame',
            'publish_tf': True
        }],
        remappings=[
            ('image', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info')
        ]
    )
    aruco_node_1 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_marker_1',
        namespace='marker_1',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.048,
            'marker_id': 1,
            'camera_frame': 'camera_color_optical_frame',
            'publish_tf': True
        }],
        remappings=[
            ('image', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info')
        ]
    )

    aruco_node_2 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_marker_2',
        namespace='marker_2',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.048,
            'marker_id': 2,
            'camera_frame': 'camera_color_optical_frame',
            'publish_tf': True
        }],
        remappings=[
            ('image', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info')
        ]
    )

    aruco_node_3 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_marker_3',
        namespace='marker_3',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.048,
            'marker_id': 3,
            'camera_frame': 'camera_color_optical_frame',
            'publish_tf': True
        }],
        remappings=[
            ('image', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info')
        ]
    )

    aruco_node_4 = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_marker_4',
        namespace='marker_4',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.048,
            'marker_id': 4,
            'camera_frame': 'camera_color_optical_frame',
            'publish_tf': True
        }],
        remappings=[
            ('image', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info')
        ]
    )

    return LaunchDescription([
        LogInfo(msg='🚀 Starting LIMO bringup...'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(limo_bringup)),
        LogInfo(msg='✅ LIMO bringup started.'),

        LogInfo(msg='📷 Launching Astra DaBai camera...'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(dabai_camera)),
        LogInfo(msg='✅ Astra camera launched.'),

        LogInfo(msg='🎯 Starting ArUco marker tracking...'),
        aruco_node_0,
        aruco_node_1,
        aruco_node_2,
        aruco_node_3,
        aruco_node_4,
        LogInfo(msg='✅ ArUco tracking nodes running.'),

        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='🤖 Launching QR behavior node...'),
                Node(
                    package='dogbot_pkg',
                    executable='qr_behavior_node',
                    name='qr_behavior_node',
                    output='screen',
                    arguments=['--ros-args', '--log-level', 'info']
                )
            ]
        )
    ])
