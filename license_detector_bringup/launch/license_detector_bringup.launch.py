import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('license_detector_bringup'),
        'config',
        'image_publisher.yaml'
    )

    image_publisher_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        on_exit=Shutdown(),
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    license_extractor_node = Node(
        package='license_extractor',
        executable='license_extractor_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    char_detector_node = Node(
        package='char_detector',
        executable='char_detector_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        image_publisher_node,
        license_extractor_node,
        char_detector_node
    ])
