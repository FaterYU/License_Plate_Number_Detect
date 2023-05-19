import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('license_detetor'),
        'config',
        'license_detetor.yaml'
    )

    license_detetor_node = Node(
        package='license_detetor',
        executable='license_detetor_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config]
    )

    return LaunchDescription([license_detetor_node])
