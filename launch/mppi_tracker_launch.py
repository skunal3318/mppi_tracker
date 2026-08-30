import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mppi_tracker')
    default_params = os.path.join(pkg_share, 'config', 'mppi_params.yaml')

    # CSV file which will contain recorded path
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value='/tmp/test_path.csv',
    )

    mppi_node = Node(
        package='mppi_tracker',
        executable='mppi_tracker',
        name='mppi_tracker',
        output='screen',
        parameters=[
            default_params,
            {'path_file': LaunchConfiguration('path_file')},
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        path_file_arg,
        mppi_node,
    ])
