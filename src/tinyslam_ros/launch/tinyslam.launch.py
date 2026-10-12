import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('tinyslam_ros'), 'config', 'tinyslam.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    tinyslam = Node(
        package='tinyslam_ros',
        executable='tinyslam_node',
        name='tinyslam',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='tinySLAM parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo /clock'),
        tinyslam,
    ])
