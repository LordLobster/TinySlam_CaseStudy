import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('tinyslam_ros'), 'config', 'tinyslam_turtlebot4.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    tinyslam = Node(
        package='tinyslam_ros',
        executable='tinyslam_node',
        name='tinyslam',
        namespace=namespace,
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        # A namespaced TurtleBot 4 publishes TF on <namespace>/tf; with no namespace
        # these resolve back to /tf and /tf_static.
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='tinySLAM parameter file'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='true when using turtlebot4_simulator'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='Robot namespace, e.g. robot1 (empty for none)'),
        tinyslam,
    ])
