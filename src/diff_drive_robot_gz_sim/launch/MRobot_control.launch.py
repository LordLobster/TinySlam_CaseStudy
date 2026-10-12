import xacro
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='empty',  description='Ignition World'),
    DeclareLaunchArgument('robot_name', default_value='ICTE4001_MRobot', description='Robot name'),
]

def generate_launch_description():

    pkg_diff_drive_robot_gz_sim = get_package_share_directory('diff_drive_robot_gz_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    control_params_file = PathJoinSubstitution(
        [pkg_diff_drive_robot_gz_sim, 'config', 'mobile_base_control.yaml']
    )

    xacro_file = os.path.join(pkg_diff_drive_robot_gz_sim, 'urdf', 'ICTE4001_MRobot_control.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters = [params]
    )

    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [LaunchConfiguration('world'), '.sdf', ' -r',' -v 4'])]
    )

    rviz_config_file = os.path.join(pkg_diff_drive_robot_gz_sim, 'rviz', 'view_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    spawn_robot = Node(
       package='ros_gz_sim', 
       executable='create',
       arguments = [
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description'
       ],
       output='screen'
    )

    diffdrive_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[control_params_file],
        arguments=['diffdrive_controller', '-c', '/controller_manager'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

        # Ensure diffdrive_controller_node starts after joint_state_broadcaster_spawner
    diffdrive_controller_callback = RegisterEventHandler(
        event_handler = OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diffdrive_controller_node],
        )
    )

    ros_ign_bridge_launch_file = os.path.join(pkg_diff_drive_robot_gz_sim, 'launch', 'ros_gz_bridge.launch.py')
    ros_ign_bridge = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(ros_ign_bridge_launch_file), 
		launch_arguments=[
			('world', LaunchConfiguration('world')), 
			('robot_name', LaunchConfiguration('robot_name'))
		]
		
	)

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(ignition_gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diffdrive_controller_callback)
    ld.add_action(rviz)
    ld.add_action(ros_ign_bridge)

    return ld
