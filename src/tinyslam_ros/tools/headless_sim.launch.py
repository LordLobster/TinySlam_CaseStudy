# Same as ICTE4001_MRobot.launch.py but: Gazebo server only (-s), no RViz, office world
import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('diff_drive_robot_gz_sim')
    gz = get_package_share_directory('ros_gz_sim')
    doc = xacro.process_file(os.path.join(pkg, 'urdf', 'ICTE4001_MRobot_sensor.urdf.xacro'))
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               parameters=[{'robot_description': doc.toxml(), 'use_sim_time': True}])
    sim = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
                                   launch_arguments=[('gz_args', 'office_world.sdf -r -s -v 2')])
    spawn = Node(package='ros_gz_sim', executable='create', arguments=['-name', 'ICTE4001_MRobot', '-topic', 'robot_description'])
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '-c', '/controller_manager'])
    dd = Node(package='controller_manager', executable='spawner', arguments=['diffdrive_controller', '-c', '/controller_manager'])
    bridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'ros_gz_bridge.launch.py')),
                                      launch_arguments=[('world', 'office_world'), ('robot_name', 'ICTE4001_MRobot')])
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'world')),
        rsp, sim, spawn, jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[dd])),
        bridge])
