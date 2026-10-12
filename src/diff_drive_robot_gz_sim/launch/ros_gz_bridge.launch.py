
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('robot_name', default_value='ICTE4001_MRobot',
                          description='Ignition model name'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='World name'),
	DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')

    # clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        namespace=namespace,
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ],
                        condition=IfCondition(use_sim_time)
	)
                                                   
    # lidar bridge
    lidar_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
    										namespace=namespace,
    										name='lidar_bridge',
    										output='screen',
    										parameters=[{'use_sim_time': use_sim_time}],
    										arguments=[
    											['/world/', LaunchConfiguration('world'),
    											'/model/', LaunchConfiguration('robot_name'),
    											'/link/lidar_link/sensor/lidar_sensor/scan'+
    											'@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
    											]
    										],
    										remappings=[(
    											['/world/', LaunchConfiguration('world'),
    											'/model/', LaunchConfiguration('robot_name'), 
    											'/link/lidar_link/sensor/lidar_sensor/scan'], 
    											'/scan')
    										]
    )
    
    #imu bridge
    imu_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
    									namespace=namespace,
    									name='imu_bridge',
    									output='screen',
    									parameters=[{'use_sim_time': use_sim_time}],
    									arguments = [
    										['/world/', LaunchConfiguration('world'),
    										'/model/', LaunchConfiguration('robot_name'),
    										'/link/imu_link/sensor/imu_sensor/imu'+
    										'@sensor_msgs/msg/Imu[gz.msgs.IMU'
    										]
    									],
    									remappings=[(
		  									['/world/', LaunchConfiguration('world'),
		  									'/model/', LaunchConfiguration('robot_name'),
		  									'/link/imu_link/sensor/imu_sensor/imu'], '/imu')
    									]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(clock_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(imu_bridge)
    return ld
