"""Spawn Mico in Gazebo Harmonic with ros2_control, IMU, head camera and ToF bridged to ROS 2.

ros2 launch humanoid_description mico_gz.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('humanoid_description')
    model = PathJoinSubstitution([pkg, 'urdf', 'mico.urdf.xacro'])
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', model]), value_type=str)

    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': '-r empty.sdf'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # base_link sits at the hip point H, 115 mm above the floor when the legs are straight
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'mico', '-z', '0.118'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                             '/tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'])

    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    pos = Node(package='controller_manager', executable='spawner', arguments=['position_controller'])

    return LaunchDescription([
        gz, rsp, spawn, bridge,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[pos])),
    ])
