"""Mico in Gazebo Harmonic with ros2_control, IMU, ToF, head camera and ground-truth odometry, viewed in Foxglove.

Headless server in the Ubuntu VM (OrbStack), Foxglove on the Mac at ws://localhost:8765:
    ros2 launch humanoid_description mico_gz.launch.py
    ros2 launch humanoid_description mico_gz.launch.py gait:=true          # also run the walk / turn / wave demo
    ros2 launch humanoid_description mico_gz.launch.py gui:=true camera:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    camera = LaunchConfiguration('camera')
    foxglove = LaunchConfiguration('foxglove')
    gait = LaunchConfiguration('gait')
    args = [
        DeclareLaunchArgument('gui', default_value='false', description='open the Gazebo GUI (needs a display)'),
        DeclareLaunchArgument('camera', default_value='true', description='simulate the head camera and ToF'),
        DeclareLaunchArgument('foxglove', default_value='true', description='start foxglove_bridge on port 8765'),
        DeclareLaunchArgument('gait', default_value='false', description='run the mico_gait demo'),
    ]

    model = PathJoinSubstitution([FindPackageShare('humanoid_description'), 'urdf', 'mico.urdf.xacro'])
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', model, ' camera:=', camera]),
        value_type=str)

    # -s: server only; --headless-rendering lets the camera and ToF render without a display (EGL)
    gz_args = PythonExpression(["'-r empty.sdf' if '", gui, "' == 'true' else '-r -s --headless-rendering empty.sdf'"])
    gz_launch = PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_launch),
                                  launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    # base_link sits at the hip point H, 115 mm above the floor when the legs are straight
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'mico', '-z', '0.118'])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'use_sim_time': True}],
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                             '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                             '/tof@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                             '/model/mico/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                             '/model/mico/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
                  remappings=[('/model/mico/odometry', '/odom'), ('/model/mico/pose', '/tf')])

    fox = Node(package='foxglove_bridge', executable='foxglove_bridge', output='screen',
               condition=IfCondition(foxglove), parameters=[{'port': 8765, 'use_sim_time': True}])

    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    jtc = Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller'])
    # same joints, so it stays inactive; switch to it to stream raw targets on /position_controller/commands
    pos = Node(package='controller_manager', executable='spawner', arguments=['position_controller', '--inactive'])
    walk = Node(package='humanoid_control', executable='mico_gait.py', output='screen', condition=IfCondition(gait),
                parameters=[{'use_sim_time': True}])

    return LaunchDescription(args + [
        gz, rsp, spawn, bridge, fox,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
        RegisterEventHandler(OnProcessExit(target_action=jtc, on_exit=[pos, walk])),
    ])
