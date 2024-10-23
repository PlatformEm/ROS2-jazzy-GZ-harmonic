from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    ld = LaunchDescription()
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'my_robot.urdf.xacro'
        ])]), value_type=str
    )
    
    world_path = PathJoinSubstitution([
        FindPackageShare('my_robot_description'),
        'worlds',
        'tugbot_depot.sdf'
    ])

    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )
    
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }]
        )
    
    # Spawn robot
    spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'my_robot',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    
    bridge_params = os.path.join(
        get_package_share_directory('my_robot_description'),
        'params',
        'my_robot_bridge.yaml'
    )
    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )
    
    # RViz
    rviz =  Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'rviz',
                'urdf_config.rviz'
            ])]
        )
    
    ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(rviz)
    
    return ld