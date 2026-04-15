from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare(package='cstu_group_project')
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'city.world'])
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'diffbot.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'ros2_controllers.yaml'])

    # Load URDF with xacro
    robot_description = Command(['xacro', ' ', urdf_file, ' ', 'controllers_yaml:=', controllers_yaml])

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/usr/share/gazebo-11/models'
    )

    # Start Gazebo with ROS integration
    gazebo_env = {
        'ALSOFT_DRIVERS': 'null',
        'GAZEBO_MODEL_PATH': '/usr/share/gazebo-11/models',
        'GAZEBO_MODEL_DATABASE_URI': '',
    }
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world_file,
            '-s',
            'libgazebo_ros_init.so',
            '-s',
            'libgazebo_ros_factory.so',
        ],
        output='screen',
        additional_env=gazebo_env,
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen',
        additional_env=gazebo_env,
    )

    # Publish robot description
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}]
    )

    # Spawn robot from URDF after Gazebo and robot_description are ready
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'diffbot', '-z', '0.02'],
        output='screen'
    )

    # Controller spawner for ros2_control after robot spawn
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', 'diff_drive_controller'],
        output='screen'
    )

    # CV processing nodes
    image_proc = Node(
        package='cstu_group_project',
        executable='image_processor',
        output='screen'
    )
    
    motion_ctrl = Node(
        package='cstu_group_project',
        executable='motion_controller',
        output='screen'
    )

    image_viewer = Node(
        package='cstu_group_project',
        executable='image_viewer',
        output='screen'
    )

    return LaunchDescription([
        gazebo_model_path,
        gzserver,
        gzclient,
        rsp,
        TimerAction(period=2.0, actions=[spawn]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[controller_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_spawner,
                on_exit=[motion_ctrl],
            )
        ),
        image_proc,
        image_viewer,
    ])
