from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    pkg_share = FindPackageShare('ulixarm_description')

    # --- Launch Arguments ulixarm_descritpion ---
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf']),
        description='Absolute path to the robot URDF file'
    )


    pkg_share = FindPackageShare('demos')
    
    # --- Launch Arguments tutorial ---
    rviz_config_arg = DeclareLaunchArgument(
        name='rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz']),
        description='Absolute path to the RViz config file'
    )

    # --- Nodes ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['cat ', LaunchConfiguration('model')])
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])