import os
 
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
 
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
 
from srdfdom.srdf import SRDF
 
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from moveit_configs_utils import MoveItConfigsBuilder
 
def generate_rsp_launch(moveit_config):
    """Launch file for robot state publisher (rsp)"""
 
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
 
    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)
 
    return ld
 
 
def generate_moveit_rviz_launch(moveit_config):
    """Launch file for rviz"""
    ld = LaunchDescription()
 
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
 
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]
 
    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )
 
    return ld
 
 
def generate_static_virtual_joint_tfs_launch(moveit_config):
    ld = LaunchDescription()
 
    name_counter = 0
 
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1
    return ld
 
 
def generate_spawn_controllers_launch(moveit_config):
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    ld = LaunchDescription()
    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    return ld
 
 
def generate_move_group_launch(moveit_config):
    ld = LaunchDescription()
 
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
 
    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
 
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
 
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }
 
    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]
 
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    return ld
 
 
def generate_demo_launch(moveit_config, launch_package_path=None):

    if launch_package_path is None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg(
        "db", default_value=False,
        description="By default, we do not start a database (it can be large)",
    ))
    ld.add_action(DeclareBooleanLaunchArg(
        "debug", default_value=False,
        description="By default, we are not in debug mode",
    ))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    ## Static virtual joint tfs
    virtual_joints_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    if virtual_joints_launch.exists():
        for action in generate_static_virtual_joint_tfs_launch(moveit_config).entities:
            ld.add_action(action)

    ## RSP
    for action in generate_rsp_launch(moveit_config).entities:
        ld.add_action(action)

    ## move_group
    for action in generate_move_group_launch(moveit_config).entities:
        ld.add_action(action)

    ## RViz
    for action in generate_moveit_rviz_launch(moveit_config).entities:
        ld.add_action(
            action if not hasattr(action, 'condition')
            else action
        )

    ## ros2_control_node
    ld.add_action(Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    ))

    ## Spawn controllers
    for action in generate_spawn_controllers_launch(moveit_config).entities:
        ld.add_action(action)

    return ld


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ulixarm", package_name="ulixarm_moveit_config")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)