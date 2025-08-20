from launch import LaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch_ros.descriptions import ComposableNode
def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="aubo_ros2_driver",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_type",
            default_value="aubo_i10",
            description='Description with aubo robot type.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="aubo_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="aubo_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="aubo_moveit_config",
            description="MoveIt configuration package for the robot, e.g. aubo_moveit_config",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="aubo_ros2.xacro",
            description="URDF/XACRO description file with the robot, e.g. aubo.xacro",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed then also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.88.223",
            description="IP of robot computer. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "aubo_port",
            default_value="80",
            description="Port at which RWS can be found. \
            Used only if 'use_fake_hardware' parameter is false.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", default_value="false", description="Launch RViz?"
        )
    )
   
    robot_ip = LaunchConfiguration("robot_ip")


    ld = LaunchDescription(declared_arguments)

    ld.add_action(TimerAction(
        period=0.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare("aubo_ros2_driver"), '/launch', '/aubo_control.launch.py']),
                    launch_arguments={
                    "robot_ip": robot_ip,
            }.items(),
            )
        ]
    ))

    # ld.add_action(TimerAction(
    #     period=5.0,
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([
    #                 FindPackageShare("aubo_moveit_config"), '/launch', '/aubo_moveit.launch.py'])
    #         )
    #     ]
    # ))
   
    return ld
