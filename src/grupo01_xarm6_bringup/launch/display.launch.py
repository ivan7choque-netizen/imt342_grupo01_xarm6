from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch",
                "_robot_description.launch.py",
            ])
        ),
        launch_arguments={
            "dof": "6",
            "robot_type": "xarm",
            "hw_ns": "xarm",
            "add_gripper": "false",
            "add_vacuum_gripper": "false",
            "add_bio_gripper": "false",
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("xarm_description"),
                "launch",
                "_rviz_display.launch.py",
            ])
        )
    )

    gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        description,
        rviz,
        TimerAction(period=1.0, actions=[gui]),
    ])
