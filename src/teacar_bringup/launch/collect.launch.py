from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    teacar_bringup_pkg = FindPackageShare('teacar_bringup')

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([teacar_bringup_pkg, 'config', 'collect_config.yaml'])
    )

    drive_launch = IncludeLaunchDescription(
        PathJoinSubstitution([teacar_bringup_pkg, 'launch', 'drive.launch.py']),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file")
        }.items()
    )

    recorder_node = Node(
        package="recorder",
        executable="recorder",
        name="recorder_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")]
    )

    return LaunchDescription([
        config_file_arg,
        drive_launch,
        recorder_node
    ])
