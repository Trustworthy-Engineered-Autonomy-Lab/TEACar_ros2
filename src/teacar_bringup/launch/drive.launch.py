from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    teacar_bringup_pkg = FindPackageShare('teacar_bringup')

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([teacar_bringup_pkg, 'config', 'drive_config.yaml'])
    )

    camera_launch = IncludeLaunchDescription(
        PathJoinSubstitution([teacar_bringup_pkg, 'launch', 'camera.launch.py']),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file")
        }.items()
    )

    # Joystick driver node
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_linux_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
        ]
    )
    
    # Joystick controller node
    joystick_controller_node = Node(
        package="controller",
        executable="joystick_controller",
        name="joystick_controller_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file")
        ]
    )
    
    # Parameter controller node
    param_controller_node = Node(
        package="controller",
        executable="param_controller",
        name="param_controller_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file")
        ]
    )

    # Actuator node
    pca9685_actuator_node = Node(
        package="actuator",
        executable="pca9685_actuator",
        name="pca9685_actuator_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        camera_launch,
        joy_node,
        param_controller_node,
        joystick_controller_node,
        pca9685_actuator_node
    ])