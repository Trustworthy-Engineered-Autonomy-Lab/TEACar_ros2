from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    teacar_bringup_pkg = FindPackageShare('teacar_bringup')

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([teacar_bringup_pkg, 'config', 'autopilot_config.yaml'])
    )

    model_file_arg = DeclareLaunchArgument(
        "model_file",
        default_value=PathJoinSubstitution([teacar_bringup_pkg, 'models', 'best.onnx'])
    )

    # Camera, joystick and actuator
    drive_launch = IncludeLaunchDescription(
        PathJoinSubstitution([teacar_bringup_pkg, 'launch', 'drive.launch.py']),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file")
        }.items()
    )

    # Neural network controller node. It steers the car, the throttle is controlled by the joystick
    nn_controller_node = Node(
        package="controller",
        executable="nn_controller",
        name="nn_controller_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {"model_file": LaunchConfiguration("model_file")}
        ],
        remappings=[
            ("~/image_raw", "/camera/image_raw")
        ]
    )

    return LaunchDescription([
        config_file_arg,
        model_file_arg,
        drive_launch,
        nn_controller_node
    ])
