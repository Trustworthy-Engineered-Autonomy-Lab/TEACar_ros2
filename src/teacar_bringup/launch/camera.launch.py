from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from pathlib import Path
import yaml

def generate_flip_method(rotate, flip):
    rotate = ((rotate % 360) + 360) % 360

    if rotate not in (0, 90, 180, 270):
        raise ValueError(f"rotate must be 0/90/180/270, got {rotate}")

    # unified lookup table
    table = {
        ('none', 0): 0,
        ('none', 90): 1,
        ('none', 180): 2,
        ('none', 270): 3,

        ('horizontal', 0): 4,
        ('horizontal', 90): 7,
        ('horizontal', 180): 6,
        ('horizontal', 270): 5,

        ('vertical', 0): 6,
        ('vertical', 90): 5,
        ('vertical', 180): 4,
        ('vertical', 270): 7,

        ('upper-right-diagonal', 0): 5,
        ('upper-right-diagonal', 90): 4,
        ('upper-right-diagonal', 180): 7,
        ('upper-right-diagonal', 270): 6,

        ('upper-left-diagonal', 0): 7,
        ('upper-left-diagonal', 90): 6,
        ('upper-left-diagonal', 180): 5,
        ('upper-left-diagonal', 270): 4,
    }

    return table.get((flip, rotate), 0)

def generate_gs_config_str(gscam_config):
    camera_resolution = gscam_config.get('camera_resolution', [1280, 720])
    framerate = gscam_config.get("framerate", 30)

    rotate = gscam_config.get('rotate', 0)
    flip = gscam_config.get('flip', 'none')
    flip_method = generate_flip_method(rotate, flip)

    gs_pipeline = ["nvarguscamerasrc",
                   f"video/x-raw(memory:NVMM), width={camera_resolution[0]}, height={camera_resolution[1]}, format=NV12, framerate={framerate}/1",
                   f"nvvidconv flip-method={flip_method}"
                   ]
    
    caps_filter = ["video/x-raw"]
    if 'image_resolution' in gscam_config:
        image_resolution = gscam_config['image_resolution']
        caps_filter.append(f"width={image_resolution[0]}")
        caps_filter.append(f"height={image_resolution[1]}")
    if 'format' in gscam_config:
        format = gscam_config['format']
        caps_filter.append(f"format={format}")
    
    if len(caps_filter) > 1:
        gs_pipeline.append(','.join(caps_filter))

    gs_pipeline.append("videoconvert")

    gs_config_str = ' ! '.join(gs_pipeline)
    return gs_config_str

def create_gscam_node(context):
    config_file = Path(LaunchConfiguration("config_file").perform(context))

    launch_descriptions = []
    if not config_file.exists():
        parameters = []
    else:
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            gscam_node_config = config['gscam_node']['ros__parameters']
            gs_config_str = generate_gs_config_str(gscam_node_config)
            launch_descriptions.append(LogInfo(msg=f"Generated gscam config string: {gs_config_str}"))
            gscam_node_config['gscam_config'] = gs_config_str
            parameters = [gscam_node_config]
        except Exception as e:
            launch_descriptions.append(LogInfo(msg=f"Failed to generate gscam config string: {e}"))
            parameters = [config_file]

    launch_descriptions.append(
        Node(
            package="gscam",
            executable="gscam_node",
            name="gscam_node",
            output="screen",
            parameters=parameters
        )
    )

    return launch_descriptions
        

def generate_launch_description():

    teacar_bringup_pkg = FindPackageShare('teacar_bringup')

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution([
            teacar_bringup_pkg,
            "config",
            "camera_config.yaml"
        ]),
        description="Config file for camera"
    )

    return LaunchDescription([
        config_file_arg,
        OpaqueFunction(function=create_gscam_node)
    ])
    