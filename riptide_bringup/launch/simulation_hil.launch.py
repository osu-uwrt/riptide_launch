import os
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration as LC
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from riptide_sim_config.launching import arguments
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python import get_package_share_directory

DEFAULT_ROBOT_NAME = "talos"
DEFAULT_ACTIVE_CONTROL_MODEL = "hybrid"
DEFAULT_HIL_CAMERA_SCALE = "0.5"


def generate_launch_description():
    return LaunchDescription(arguments() + [
        DeclareLaunchArgument(
            "robot", 
            default_value=DEFAULT_ROBOT_NAME, 
            description="Name of the robot"
        ),
        
        DeclareLaunchArgument(
            'active_control_model',
            default_value=DEFAULT_ACTIVE_CONTROL_MODEL,
            description="Name of the active control model to use."
        ),
        
        DeclareLaunchArgument(
            'active_control_enabled',
            default_value="True",
            description="Whether or not the active control should be launched with the system."
        ),

        # Raw sim images cross the network to the robot's zenoh router in HIL.
        # Full-res 32FC1 depth (~9 MB/frame) saturates gigabit, so raw topics
        # never arrive. Scales cameras.yaml resolution_scale; normal sim is unaffected.
        DeclareLaunchArgument(
            'camera_scale',
            default_value=DEFAULT_HIL_CAMERA_SCALE,
            description="Sim camera resolution multiplier for HIL (1.0 = native ZED resolution)."
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory("riptide_bringup2"), "launch", "simulation.launch.py")
            ),
            
            launch_arguments=[
                ('with_camera_faker', 'True'),
                ('with_bringup', 'False'),
                ('robot', LC('robot')),
                ('year', LC('year')), ('scenario', LC('scenario')),
                ('resolved_config', LC('resolved_config')),
                ('active_control_enabled', LC('active_control_enabled')),
                ('active_control_model', LC('active_control_model')),
                ('camera_scale', LC('camera_scale'))
            ]
        )
    ])
