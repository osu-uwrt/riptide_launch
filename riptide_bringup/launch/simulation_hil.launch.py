import os
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration as LC
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python import get_package_share_directory

DEFAULT_ROBOT_NAME = "tempest"
DEFAULT_ACTIVE_CONTROL_MODEL = "hybrid"


def generate_launch_description():
    return LaunchDescription([
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
        
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory("riptide_bringup2"), "launch", "simulation.launch.py")
            ),
            
            launch_arguments=[
                ('hardware', 'none'),
                ('with_zed_faker', 'True'),
                ('with_bringup', 'False'),
                ('robot', LC('robot')),
                ('active_control_enabled', LC('active_control_enabled')),
                ('active_control_model', LC('active_control_model'))
            ]
        )
    ])
