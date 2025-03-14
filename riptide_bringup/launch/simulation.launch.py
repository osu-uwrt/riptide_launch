import os
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration as LC
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python import get_package_share_directory

DEFAULT_ROBOT_NAME = "tempest"
DEFAULT_ACTIVE_CONTROL_MODEL = "hybrid"

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value=DEFAULT_ROBOT_NAME,
            description="Name of the robot to use."
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
        
        DeclareLaunchArgument(
            "with_camera_faker",
            default_value="True",
            description="Enable or disable the camera faker"
        ),
        
        DeclareLaunchArgument(
            'with_tensorrt',
            default_value="True",
            description="Whether or not to launch tensorrt."
        ),
        
        # launch regular bringup processes
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('riptide_bringup2'),
                    'launch',
                    'bringup.launch.py'
                )
            ),
            
            launch_arguments=[
                ('hardware', 'none'),
                ('robot', LC('robot')),
                ('active_control_enabled', LC('active_control_enabled')),
                ('active_control_model', LC('active_control_model')),
                ('with_tensorrt', LC('with_tensorrt'))
            ]
        ),
        
        #launch simulator 
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('c_simulator'),
                    'launch',
                    'full_simulator.launch.py'
                )
            ),
            
            launch_arguments=[
                ('robot', LC('robot')),
                ('with_camera_faker', LC('with_camera_faker'))
            ]
        )
    ])
