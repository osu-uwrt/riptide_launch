import os
from launch.launch_description import LaunchDescription
from launch.substitutions import LaunchConfiguration as LC
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from riptide_sim_config.launching import arguments, prepare
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python import get_package_share_directory

DEFAULT_ROBOT_NAME = "talos"
DEFAULT_ACTIVE_CONTROL_MODEL = "hybrid"

def generate_launch_description():
    return LaunchDescription(arguments() + [OpaqueFunction(function=prepare),
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
        DeclareLaunchArgument('with_rviz', default_value='True'),
        DeclareLaunchArgument('with_bringup', default_value='True',
                              description='Launch navigation, control, and autonomy alongside the simulator'),
        DeclareLaunchArgument(
            'mapping_config_yaml',
            default_value=os.path.join(
                get_package_share_directory('riptide_mapping2'), 'config', 'config.yaml'),
            description='Mapping node configuration, independent of the simulator course layout.'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description="Run the whole stack on the /clock published by the physics simulator, "
                        "so estimation follows the plant even when physics runs faster or slower than wall time."
        ),

        DeclareLaunchArgument(
            'real_time_factor',
            default_value='1.0',
            description="Simulated seconds per wall second. Change at runtime with "
                        "ros2 param set /<robot>/physics_simulator real_time_factor <value>."
        ),

        # Every node launched below, bringup and simulator alike, shares the
        # simulator's clock. This must precede the includes.
        SetParameter(name='use_sim_time',
                     value=ParameterValue(LC('use_sim_time'), value_type=bool)),

        # launch regular bringup processes
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('riptide_bringup2'),
                    'launch',
                    'bringup.launch.py'
                )
            ),
            condition=IfCondition(LC('with_bringup')),
            
            launch_arguments=[
                ('hardware', 'none'),
                ('robot', LC('robot')),
                ('active_control_enabled', LC('active_control_enabled')),
                ('active_control_model', LC('active_control_model')),
                ('mapping_config_yaml', LC('mapping_config_yaml'))
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
                ('with_camera_faker', LC('with_camera_faker')),
                ('year', LC('year')), ('scenario', LC('scenario')), ('resolved_config', LC('resolved_config')),
                ('with_rviz', LC('with_rviz')),
                ('use_sim_time', LC('use_sim_time')),
                ('real_time_factor', LC('real_time_factor')),
            ]
        )
    ])
