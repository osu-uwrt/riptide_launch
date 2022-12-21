import launch
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration as LC
import os
from pathlib import Path

home = str(Path.home())

# all of the launch files to start
bringup_file = os.path.join(
        get_package_share_directory('riptide_bringup2'),
        'launch',
        'bringup.launch.py'),


def generate_launch_description():
    # read the parameter for robot name
    robot_name = LC('robot')
    bag_path = LC('bag_path')

    # setup a list to collect launch descriptions
    bags = [
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 
                launch.substitutions.PathJoinSubstitution([
                    bag_path,
                    'rosbag_video'
                ]),
                '/rosout_agg',
                'stereo/left/image_raw/compressed ',
                'stereo/left/camera_info',
                'stereo/right/image_raw/compressed',
                'stereo/right/camera_info',
            ],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 
                launch.substitutions.PathJoinSubstitution([
                    bag_path,
                    'rosbag_sensors'
                ]),
                '/rosout_agg',
                'dvl/twist',
                'dvl_twist',
                'dvl_sonar0',
                'dvl_sonar1',
                'dvl_sonar2',
                'dvl_sonar3',
                'depth/raw',
                'depth/pose',
                'imu/data',
                '/tf',
            ],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 
            launch.substitutions.PathJoinSubstitution([
                bag_path,
                'rosbag_mapping'
            ]),
            '/rosout_agg',
            'mapping/cutie',
            'mapping/tommy',
            'mapping/gman',
            'mapping/bootlegger',
            'mapping/badge',
            'mapping/gate',
            'dope/detected_objects',
            ],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
            launch.substitutions.PathJoinSubstitution([
                bag_path,
                'rosbag_diagnostics'
            ]),
            '/rosout_agg',
            '/diagnostics',
            '/diagnostics_agg',
            '/diagnostics_toplevel_state',
            ],
            output='screen'
        )
    ]

    # create the launch description 
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tempest', description='name of the robot to spawn'),
        DeclareLaunchArgument('bag_path', default_value=home, description='the directory to put the bags in'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(bringup_file),
                launch_arguments=[
                    ('robot', LC('robot')),
                ]
            ),
        GroupAction(bags)
    ])