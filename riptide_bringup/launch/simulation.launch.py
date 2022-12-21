from distutils.command.config import config
from signal import pause
from time import sleep
from click import launch

from scipy import rand
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration as LC
from riptide_SNIB import simulinkControl, gazeboControl
import os

default_robot_name = "tempest"

# all of the robot namespaced launch files to start
ns_launch_files = [
    os.path.join(
        get_package_share_directory('riptide_controllers2'),
        'launch',
        'controller.launch.py'),
    os.path.join(
        get_package_share_directory('riptide_mapping2'),
        'launch',
        'mapping.launch.py'),
    os.path.join(
        get_package_share_directory('riptide_SNIB'),
        "launch",
        "snib.launch.py"
    ),
    os.path.join(
        get_package_share_directory('riptide_localization2'),
        'launch',
        'navigation.launch.py'
    ),
    os.path.join(
        get_package_share_directory('uwrt_ros_gz'),
        "launch",
        "uwrt_ros_gz.launch.py"
    ),
]

simulinkControl.launchSimulink()
gazeboControl.launchGazebo()

def generate_launch_description():
    # read the parameter for robot name
    robot_name = LC('robot')

    # setup a list to collect launch descriptions
    ns_descrips = []

    # setup a namespace to put everything in
    ns_descrips.append(PushRosNamespace(robot_name))

    # iterate the list of launch files we were given to start
    for launch_file in ns_launch_files:
        ns_descrips.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(launch_file),
                launch_arguments=[
                    ('namespace', robot_name),
                    ('robot', robot_name),
                ]
            )
        )

    # create the launch description 
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value=default_robot_name, description='name of the robot to spawn'),

        GroupAction(ns_descrips),
    ])