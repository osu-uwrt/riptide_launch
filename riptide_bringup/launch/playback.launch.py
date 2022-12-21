import launch
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration as LC
import os

#TODO Fix as this does nothing right now
# need to figure out how to play the bags back

def generate_launch_description():
    # read the parameter for robot name
    robot_name = LC('robot')

    # create the launch description 
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='tempest', description='name of the robot to spawn'),
    ])