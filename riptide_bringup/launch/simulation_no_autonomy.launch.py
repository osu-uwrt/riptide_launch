"""Normal simulation stack, with neither old autonomy nor a replacement executor."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from riptide_sim_config.launching import arguments, prepare


def include(package, filename, condition=None, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(share(package)) / 'launch' / filename)),
        condition=condition, launch_arguments=kwargs.items())


def generate_launch_description():
    robot = LC('robot')
    bringup = IfCondition(LC('with_bringup'))
    return LaunchDescription(arguments() + [
        DeclareLaunchArgument('active_control_model', default_value='hybrid'),
        DeclareLaunchArgument('active_control_enabled', default_value='True'),
        DeclareLaunchArgument('with_camera_faker', default_value='True'),
        DeclareLaunchArgument('with_rviz', default_value='True'),
        DeclareLaunchArgument('with_bringup', default_value='True'),
        DeclareLaunchArgument('mapping_config_yaml', default_value=str(
            Path(share('riptide_mapping2')) / 'config' / 'config.yaml'),
            description='Mapping node configuration, independent of the simulator course layout.'),
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('real_time_factor', default_value='1.0'),
        DeclareLaunchArgument('detector_params_file', default_value=str(
            Path(share('tensor_detector')) / 'config' / 'yolo_orientation.yaml')),
        DeclareLaunchArgument('camera_scale', default_value='1.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('detections', default_value='true'),
        DeclareLaunchArgument('show_tf', default_value='true'),
        DeclareLaunchArgument('show_scorecard', default_value='true'),
        DeclareLaunchArgument('initial_focus', default_value='Vehicle'),
        OpaqueFunction(function=prepare),
        SetParameter(name='use_sim_time', value=ParameterValue(LC('use_sim_time'), value_type=bool)),
        # Same components as bringup.launch.py hardware:=none, excluding only
        # autonomy.launch.py. The simulator already supplies actuator services.
        include('riptide_acoustics', 'acoustics.launch.py', bringup, robot=robot),
        include('riptide_controllers2', 'control_system.launch.py', bringup,
                robot=robot, active_control_enabled=LC('active_control_enabled'),
                active_control_model=LC('active_control_model')),
        include('riptide_hardware2', 'navigation.launch.py', bringup, robot=robot),
        include('tensor_detector', 'tensorrt.launch.py', bringup, robot=robot,
                use_sim_time=LC('use_sim_time'), params_file=LC('detector_params_file')),
        include('riptide_mapping2', 'mapping.launch.py', bringup, robot=robot,
                config_yaml=LC('mapping_config_yaml')),
        include('c_simulator', 'full_simulator.launch.py', robot=robot,
                year=LC('year'), scenario=LC('scenario'), resolved_config=LC('resolved_config'),
                with_camera_faker=LC('with_camera_faker'), with_rviz=LC('with_rviz'),
                use_sim_time=LC('use_sim_time'), real_time_factor=LC('real_time_factor')),
    ])
