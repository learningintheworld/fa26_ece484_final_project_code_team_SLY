"""Existing GEM simulator, Highbay world; no track truth enters perception."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share


def generate_launch_description():
    gazebo=Path(share('gem_gazebo'))
    return LaunchDescription([
        DeclareLaunchArgument('x',default_value='-8'),
        DeclareLaunchArgument('y',default_value='0'),
        DeclareLaunchArgument('yaw',default_value='0'),
        DeclareLaunchArgument('gz_args',default_value='-r -s --headless-rendering -v1 '),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',str(gazebo/'models')),
        SetEnvironmentVariable('GZ_SIM_VERSION','8'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args':[LaunchConfiguration('gz_args'),str(gazebo/'worlds/highbay_testbed.world')],
                              'gz_version':'8','on_exit_shutdown':'true'}.items()),
        Node(package='ros_gz_bridge',executable='bridge_node',parameters=[{'config_file':str(gazebo/'config/gz_bridge.yaml')}]),
        Node(package='ros_gz_image',executable='image_bridge',arguments=['/camera/image_raw']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo/'launch/gem_vehicle.launch.py')),
            launch_arguments={k:LaunchConfiguration(k) for k in ('x','y','yaw')}.items()),
    ])
