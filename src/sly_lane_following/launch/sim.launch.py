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
        # Same official description, spawn and ros2_control controllers. Only
        # the Ackermann driver's TF startup wait is replaced.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('gem_description'))/'launch/gem_description.launch.py')),
            launch_arguments={'use_sim_time':'true','start_rviz':'false'}.items()),
        Node(package='ros_gz_sim',executable='create',arguments=[
            '-topic','robot_description','-name','gem','-allow_renaming','false',
            '-x',LaunchConfiguration('x'),'-y',LaunchConfiguration('y'),'-z','0.3',
            '-Y',LaunchConfiguration('yaw')],parameters=[{'use_sim_time':True}]),
        Node(package='controller_manager',executable='spawner',arguments=[
            'joint_state_broadcaster','left_steering_controller','right_steering_controller',
            'left_front_wheel_controller','right_front_wheel_controller',
            'left_rear_wheel_controller','right_rear_wheel_controller']),
        Node(package='sly_lane_following',executable='sim_ackermann',name='ackermann_controller',
            parameters=[{'cmd_timeout':.5,'use_sim_time':True},
                        str(gazebo/'config/gem_ackermann_control_params_ros2.yaml')]),
    ])
