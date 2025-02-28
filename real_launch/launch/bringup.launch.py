import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # pkg_share = get_package_share_directory('real_launch')
    sllidar_pkg_share = get_package_share_directory('sllidar_ros2')
    slam_toolbox_pkg_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_port = LaunchConfiguration('odom_port')
    imu_port = LaunchConfiguration('imu_port')
    lidar_port = LaunchConfiguration('lidar_port')
    
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False'
    )
    declare_odom_port = DeclareLaunchArgument(
        name='odom_port',
        default_value='/dev/ttyACM0'
    )
    declare_imu_port = DeclareLaunchArgument(
        name='imu_port',
        default_value='/dev/ttyUSB0'
    )
    declare_lidar_port = DeclareLaunchArgument(
        name='lidar_port',
        default_value='/dev/ttyUSB1'
    )

    # 170-degrees CCW rotation about z-axis
    lidar_to_base_footprint_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0.996195','0.087156','base_footprint','laser']
    )        
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sllidar_pkg_share, 'launch', 'sllidar_a1_launch.py')),
        launch_arguments={
            'serial_port': lidar_port
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_pkg_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_odom_port,
        declare_imu_port,
        declare_lidar_port,
        
        lidar_to_base_footprint_static_tf,
        lidar_launch,
        slam_launch
    ])
