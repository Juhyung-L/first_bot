from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    odom_port = LaunchConfiguration('odom_port')
    imu_port = LaunchConfiguration('imu_port')

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

    microcontoller_node = Node(
        package='microcontroller_interface',
        executable='microcontroller_interface',
        name='microcontroller_interface_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'odom_port': odom_port},
                    {'imu_port': imu_port}]
    )
    calculate_imu_covariance_node = Node(
        package='extended_kalman_filter',
        executable='calculate_imu_covariance',
        name='calculate_imu_covariance',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_odom_port,
        declare_imu_port,

        microcontoller_node,
        calculate_imu_covariance_node
    ])