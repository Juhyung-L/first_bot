import os

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, GroupAction
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False'
    )
    
    params_file = os.path.join(
        get_package_share_directory('real_launch'),
        'config',
        'params.yaml'
    )

    load_nodes = GroupAction(
        actions=[
            Node(
                package='microcontroller_interface',
                executable='microcontroller_interface',
                name='microcontroller_interface_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            params_file]),
            Node(
                package='extended_kalman_filter',
                executable='filter_node',
                name='filter_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            params_file]),
            Node(
                package='extended_kalman_filter',
                executable='visualizer',
                name='visualizer',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        
        load_nodes
    ])