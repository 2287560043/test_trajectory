from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('gimbal_trajectory')
    
    set_log_level = SetEnvironmentVariable(
        'RCUTILS_LOGGING_SEVERITY', 'INFO' 
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='False',
        description='Whether to show debug info'
    )
    
    gimbal_trajectory_node = Node(
        package='gimbal_trajectory',
        executable='gimbal_trajectory_node',
        name='gimbal_trajectory_node',
        output='screen',
        parameters=[{
            'debug': LaunchConfiguration('debug'),
        }],
        arguments=['--ros-args', '--log-level', 'gimbal_trajectory_node:=debug'],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        set_log_level, 
        debug_arg,
        gimbal_trajectory_node,
    ])