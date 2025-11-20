from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('gimbal_trajectory')
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
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
        # parameters=[os.path.join(pkg_share_dir, 'config', 'params.yaml')],
        arguments=['--ros-args', '--log-level', 'info'],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        debug_arg,
        gimbal_trajectory_node,
    ])