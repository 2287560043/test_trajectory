import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
# sys.path.append(os.path.join(get_package_share_directory('double_head_bringup'), 'launch'))
from launch.substitutions import Command
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown
from launch import LaunchDescription

def generate_launch_description():

    # launch_params = yaml.safe_load(open(os.path.join(
    #     get_package_share_directory('double_head_bringup'), 'config', 'launch_params.yaml')))

    # robot_description = Command(['xacro ', os.path.join(
    #     get_package_share_directory('double_head_bringup'), 'descriptions', 'urdf', 'gimbal_description.urdf.xacro'),
    #     ' left_odoom_xyz:=', launch_params['left_odoom_xyz'], ' right_odoom_xyz:=', launch_params['right_odoom_xyz'],
    #     ' left_xyz:=', launch_params['odom2leftcamera']['xyz'], ' left_rpy:=', launch_params['odom2leftcamera']['rpy'], 
    #     ' right_xyz:=', launch_params['odom2rightcamera']['xyz'], ' right_rpy:=', launch_params['odom2rightcamera']['rpy']])
    
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': robot_description,
    #                 'publish_frequency': 1000.0}]
    # )

    return LaunchDescription([
        # robot_state_publisher,
        Node(
            package='ctrl_bridge',
            executable='ctrl_bridge_node',
            emulate_tty=True,
            output='both',
        ),
    ])
