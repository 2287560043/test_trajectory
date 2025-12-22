import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('autoaim_bring_up'), 'launch'))
from launch.substitutions import Command
from launch.actions import TimerAction, Shutdown
IS_GDB = 0 
IS_HIK = 1
def get_terminal_command():
    if os.environ.get('GNOME_DESKTOP_SESSION_ID'):
        return 'gnome-terminal --'
    elif os.environ.get('TERM_PROGRAM') == 'Apple_Terminal':
        return 'open -a Terminal --args'
    else:
        return 'x-terminal-emulator -e' 


def generate_launch_description():

    from launch_ros.actions import Node
    from launch import LaunchDescription

    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'config', 'launch_params.yaml')))

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'descriptions', 'urdf', 'gimbal_description.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    node_params = os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'config', 'node_params.yaml')

    camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        name='mv_camera',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        parameters=[node_params],
    )
    if IS_HIK:
        camera_node = Node(
            package='hik_camera',           
            executable='hik_camera_node',  
            name='hik_camera',      
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
            parameters=[node_params],
        )
    else:
        camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        name='mv_camera',
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
        parameters=[node_params],
    )


    armor_tracker_node = Node(
        package='armor_predictor',
        executable='armor_predictor_node',
        name='armor_predictor',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--log-level', 'armor_predictor_node:='+launch_params['predictor_log_level']],
    )
    
    energy_tracker_node = Node(
        package='energy_predictor',
        executable='energy_predictor_node',
        name='energy_predictor',
        output='both',
        on_exit=Shutdown(),
        emulate_tty=True,
        parameters=[node_params],
        ros_arguments=['--log-level', 'energy_predictor_node:='+launch_params['predictor_log_level']],
    )

    least_squares_node = Node(
        package='energy_predictor',
        executable='least_squares_node',   
    )
    if not IS_GDB:  
        detector_node = Node(
            package='detector_node',
            executable='detector_node_node',
            name='armor_detector',
            emulate_tty=True,
            output='both',
            parameters=[node_params],
            arguments=['--ros-args', '--log-level',
                    'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )
    else:
        detector_node = Node(
            package='detector_node',
            executable='detector_node_node',
            name='detector_node',
            emulate_tty=True,
            output={'stderr': 'screen', 'stdout': 'screen'},
            parameters=[node_params],
            arguments=['--ros-args', '--log-level',
                    'armor_detector:='+launch_params['detector_log_level']],
            prefix=[f"{get_terminal_command()} gdb -ex run --args"],
        )
    node_tf2_yaw= Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        output='screen',
        arguments = ["0", "0", "0", "0", "0", "0", "odoom", "yaw_link"]
    )

    node_tf2_pitch= Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        output='screen',
        arguments = ["0", "0", "0", "0", "0", "0", "yaw_link", "pitch_link"]
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', 'foxglove_bridge:=error']
    )

    autoaim_debugger = Node(
        package='autoaim_debugger',
        executable='autoaim_debugger_node',
        name='autoaim_debugger_node',
        output='both',
        emulate_tty=True,
    )

    camera_recorder = Node(
        package='camera_recorder',
        executable='camera_recorder_node',
        name='camera_recorder_node',
        output='both',
        emulate_tty=True,
    )

    priority_client = Node(
        package='autoaim_priority',
        executable='autoaim_priority_node',
        name='autoaim_priority',
        output='both',
    )

    return LaunchDescription([
        robot_state_publisher,
        camera_node,
        detector_node,
        # armor_tracker_node,
        energy_tracker_node,
        least_squares_node,
        node_tf2_yaw,
        node_tf2_pitch,
        # autoaim_debugger,
        # camera_recorder,
        foxglove_bridge,
        # priority_client,
    ])

