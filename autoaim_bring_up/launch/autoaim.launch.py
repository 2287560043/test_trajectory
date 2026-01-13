import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('autoaim_bring_up'), 'launch'))
from launch.substitutions import Command
IS_GDB = 0 
IS_HIK = 0
def get_terminal_command():
    if os.environ.get('GNOME_DESKTOP_SESSION_ID'):
        return 'gnome-terminal --'
    elif os.environ.get('TERM_PROGRAM') == 'Apple_Terminal':
        return 'open -a Terminal --args'
    else:
        return 'x-terminal-emulator -e' 

def generate_launch_description():
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
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
    

    # def get_camera_node(package, plugin, camera_type):
    #     return ComposableNode(
    #         package=package,
    #         plugin=plugin,
    #         name=f'{camera_type}_camera',
    #         parameters=[node_params],
    #         extra_arguments=[{'use_intra_process_comms': True}]
    #     )

    # def get_camera_detector_container(camera_node):
    #     return ComposableNodeContainer(
    #         name='camera_detector_container',
    #         namespace='',
    #         package='rclcpp_components',
    #         executable='component_container',
    #         composable_node_descriptions=[
    #             camera_node,
    #             ComposableNode(
    #                 package='detector_node',
    #                 plugin='helios_cv::DetectorNode',
    #                 name='armor_detector',
    #                 parameters=[node_params],
    #                 extra_arguments=[{'use_intra_process_comms': True}]
    #             )
    #         ],
    #         output='both',
    #         emulate_tty=True,
    #         ros_arguments=['--ros-args', '--log-level',
    #                        'armor_detector:='+launch_params['detector_log_level']],
    #         on_exit=Shutdown(),
    #     )

    # hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode', 'hik')
    # mv_camera_node = get_camera_node('mindvision_camera', 'RMCamera::MVCamera', 'mv')

    # if (launch_params['camera'] == 'hik'):
    #     cam_detector = get_camera_detector_container(hik_camera_node)
    # elif (launch_params['camera'] == 'mv'):
    #     cam_detector = get_camera_detector_container(mv_camera_node)
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
    # if not IS_GDB:  
    #     detector_node = Node(
    #         package='detector_node',
    #         executable='detector_node_node',
    #         name='armor_detector',
    #         emulate_tty=True,
    #         output='both',
    #         parameters=[node_params],
    #         arguments=['--ros-args', '--log-level',
    #                 'armor_detector:='+launch_params['detector_log_level']],
    #         on_exit=Shutdown(),
    #     )
    # else:
    #     detector_node = Node(
    #         package='detector_node',
    #         executable='detector_node_node',
    #         name='detector_node',
    #         emulate_tty=True,
    #         output={'stderr': 'screen', 'stdout': 'screen'},
    #         parameters=[node_params],
    #         arguments=['--ros-args', '--log-level',
    #                 'armor_detector:='+launch_params['detector_log_level']],
    #         prefix=[f"{get_terminal_command()} gdb -ex run --args"],
    #     )
    # autoaim_bridge_node = Node(
    #     package='autoaim_bridge',
    #     executable='autoaim_bridge_node',
    #     name='autoaim_bridge',
    #     output='both',
    #     emulate_tty=True,
    #     parameters=[node_params],
    #     on_exit=Shutdown(),
    #     ros_arguments=['--ros-args', '--log-level',
    #                    'autoaim_bridge:='+launch_params['autoaim_bridge_log_level']],
    # )

    ctrl_bridge_node = Node(
        package = 'ctrl_bridge',
        executable = 'ctrl_bridge_node',
        name = 'ctrl_bridge',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'ctrl_bridge:='+launch_params['ctrl_bridge_log_level']],
    )

    delay_ctrl_bridge_node = TimerAction(
        period=2.0,
        actions=[ctrl_bridge_node],
    )

    # delay_autoaim_bridge_node = TimerAction(
    #     period=2.0,
    #     actions=[autoaim_bridge_node],
    # )

    delay_armor_tracker_node = TimerAction(
        period=2.0,
        actions=[armor_tracker_node],
    )

    delay_camera_node = TimerAction(
        period=0.5,
        actions=[camera_node],
    )
    
    delay_energy_tracker_node = TimerAction(
        period=2.0,
        actions=[energy_tracker_node, least_squares_node],
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', 'foxglove_bridge:=error']
    )
    node_tf2= Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        output='screen',
        arguments = ["0", "0", "0", "0", "0", "0", "odoom", "gimbal_link"]
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
        name='recorder',
        output='both',
        emulate_tty=True,
    )
    camera_imu_syncer = Node(
        package='camera_imu_syncer',
        executable='camera_imu_syncer_node',
        name='camera_imu_syncer_node',
        output='both',
        emulate_tty=True,
    )
    autoaim_armor_detector = Node(
        package='autoaim_armor_detector',
        executable='autoaim_armor_detecto_node',
        name='autoaim_armor_detector',
        emulate_tty=True,
        output={'stderr': 'screen', 'stdout': 'screen'},
        parameters=[node_params],
        arguments=['--ros-args', '--log-level',
                'armor_detector:='+launch_params['detector_log_level']],
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name='camera_imu_bridge_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            # parameters=[node_params],
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_imu_bridge',
                    plugin='helios_cv::CameraImuBridgeNode',
                    name='camera_imu_bridge',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='autoaim_armor_detector',
                    plugin='helios_cv::ArmorDetectorNode',
                    name='armor_detector_node',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                
            ],
            # prefix=[f"{get_terminal_command()} gdb -ex run --args"],
            output='both',
        ),

        robot_state_publisher,
        autoaim_debugger,
        delay_armor_tracker_node,
        delay_energy_tracker_node,

        #cam_detector,
        # delay_autoaim_bridge_node,
        # delay_camera_node,
        # detector_node,
        # delay_ctrl_bridge_node,
        # camera_imu_syncer,
        # node_tf2,
        # camera_recorder,
        # foxglove_bridge,
    ])

