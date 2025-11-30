import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('autoaim_bring_up'), 'launch'))
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch import LaunchDescription
import time

IS_GDB = 0 
IS_HIK = 0  # 0=MindVision, 1=HIK

def generate_launch_description():
    # ==================== Trace 配置 ====================
    enable_trace_arg = DeclareLaunchArgument(
        'enable_trace',
        default_value='true',
        description='Enable ROS2 tracing (true/false)'
    )
    
    enable_trace = LaunchConfiguration('enable_trace')
    
    # 设置 trace 目录(带时间戳)
    trace_dir = f'/home/helios/.ros/tracing/autoaim_{time.strftime("%Y%m%d_%H%M%S")}'
    
    set_trace_env = SetEnvironmentVariable(
        name='ROS_TRACE_DIR',
        value=trace_dir,
        condition=IfCondition(enable_trace)
    )
    
    # ==================== 原有配置 ====================
    launch_params = yaml.safe_load(open(os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'config', 'launch_params.yaml')))

    node_params = os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'config', 'node_params.yaml')

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('autoaim_bring_up'), 'descriptions', 'urdf', 'gimbal_description.urdf.xacro'),
        ' xyz:=', launch_params['odom2camera']['xyz'], ' rpy:=', launch_params['odom2camera']['rpy']])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    # ==================== 关键优化：使用 ComposableNode ====================
    def get_camera_node(package, plugin, camera_name):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=camera_name,
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]  # 零拷贝！
        )

    def get_detector_node():
        return ComposableNode(
            package='detector_node',
            plugin='helios_cv::DetectorNode',  # 需要确认你的插件名
            name='armor_detector',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node, detector_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                detector_node
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    # 根据相机类型选择
    if IS_HIK:
        camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode', 'hik_camera')
    else:
        camera_node = get_camera_node('mindvision_camera', 'RMCamera::MVCamera', 'mv_camera')
    
    detector_node = get_detector_node()
    
    # 组合到一个容器里
    cam_detector_container = get_camera_detector_container(camera_node, detector_node)
    
    # ==================== 其他节点保持独立 ====================
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

    ctrl_bridge_node = Node(
        package='ctrl_bridge',
        executable='ctrl_bridge_node',
        name='ctrl_bridge',
        output='both',
        emulate_tty=True,
        parameters=[node_params],
        on_exit=Shutdown(),
        ros_arguments=['--ros-args', '--log-level',
                       'ctrl_bridge:='+launch_params['ctrl_bridge_log_level']],
    )

    # 延迟启动
    delay_ctrl_bridge_node = TimerAction(
        period=2.0,
        actions=[ctrl_bridge_node],
    )

    delay_armor_tracker_node = TimerAction(
        period=2.0,
        actions=[armor_tracker_node],
    )

    delay_cam_detector = TimerAction(
        period=0.5,
        actions=[cam_detector_container],  # 延迟启动整个容器
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

    autoaim_debugger = Node(
        package='autoaim_debugger',
        executable='autoaim_debugger_node',
        name='autoaim_debugger_node',
        output='both',
        emulate_tty=True,
    )

    return LaunchDescription([
        # ==================== Trace 配置(放最前面) ====================
        enable_trace_arg,
        set_trace_env,
        
        # ==================== 原有节点 ====================
        robot_state_publisher,
        delay_cam_detector,
        delay_ctrl_bridge_node,
        delay_armor_tracker_node,
        delay_energy_tracker_node,
        # foxglove_bridge,
        # autoaim_debugger,
    ])