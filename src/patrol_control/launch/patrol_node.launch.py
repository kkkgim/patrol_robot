# patrol_control/launch/patrol_system.launch.py
import os
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('patrol_control'),
        'config',
        'operation_policy.yaml'
    )

    control_node = Node(
        package='patrol_control',
        executable='control_node',
        name='control_node',
        parameters=[config],
        output='screen'
    )

    executor_node = Node(
        package='patrol_control',
        executable='executor_node',
        name='executor_node',
        parameters=[config],
        output='screen'
    )

    # 웹서버 실행 (config 파일 경로 환경변수로 전달)
    web_server = ExecuteProcess(
        cmd=['python3', '/root/patrol_robot/src/web_server/app.py'],
        additional_env={'CONFIG_PATH': config},
        output='screen'
    )

    return LaunchDescription([
        control_node,
        executor_node,
        TimerAction(period=2.0, actions=[web_server]),  # ROS노드 뜨고 2초 후 웹서버 시작
    ])