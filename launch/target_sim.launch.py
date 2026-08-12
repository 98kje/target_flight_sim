"""target_flight_sim 런치 — 실행하면 즉시 표적 위치/속도 토픽 발행 시작.

    ros2 launch target_flight_sim target_sim.launch.py

출발점/목표점/속도 등은 config/target_sim_params.yaml 을 수정해서 바꾸면 됩니다
(재빌드 불필요, 다시 launch만 하면 새 값이 적용됩니다).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('target_flight_sim')
    default_cfg = os.path.join(share, 'config', 'target_sim_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_cfg),
        Node(
            package='target_flight_sim',
            executable='target_sim_node',
            name='target_sim',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
