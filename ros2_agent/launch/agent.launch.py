from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_agent')
    config_file = os.path.join(pkg_dir, 'config', 'robots.config.yaml')
    
    return LaunchDescription([
        Node(
            package='ros2_agent',
            executable='ros2_agent_node',
            name='agent_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'robots_config': config_file},
                {'llm_model': 'qwen3:8b'},
                {'mission_name': 'sar_mission'}
            ]
        )
    ])