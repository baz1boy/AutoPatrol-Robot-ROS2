import launch
import launch_ros  
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    autopatrol_robot_path = get_package_share_directory('autopatrol_robot')
    default_patrol_config_path = os.path.join(autopatrol_robot_path, 'config', 'patrol_config.yaml')

    action_patrol_node = launch_ros.actions.Node(
        package='autopatrol_robot', # 包名
        executable='patrol_node',   # 文件名
        output='screen',
        parameters=[default_patrol_config_path],
    )

    action_speaker_node = launch_ros.actions.Node(
        package='autopatrol_robot',
        executable='speaker',
        output='screen',
    )

    return launch.LaunchDescription([
        action_speaker_node,
        action_patrol_node,
    ])