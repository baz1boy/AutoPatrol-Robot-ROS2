import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取与拼接默认路径
    pabot_navigation2_dir = get_package_share_directory('patrol_robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 声明新的 Launch 参数
    declare_use_sim_time = DeclareLaunchArgument(
        name = 'use_sim_time', 
        default_value = 'true',
        description = 'Use simulation (Gazebo) clock if true')
    
    declare_map_yaml = DeclareLaunchArgument(
        name = 'map', 
        default_value=os.path.join(pabot_navigation2_dir, 'maps', 'room.yaml'),
        description = 'Full path to map file to load')
    
    # 复制nav2中默认的nav2_params.yaml作为配置文件，在其中修改
    declare_params_file = DeclareLaunchArgument(
        name = 'params_file', 
        default_value=os.path.join(pabot_navigation2_dir, 'config', 'nav2_params.yaml'),
        description = 'Full path to param file to load')

    # 使用参数值
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_path = LaunchConfiguration('map')
    nav2_param_path = LaunchConfiguration('params_file')

    # 启动Navigation2
    action_launch_bringup = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        # 使用 Launch 参数替换原有参数
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items()
    )
        
    action_launch_rviz2 = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return launch.LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        declare_params_file,

        action_launch_bringup,
        action_launch_rviz2
    ])