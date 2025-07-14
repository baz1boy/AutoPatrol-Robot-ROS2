import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros  
import launch.launch_description_sources
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions

def generate_launch_description():

    # 获取功能包的share路径
    urdf_package_path = get_package_share_directory('patrol_robot_description')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'pabot/pabot.urdf.xacro')
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.world')

    # 声明一个urdf目录的参数，方便修改
    action_declare_arg_mode_path = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_xacro_path), 
        description='加载的模型文件路径'
    )
    # 通过文件路径，获取内容，并转换成参数值对象，以供传入robot_state_publisher
    xacro_model_file = launch.substitutions.Command(['xacro ', LaunchConfiguration('model')])

    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(xacro_model_file, value_type=str)
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_value}]
    )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource
        ([get_package_share_directory('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments=[('world', default_gazebo_world_path), ('verbose', 'true')]
    )

    # 请求 Gazebo 加载机器人
    action_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description', '-entity', 'pabot']
    )

     # 加载并激活 robot_joint_state_broadcaster 控制器
    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller pabot_joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    # 加载并激活 robot_diff_driver_controller 控制器
    action_load_diff_driver_controller = launch.actions.ExecuteProcess(
        cmd='ros2 control load_controller pabot_diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        action_spawn_entity,
        
        # 事件动作，当加载机器人结束后执行    
        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action = action_spawn_entity,
                on_exit = [action_load_joint_state_controller],
            )
        ),

        launch.actions.RegisterEventHandler(
            event_handler = launch.event_handlers.OnProcessExit(
                target_action = action_load_joint_state_controller,
                on_exit = [action_load_diff_driver_controller],
            )
        ),
    ])