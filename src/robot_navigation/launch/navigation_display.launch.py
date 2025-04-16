import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    robot_navigation2_dir = get_package_share_directory(
        'robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    
    # 创建 Launch 配置
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(robot_navigation2_dir, 'maps', 'room3.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(robot_navigation2_dir, 'config', 'nav2_params2.yaml'))
    tf2_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='manual_tf_pub_map',
        arguments=['0', '0', '0.02','0', '0', '0', '1','base_link','map'],
    )
    return launch.LaunchDescription([
        tf2_node,
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
    ])