import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

import launch_ros.parameter_descriptions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    urdf_package_path = get_package_share_directory('robot_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf/robot/', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(urdf_package_path, 'config', 'display_robot_model.rviz')
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_urdf_path), description='The path of loading model file'
    )
    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_value
        }]
    )
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_config_path]
    )
    action_diff_wheel_node = launch_ros.actions.Node(
        package='robot_diff_wheel',
        executable='diff_wheel'
    )

    # 雷达
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               urdf_package_path, 'params', 'X2.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')
    radar_driver_node = launch_ros.actions.LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )
    tf2_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02','0', '0', '0', '1','laser_link','laser_frame'],
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_diff_wheel_node,
        params_declare,
        radar_driver_node,
        tf2_node,
        action_rviz_node,
    ])