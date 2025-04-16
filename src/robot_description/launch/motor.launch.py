import launch
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.parameter_descriptions

def generate_launch_description():

    # outspeed_arg = DeclareLaunchArgument(
    #     'outspeed', default_value='0', description='Output the info of speed ( 0 is no )'
    # )

    action_diff_wheel_node = launch_ros.actions.Node(
        package='robot_diff_wheel',
        executable='diff_wheel',
        # parameters=[{
        #     'outspeed': LaunchConfiguration('outspeed')
        # }]
    )

    return launch.LaunchDescription([
        # outspeed_arg,
        action_diff_wheel_node,
    ])