# 简介
    本项目是基于ROS2 humble的机器人项目，正在开发中......
    主要功能：导航，网页可视化，语音控制。

# 常用指令
    建图:
        ros2 launch slam_toolbox online_async_launch.py
    保存:
        ros2 run nav map_server map_saver_cli -f room
    编译:
        指定功能包:
            colcon build --packages-select 
    测试:
        source install/setup.bash
        ros2 launch robot_description motor.launch.py
        ros2 run teleop_twist_keyboard teleop_twist_keyboard 
        ros2 launch robot_description radar_motor.launch.py
        ros2 launch robot_navigation navigation_display.launch.py