# 简介
    本项目是基于ROS2 humble的机器人项目，正在开发中......
    主要功能：导航，网页可视化，语音控制。

# 项目结构
    ├──src
    │   ├──robot_description // 可视化
    │   ├──robot_diff_wheel // 双轮差速驱动
    │   │   ├──src
    │   │   │   ├──main.c // 主程序
    │   │   │   ├──message.c // 通信
    │   ├──robot_flask // 网页端、
    │   │   ├──models // 模型
    │   │   ├──views // 网页
    │   │   ├──robot_flask
    │   │   │   ├──app.py // 服务器程序
    │   │   │   ├──voice_control.py // 语音程序
    │   ├──robot_navigation // 导航
    │   ├──robot_ydlidar_x2 // 雷达驱动
    │   ├──robot_jps_planner // JPS算法

# 常用指令
    Edge浏览器限制了局域网的权限
    使用edge://flags/#unsafely-treat-insecure-origin-as-secure添加信任网站
    建图:
        ros2 launch slam_toolbox online_async_launch.py
    保存:
        ros2 run nav2_map_server map_saver_cli -f room
    编译:
        指定功能包:
            colcon build --packages-select 
    测试:
        source install/setup.bash
        ros2 launch robot_description motor.launch.py
        ros2 run teleop_twist_keyboard teleop_twist_keyboard 
        ros2 launch robot_description radar_motor.launch.py
        ros2 launch robot_navigation test.launch.py
        /bin/python3 /home/ros_system/ros_workspace/src/robot_flask/robot_flask/voice_control.py