# TurtleBot4

## 建图

1. 键盘遥控

ros2 run teleop_twist_keyboard teleop_twist_keyboard

2. 启动同步见图
ros2 launch turtlebot4_navigation slam_sync.launch.py

3. 可视化建图过程
ros2 launch turtlebot4_viz view_robot.launch.py

4. 保存地图

ros2 run nav2_map_server map_saver_cli -f <map_name>

## 导航

1. 运行导航程序（注意指定地图路径）
ros2 launch turtlebot4_navigation nav_bringup.launch.py slam:=off localization:=true map:=/home/lym/map.yaml

2. 启动可视化界面
ros2 launch turtlebot4_viz view_robot.launch.py

3. 保存雷达数据到文档

ros2 topic echo /scan > a.txt
