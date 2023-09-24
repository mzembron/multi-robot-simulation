### multi-robot simulation
Simple launch from nav2 lib:

`ros2 launch multi_robot_simulation unique_multi_tb3_simulation_launch.py`

Keyboard teleopearation of one of the robots:

`ros2 run teleop_twist_keyboard teleop_twist_keyboard /cmd_vel:=/robot1/cmd_vel`