## Install
1. Install gazebo_mecanum_plugins: https://github.com/qaz9517532846/gazebo_mecanum_plugins
2. Install universal_robot simulation package: https://github.com/ros-industrial/universal_robot
1. Install Descartes Motion planning: https://github.com/PickNikRobotics/descartes_capability

roslaunch robot_cleaner spawn_ur5e_cleaner.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
roslaunch robot_cleaner moveit_planning_execution.launch sim:=true
roslaunch robot_cleaner moveit_rviz.launch