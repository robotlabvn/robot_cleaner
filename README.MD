# ROBOT CLEANER ROS PACKAGE
This is robot_cleaner ROS package for demonstration a mobile manipulator cleaning the toilet
![robot cleaner](./fig/robot_cleaner_01.png)
## Video Demonstration 

## 1. Install ROS packages
- 1. Install gazebo_mecanum_plugins: https://github.com/qaz9517532846/gazebo_mecanum_plugins
- 2. Install universal_robot simulation package: https://github.com/ros-industrial/universal_robot
- 3. Install Descartes Motion planning: https://github.com/PickNikRobotics/descartes_capability
- 4. Install teleop keyboard 
```
sudo apt-get install ros-$ROS_DISTRO-teleop-twist-keyboard
```

- 5. Install robot_cleaner
```
cd catkin_ws/src
git clone https://github.com/robotlabvn/robot_cleaner
catkin build
```

## 2. Running 
```
roslaunch robot_cleaner spawn_ur5e_cleaner.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
roslaunch robot_cleaner moveit_planning_execution.launch sim:=true
roslaunch robot_cleaner moveit_rviz.launch
```

