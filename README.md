# ros2_kdl_package



This package  is supposed to be used together with the [ros2_iiwa package](https://github.com/RoboticsLab2025/ros2_iiwa) 


Clone this package in the `src` folder of your ROS 2 workspace together ros2_iiwa.
```
git clone 
```
Build your new package
```
colcon build 
```
Source the setup files
```
source install/setup.bash
```
launch this
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"

```
Open new terminal and run
```
ros2 run ros2_kdl_package kdl_action_server_node --ros-args -p cmd_interface:=velocity

```
Open new terminal and run
```
ros2 run ros2_kdl_package kdl_action_client_node

```


