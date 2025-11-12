# ros2_kdl_package


This package is supposed to be used together with the [ros2_iiwa package](https://github.com/RoboticsLab2025/ros2_iiwa) 


Clone this package in the `src` folder of your ROS 2 workspace and put [ros2_iiwa package](https://github.com/RoboticsLab2025/ros2_iiwa) in `src`  .
```
git clone https://github.com/emanudevito14/ros2_kdl_package.git
```
Go in root of your workspace and build
```
colcon build 
```
Source the setup files
```
. install/setup.bash
```


Run the node
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```
Open new terminal and launch
```
ros2 launch ros2_kdl_package ros2_kdl.launch.py ctrl:="velocity_ctrl"

```
or 
```
ros2 launch ros2_kdl_package ros2_kdl.launch.py ctrl:="velocity_ctrl_null"

```
Open new terminal and compare commanded velocity and joint velocity both controller
```
ros2 run ros2_kdl_package compare.py -- --joint=0 

```
or
```
ros2 run ros2_kdl_package compare.py -- --joint=1
ros2 run ros2_kdl_package compare.py -- --joint=2
ros2 run ros2_kdl_package compare.py -- --joint=3
ros2 run ros2_kdl_package compare.py -- --joint=4
ros2 run ros2_kdl_package compare.py -- --joint=5
ros2 run ros2_kdl_package compare.py -- --joint=6
 
```


