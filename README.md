# ros2_kdl_package



This package  is supposed to be used together with the [ros2_iiwa package](https://github.com/emanudevito14/iiwa_ros2) 

Use this [Dockerfile](https://github.com/emanudevito14/ros2_kdl_package/blob/vision/Dockerfile)

Clone this package in the `src` folder of your ROS 2 workspace together ros2_iiwa.
```
git clone -b 1c --single-branch https://github.com/emanudevito14/ros2_kdl_package.git
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
export IGN_GAZEBO_MODEL_PATH=$IGN_GAZEBO_MODEL_PATH:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/models:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/worlds
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"

```
Open new terminal and run
```
ros2 run ros2_kdl_package kdl_action_server_node --ros-args -p cmd_interface:=velocity

```
Open new terminal and run
```
ros2 run ros2_kdl_package kdl_action_client_node --ros-args --params-file src/ros2_kdl_package/config/waypoint.yaml


```


