# ros2_kdl_package


Use this file from this repository
 
This package  is supposed to be used together with the [ros2_iiwa package](https://github.com/emanudevito14/iiwa_ros2) and [aruco_ros](https://github.com/pal-robotics/aruco_ros)


Clone this package in the `src` folder of your ROS 2 workspace together ros2_iiwa and aruco_ros.
```
git clone -b vision --single-branch https://github.com/emanudevito14/ros2_kdl_package.git
```
In the terminal 
```
colcon build 
source install/setup.bash
export IGN_GAZEBO_MODEL_PATH=$IGN_GAZEBO_MODEL_PATH:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/models
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/models:/home/user/ros2_ws/install/iiwa_description/share/iiwa_description/gazebo/worlds

ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:="true"


```
Open new terminal and run
```
ros2 launch aruco_ros single.launch.py eye:=right marker_id:=14 marker_size:=0.15


```
Open new terminal and run
```
ros2 run ros2_kdl_package aruco_setpose_client

```
Open new terminal and run
```
ros2 launch ros2_kdl_package kdl_action.launch.py 

```

Move arrow up,down , left ,right and U,D to move Aruco and R,T,F,G,H,J to rotate Aruco


