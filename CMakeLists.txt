cmake_minimum_required(VERSION 3.8)
project(ros2_kdl_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- Generate messages for actions ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionError.msg"
  "action/LinearTrajectory.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
# --- Include directories ---
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# --- Action nodes ---

add_executable(kdl_action_client_node src/ros2_kdl_action_client.cpp)
ament_target_dependencies(kdl_action_client_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  orocos_kdl
)
target_compile_features(kdl_action_client_node PUBLIC cxx_std_17)

add_executable(kdl_action_server_node 
  src/ros2_kdl_action_server.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)
ament_target_dependencies(kdl_action_server_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  orocos_kdl
  kdl_parser
  sensor_msgs
)
target_compile_features(kdl_action_server_node PUBLIC cxx_std_17)

rosidl_get_typesupport_target(kdl_cpp_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kdl_action_client_node ${kdl_cpp_typesupport})
target_link_libraries(kdl_action_server_node ${kdl_cpp_typesupport})

# --- Legacy nodes ---
add_executable(ros2_kdl_node
  src/ros2_kdl_node.cpp
  src/kdl_robot.cpp
  src/kdl_planner.cpp
  src/kdl_control.cpp
)
target_include_directories(ros2_kdl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ros2_kdl_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node 
  orocos_kdl
  rclcpp
  kdl_parser
  sensor_msgs
  std_msgs
)

add_executable(ros2_kdl_node_example src/ros2_kdl_node_example.cpp)
target_include_directories(ros2_kdl_node_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ros2_kdl_node_example PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ros2_kdl_node_example 
  orocos_kdl
  rclcpp
)

# --- Install targets ---
install(TARGETS
  kdl_action_client_node
  kdl_action_server_node
  ros2_kdl_node
  ros2_kdl_node_example
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install include, scripts, launch, config ---
install(DIRECTORY include/
  DESTINATION include/
)
install(PROGRAMS
  scripts/compare.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch bags config
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

