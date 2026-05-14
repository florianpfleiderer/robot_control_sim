cmake_minimum_required(VERSION 3.14)
project(robot_control_sim_ros2 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Reuse the standalone robot_sim_lib from the parent repository --------
# The repo lives two levels up from this package (ros2_ws/src/robot_control_sim_ros2/).
set(ROBOT_SIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")
add_subdirectory(${ROBOT_SIM_ROOT} robot_sim_lib_build EXCLUDE_FROM_ALL)

set(COMMON_DEPS
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- Nodes ----------------------------------------------------------------
add_executable(plant_node      src/plant_node.cpp)
add_executable(sensor_node     src/sensor_node.cpp)
add_executable(estimator_node  src/estimator_node.cpp)
add_executable(controller_node src/controller_node.cpp)

foreach(target plant_node sensor_node estimator_node controller_node)
  target_link_libraries(${target} robot_sim_lib)
  ament_target_dependencies(${target} ${COMMON_DEPS})
endforeach()

install(TARGETS
  plant_node
  sensor_node
  estimator_node
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
