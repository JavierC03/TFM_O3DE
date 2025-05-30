cmake_minimum_required(VERSION 3.8)
project(manipulation_o3de)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)

# Ejecutable test_moveit
add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  yolo_msgs
  geometry_msgs
  moveit_core
)
target_compile_features(test_moveit PUBLIC cxx_std_17)
install(TARGETS test_moveit DESTINATION lib/${PROJECT_NAME})

# Ejecutable yolo_moveit
add_executable(yolo_moveit src/yolo_moveit.cpp)
ament_target_dependencies(yolo_moveit
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  yolo_msgs
  geometry_msgs
  moveit_core
)
target_compile_features(yolo_moveit PUBLIC cxx_std_17)
install(TARGETS yolo_moveit DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
