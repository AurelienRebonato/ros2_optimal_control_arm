cmake_minimum_required(VERSION 3.8)
project(robot_arm_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Header inclusion
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Exe creation
add_executable(controller_node 
  src/controller_node.cpp 
  src/arm.cpp 
  src/OFC_controller.cpp
  src/CTC_controller.cpp
  src/screw_math.cpp
  src/controller_base.cpp
  src/trajectory_generator.cpp
  src/RRT_planner.cpp
  src/ukf.cpp
)


target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(controller_node PUBLIC cxx_std_17)
ament_target_dependencies(controller_node rclcpp sensor_msgs std_msgs)

# Installation
install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  urdf
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()