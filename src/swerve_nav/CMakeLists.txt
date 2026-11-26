cmake_minimum_required(VERSION 3.20)
project(swerve_nav)

# Use C++20 for all future C++ targets in this package.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_library(swerve_kinematics STATIC
  src/swerve_kinematics.cpp
)

target_include_directories(swerve_kinematics PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

add_executable(swerve_kinematics_node
  src/swerve_kinematics_node.cpp
)
target_link_libraries(swerve_kinematics_node PRIVATE
  swerve_kinematics
  angles::angles
  geometry_msgs::geometry_msgs
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  urdf::urdf
)

install(TARGETS swerve_kinematics_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY urdf launch config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
