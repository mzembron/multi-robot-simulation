################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(multi_robot_simulation)


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

# arm simulation
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)

include_directories(include)
link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
)

################################################################################
# Install
################################################################################
install(DIRECTORY launch config models rviz urdf worlds params
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot_arm.dsv.in")

ament_package()
