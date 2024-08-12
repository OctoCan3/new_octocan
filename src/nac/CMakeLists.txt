cmake_minimum_required(VERSION 3.5)
project(nac)

# Specify the C++ standard
add_compile_options(-std=c++11)  # ROS2 supports C++11, so we specify it here

# Find required packages using ament instead of catkin used in ROS1.
# ament_cmake is the build system replacement for catkin in ROS2.
find_package(ament_cmake REQUIRED)

# Find the rclcpp package which replaces the roscpp package from ROS1.
find_package(rclcpp REQUIRED)

# The std_msgs and geometry_msgs packages have the same name in ROS1 and ROS2.
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# The tf2 and tf2_ros packages replace the tf package from ROS1.
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# sensor_msgs package has the same name in ROS1 and ROS2.
find_package(sensor_msgs REQUIRED)

# Eigen3 is used for matrix and vector operations, similar usage in both ROS versions.
find_package(Eigen3 REQUIRED)

# Realtime_tools, if used, must be checked for compatibility in ROS2 environments.
find_package(realtime_tools REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}  # Eigen3 headers are included similarly in ROS1 and ROS2.
)

# Declare a C++ library, linking it against the ROS2 dependencies.
add_library(${PROJECT_NAME} src/nac_nn_two.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  Eigen3
  realtime_tools
)

# Optionally declare a C++ executable, showing how executables are managed in ROS2.
# add_executable(${PROJECT_NAME}_node src/nac_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   std_msgs
#   geometry_msgs
#   tf2
#   tf2_ros
#   sensor_msgs
#   Eigen3
#   realtime_tools
# )
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Installation rules for libraries and executables, similar to ROS1 but using ROS2's ament_cmake.
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Uncomment below lines for adding tests using ament in ROS2.
# ament_add_gtest(${PROJECT_NAME}-test test/test_nac.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# Registers the package with ament, essential for ROS2 packages.
ament_package()

