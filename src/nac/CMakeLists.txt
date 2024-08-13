cmake_minimum_required(VERSION 3.5)
project(nac)

# Specify the C++ standard
add_compile_options(-std=c++11)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
# find_package(octocan REQUIRED)
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
  rclcpp #roscpp -> rclcpp
  rclpy  #rclpy -> rospy
  std_msgs
  geometry_msgs
  tf2  #tf->tf2
  tf2_ros            # tf_conversions
  tf2_geometry_msgs  # tf_conversions
  std_srvs
  action_msgs        # actionlib
  eigen3_cmake_module # eigen_conversions]
  kdl_parser 
  sensor_msgs
  # no need for cmake_modules
  realtime_tools
  rosbag2_storage    #rosbag
  rosbag2_transport   #rosbag
  #octocan
  #ecl_threads -> rclcpp
  #dynamic_reconfigure -> rclcpp
  Eigen3
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
