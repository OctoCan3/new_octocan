cmake_minimum_required(VERSION 3.5)
project(nac)

# Compile as C++11
add_compile_options(-std=c++11)  # ROS2 supports C++11, so we specify it here

# Converted by ChatGPT
# Find ament_cmake and other dependencies
# In ROS2, `ament_cmake` replaces `catkin` as the build system
find_package(ament_cmake REQUIRED)  # Essential for ROS2 packages
find_package(rclcpp REQUIRED)       # Replaces roscpp in ROS2
find_package(std_msgs REQUIRED)     # Standard messages (same name in ROS2)
find_package(geometry_msgs REQUIRED) # Geometry messages (same name in ROS2)
find_package(tf2 REQUIRED)          # Replaces tf in ROS2
find_package(tf2_ros REQUIRED)      # Replaces tf_ros in ROS2
find_package(sensor_msgs REQUIRED) # Sensor messages (same name in ROS2)
find_package(Eigen3 REQUIRED)       # Eigen3 usage is consistent between ROS1 and ROS2
find_package(realtime_tools REQUIRED) # Replaces realtime_tools in ROS2

# If using dynamic reconfigure parameters, handle with ROS2 parameters instead
# ROS2 uses different mechanisms for parameters

# Declare messages, services, and actions if used
# ROS2 uses `rosidl_generate_interfaces()` for custom messages, services, and actions
# Uncomment and specify files as needed
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Message1.msg"
#   "srv/Service1.srv"
#   "action/Action1.action"
#   DEPENDENCIES std_msgs
# )

# Catkin-specific configuration is replaced with ament
# `ament_package()` is used to register the package
catkin_package(
  INCLUDE_DIRS include         # Include directories for header files
  LIBRARIES nac                 # Libraries to be exported
  CATKIN_DEPENDS dynamic_reconfigure ecl_threads realtime_tools roscpp rospy # ROS2 dependencies (update to ROS2 equivalents)
  DEPENDS eigen                 # External dependencies
)

# Build configuration
include_directories(
  include                       # Local include directory
  ${catkin_INCLUDE_DIRS}        # ROS1-specific, replace with ROS2 equivalents
  ${Eigen3_INCLUDE_DIRS}        # Eigen3 include directories
)

# Declare a C++ library
# In ROS2, use `ament_target_dependencies()` to link dependencies
add_library(${PROJECT_NAME} src/nac_nn_two.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp std_msgs geometry_msgs tf2 tf2_ros sensor_msgs Eigen3 realtime_tools
)

# Optionally declare a C++ executable
# Uncomment and configure if needed
# add_executable(${PROJECT_NAME}_node src/nac_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp std_msgs geometry_msgs tf2 tf2_ros sensor_msgs Eigen3 realtime_tools
# )
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# Installation rules
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/nac_controller/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Testing configuration
# Use `ament_add_gtest()` and `ament_add_nosetests()` for ROS2 testing
# Uncomment and configure if using tests
# ament_add_gtest(${PROJECT_NAME}-test test/test_nac.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
# ament_add_nosetests(test)

ament_package()  # Registers the package with ament

