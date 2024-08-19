cmake_minimum_required(VERSION 3.5)
project(octocan)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Uncomment if the package has a setup.py
# ament_python_install_package(${PROJECT_NAME})

###################################
## Declare ROS interfaces        ##
###################################
# Find package containing the ROS interfaces that your package needs
# find_package(std_msgs REQUIRED)
# find_package(example_interfaces REQUIRED)

# Generate messages and services
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/Message1.msg"
#   "srv/Service1.srv"
#   "action/Action1.action"
#   DEPENDENCIES std_msgs
# )

###################################
## ament specific configuration  ##
###################################
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/skintalk.c
  src/profile.c
  src/layout.c
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

# Declare a C++ executable
# add_executable(${PROJECT_NAME}_node src/octocan_node.cpp)

# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
# )

# Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${PROJECT_NAME}
#   ${rclcpp_LIBRARIES}
# )

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files, configurations, and other resources
# install(DIRECTORY
#   launch
#   config
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
# ament_add_gtest(test_${PROJECT_NAME} test/test_octocan.cpp)
# if(TARGET test_${PROJECT_NAME})
#   target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
# endif()

ament_package()
