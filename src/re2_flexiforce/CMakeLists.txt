cmake_minimum_required(VERSION 3.0.2)
project(re2_flexiforce)

# Specify the C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_PREFIX_PATH "/opt/ros/humble" ${CMAKE_PREFIX_PATH})

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Need to add neuromeka libraries, not hebi

find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(octocan REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add additional dependencies as necessary


## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NAC_input.msg"
  "msg/OctoData.msg"
  DEPENDENCIES std_msgs  # Add any other dependencies your messages need
)


## Generate messages and services with dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NAC_input.msg"
  "msg/OctoData.msg"
  DEPENDENCIES std_msgs  # Or other packages containing msgs
)

###################################
## ament specific configuration ##
###################################
## The ament_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## ament_export_dependencies: list of ament packages that dependent projects also need
## ament_export_include_directories: paths to be included for dependent projects
## ament_export_libraries: libraries you create in this project that dependent projects also need

ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs)
ament_export_libraries(${PROJECT_NAME})

ament_package()


###########
## Build ##
###########

## Specify additional locations of header files
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

## Install Python scripts
install(PROGRAMS scripts/flexiforce.py
  DESTINATION lib/${PROJECT_NAME}
)
  
add_executable(
  joint_state_pub
  src/joint_state_pub.cpp )

add_executable(
  NAC_node
  src/neuroAdaptControl_node.cpp )

add_executable(
  octocan_pub
  src/OctoCan_pub.cpp)

add_executable(
  RE2IK_pub
  src/RE2_ik_node.cpp)

add_executable(
  sensor_calib
  src/RE2_Sensor_Calib.cpp)

add_executable(
  quadcan_calib
  src/RE2_QuadCan_Calib.cpp)


add_executable(
  re2_fk
  src/RE2_fk_node.cpp)

add_executable(
  NAC_Trajectory_Testing
  src/NAC_Trajectory_Testing.cpp )

add_dependencies(
  joint_state_pub
  ${catkin_EXPORTED_TARGETS}
)

# Add the executable
add_executable(NAC_node src/neuroAdaptControl_node.cpp)

# Link the executable with necessary libraries and message dependencies
ament_target_dependencies(NAC_node rclcpp std_msgs your_custom_msgs)

#Install the executable
install(TARGETS NAC_node
  DESTINATION lib/${PROJECT_NAME}
)


add_dependencies(
  octocan_pub
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  RE2IK_pub
  ${catkin_EXPORTED_TARGETS}
)


add_dependencies(
  sensor_calib
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  quadcan_calib
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  re2_fk
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  NAC_Trajectory_Testing
  ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(
  joint_state_pub
 ${catkin_LIBRARIES}
)

target_link_libraries(
  NAC_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
  octocan_pub
  ${catkin_LIBRARIES}
)

target_link_libraries(
  RE2IK_pub
  ${catkin_LIBRARIES}
)


target_link_libraries(
  sensor_calib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  quadcan_calib
  ${catkin_LIBRARIES}
)

target_link_libraries(
  re2_fk
  ${catkin_LIBRARIES}
)

target_link_libraries(
  NAC_Trajectory_Testing
  ${catkin_LIBRARIES}
)

