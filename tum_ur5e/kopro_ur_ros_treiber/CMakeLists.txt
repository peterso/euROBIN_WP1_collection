cmake_minimum_required(VERSION 3.0.2)
project(ur_ros_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_geometry_msgs
  eigen_conversions
  message_generation
  geometry_msgs
  sensor_msgs
  std_msgs
  controller_manager
  hardware_interface
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  GripperInfo.msg
  PathEntry.msg
  JogControl.msg
  DashboardInfo.msg
)

## Generate services in the 'srv' folder
add_service_files( 
  FILES
  SetForceTarget.srv
  SetContactTarget.srv
  SetFreedrive.srv 
  SetCartTarget.srv
  SetRelCartTarget.srv
  SetTrajectory.srv
  SetJointTarget.srv
  SetGripper.srv 
  GetGripperCalib.srv
  SetSpeedSlider.srv
  StartJog.srv
  Power.srv
  Log.srv
  SetPayload.srv
  GetTimeStamp.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs   
  sensor_msgs   
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(ur_rtde REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs 
    sensor_msgs 
    std_msgs
    hardware_interface
  DEPENDS 
    EIGEN3
)

include_directories(
  include
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur_ros_driver_node
  src/hw_interface.cpp
  src/dashboard_client.cpp
  src/robotiq_gripper_interface.cpp
)

target_link_libraries(ur_ros_driver_node PRIVATE ur_rtde::rtde ${catkin_LIBRARIES})