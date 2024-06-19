cmake_minimum_required(VERSION 3.0.2)
project(robothon2023)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  geometry_msgs
  eigen_conversions
  ur_ros_driver
  message_generation
  cv_bridge
  image_transport
)


## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ur_rtde REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(CGAL REQUIRED)
# find_package (ids_peak REQUIRED)
# find_package (ids_peak_ipl REQUIRED)
find_package (Threads REQUIRED)
find_package (OpenCV REQUIRED)


add_service_files(
  FILES
  AddTf2.srv
  GetTriangles.srv
  GetBoardLocation.srv
  GetBeep.srv
  GetFinished.srv
  CheckComponents.srv
  GetRedBlock.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  DEPENDS 
    EIGEN3
    ur_ros_driver
)

include_directories(
  include
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
  ${OpenCV_LIB_DIR} 
)


catkin_install_python(PROGRAMS
  src/triangle_lab.py
  src/board_detection.py
  src/readOscilloscope.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robothon2023.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/robothon2023_node.cpp)

add_executable(${PROJECT_NAME}_point_recording src/point_recording.cpp)
add_executable(${PROJECT_NAME}_touch_detect src/touchDetection.cpp)
add_executable(${PROJECT_NAME}_tf2_publisher src/tf2_publisher.cpp)

add_executable(${PROJECT_NAME}_taskboard src/task_board_tasks.cpp src/task_board_scheduler.cpp)

# add_executable(${PROJECT_NAME}_ids_node src/ids_camera.cpp)

target_link_libraries(${PROJECT_NAME}_point_recording PRIVATE nlohmann_json::nlohmann_json ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_touch_detect PRIVATE nlohmann_json::nlohmann_json CGAL::CGAL ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_tf2_publisher nlohmann_json::nlohmann_json ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_taskboard ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

# target_link_libraries(${PROJECT_NAME}_ids_node ids_peak ids_peak_ipl ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} ${catkin_LIBRARIES} )