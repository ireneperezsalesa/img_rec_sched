cmake_minimum_required(VERSION 3.0.2)
project(img_rec_sched)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  dvs_msgs
  image_transport
)
find_package(catkin_simple REQUIRED)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Trigger.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

#catkin_simple()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES img_rec
  CATKIN_DEPENDS rpg_dvs_ros dvs_msgs message_runtime 
#  DEPENDS system_lib
)

include_directories(
 #include
 ${catkin_INCLUDE_DIRS}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
  launch/img_rec.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}) # Not needed for CMake >= 2.8.11

add_executable(scheduler src/scheduler.cpp)
target_link_libraries(scheduler
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(img_rec src/img_rec.cpp)
target_link_libraries(img_rec
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(scheduler img_rec_sched_gencpp)
add_dependencies(scheduler img_rec_sched_generate_messages_cpp)
add_dependencies(img_rec img_rec_sched_gencpp)
