cmake_minimum_required(VERSION 2.8.3)
project(xr1controllerros)

## Compile as C++11, Seriously need this thing
 add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
  tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



 add_message_files(
   FILES
   JointAngles.msg
   IK_msg.msg
   HandMsgs.msg
   JointVisualization.msg
   ArmMsgs.msg
   BodyMsgs.msg
   ChainModeChange.msg
   JointAttributeMsgs.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xr1controllerros
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)



include_directories(
  include/${PROJECT_NAME}
  XR1ControllerPM_build
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/Eigen3
)



 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/XR1ControllerROS.cpp
 )

 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/XR1ControllerPM_build/libXR1ControllerPM.so
 )



add_executable(drawing_vel src/Drawing_Vel.cpp) 
target_link_libraries(drawing_vel ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(drawing_vel ${PROJECT_NAME})


add_executable(ik_example src/IK_Example.cpp) 
target_link_libraries(ik_example ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(ik_example ${PROJECT_NAME})