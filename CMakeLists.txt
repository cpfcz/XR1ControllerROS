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
   HeadMsgs.msg
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
  #INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)