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

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xr1controllerros
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include/${PROJECT_NAME}
  XR1ControllerPM_build
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/Eigen3
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/XR1ControllerROS.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/xr1controllerros_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/XR1ControllerPM_build/libXR1ControllerPM.so
 )



add_executable(drawing_vel src/Drawing_Vel.cpp) 
target_link_libraries(drawing_vel ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(drawing_vel ${PROJECT_NAME})