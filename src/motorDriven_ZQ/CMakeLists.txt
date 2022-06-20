cmake_minimum_required(VERSION 3.0.2)
project(motorDriven_ZQ)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  rviz
  urdf
  xacro
  actionlib
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(FILES ControlRelay.msg)
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package( CATKIN_DEPENDS 
   roscpp actionlib message_runtime

)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(cagvinfo
   include/cagvinfo.h
   src/communication/cagvinfo.cpp
 )
 add_library(ccommandinfo
   include/ccommandinfo.h
   src/communication/ccommandinfo.cpp
 )
 add_library(cmotorinfo
   include/cmotorinfo.h
   src/communication/cmotorinfo.cpp
 )
 add_library(cprotocol
   include/cprotocol.h
   src/communication/cprotocol.cpp
 )
 add_library(cupdpsocket
   include/cupdpsocket.h
   src/communication/cupdpsocket.cpp
 )
add_library(JogMotion
  include/JogMotion.h
   src/JogMotion.cpp
)
add_library(CRobotTeachMission
  include/RobotTeachMission.h
   src/RobotTeachMission.cpp
)

add_dependencies(JogMotion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(motorDrivenMain
    src/motorDrivenMain.cpp
    src/JogMotion.cpp
    src/AccDecPlanningMethod.cpp
    src/RobotTeachMission.cpp)
#注意添加的先后顺序
add_dependencies(motorDrivenMain motorDriven_ZQ_generate_messages_cpp) #务必加上这句话，不然自定义消息没法用
target_link_libraries(motorDrivenMain JogMotion cagvinfo ccommandinfo cmotorinfo cprotocol cupdpsocket ${catkin_LIBRARIES})
#############
## Install ##
#############



#############
## Testing ##
#############

