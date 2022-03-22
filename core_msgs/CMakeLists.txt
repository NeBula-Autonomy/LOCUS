cmake_minimum_required(VERSION 2.8.3)
project(core_msgs)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED)

set(DEPEND_MSGS
  diagnostic_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
 # roadmap_msgs
 # artifact_msgs
)


find_package(catkin REQUIRED COMPONENTS
  ${DEPEND_MSGS}
  message_generation 
)

add_message_files(
  DIRECTORY
    msg
  FILES
    MapInfo.msg
)

#add_service_files(
#    DIRECTORY srv
#    FILES
#    SimCommsThrottle.srv
#)

generate_messages(DEPENDENCIES ${DEPEND_MSGS})

catkin_package(CATKIN_DEPENDS ${DEPEND_MSGS} message_runtime)
