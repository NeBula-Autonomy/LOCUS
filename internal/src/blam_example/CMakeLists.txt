cmake_minimum_required(VERSION 2.8.3)
project(blam_example)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

find_package(catkin REQUIRED)
catkin_package()
