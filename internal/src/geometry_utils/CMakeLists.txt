cmake_minimum_required(VERSION 2.8.3)
project(geometry_utils)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  DEPENDS
    Eigen3
)

include_directories(include ${EIGEN3_INCLUDE_DIR})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY cmake/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.cmake"
)

# add_executable(test_math tests/test_math.cc)
# target_link_libraries(test_math
#   ${catkin_LIBRARIES}
#   boost_unit_test_framework
# )
# 
# add_executable(test_base tests/test_base.cc)
# target_link_libraries(test_base
#   ${catkin_LIBRARIES}
#   boost_unit_test_framework
# )
# 
# add_executable(test_equals tests/test_equals.cc)
# target_link_libraries(test_equals
#   ${catkin_LIBRARIES}
#   boost_unit_test_framework
# )
# 
# add_executable(test_so3error tests/test_so3error.cc)
# target_link_libraries(test_so3error
#   ${catkin_LIBRARIES}
#   boost_unit_test_framework
# )
